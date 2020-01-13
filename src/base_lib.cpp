#include "main.h"
#include "base_lib.hpp"

Motor FL (FLPort);
Motor BL (BLPort);
Motor FR (FRPort);
Motor BR (BRPort);

#define COMPETITION_MODE true //Set to false to test odometry during driver

/*-----------------------------------------USER INPUT-----------------------------------------*/
#define baseRPM 200									//Enter RPM of base motors
#define UNLIMITED_POWER 90         //Enter maximum power of base motors
#define rampingPow 5              	//Increase in power / 25 millis

#define PI      3.141592653589793238462643383279			//CONST PI VALUE
#define halfPI  1.570796326794896619231321691639 			//CONST PI/2 VALUE
#define twoPI	  6.283185307179586476925286766559			//CONST 2*PI VALUE

#define DISTANCE_LEEWAY   3.0				//Enter Distance Leeway in Degrees

#define baseWidth 12.52198690477777	//Enter distance in inches between side encoders
#define inPerDeg 0.0354472057977424 //Determine empirically using at least 1 rotation
/*--------------------------------------------------------------------------------------------*/

typedef struct{
	double x,y,angle;
} Coordinates;

Coordinates position;

double targetL=0,targetR=0;
double kP=0,kI=0,kD=0;
double lastResetAngle=0; //IN RADIANS

double encdL=0,encdR=0;

double motorTargetL=0,motorTargetR=0;

void baseOdometry(void * ignore){
	Controller master(E_CONTROLLER_MASTER);

  FL.tare_position();
	FR.tare_position();
	BL.tare_position();
	BR.tare_position();

  double prevEncdL = 0;
	double prevEncdR = 0;
	double prevAngle = 0;
	int count = 0;
	while(true){ //!COMPETITION_MODE || competition::is_autonomous()
    encdL = BL.get_position();
    encdR = BR.get_position();
		double encdChangeL = (encdL - prevEncdL)*inPerDeg;
		double encdChangeR = (encdR - prevEncdR)*inPerDeg;
		position.angle = lastResetAngle + (encdL - encdR)*inPerDeg/baseWidth; //Theta = (s2-s1)/width
		double deltaAngle = position.angle - prevAngle;
		double sumEncdChange = encdChangeL+encdChangeR;

		if(deltaAngle == 0) //Cannot divide by 0
		{
			position.x += sumEncdChange/2*sin(position.angle);            	//Simple trigo
			position.y += sumEncdChange/2*cos(position.angle);
		}
		else                //Refer to formulas
		{
			double halfDeltaAngle = deltaAngle/2;
			double strDist = (sumEncdChange/deltaAngle)*sin(halfDeltaAngle);
			position.x += strDist * sin(prevAngle+halfDeltaAngle);
			position.y += strDist * cos(prevAngle+halfDeltaAngle);
		}

		prevEncdL = encdL;
		prevEncdR = encdR;
		prevAngle = position.angle;

		master.print(2,0,"%2.1f %2.1f %3f",position.x,position.y,position.angle/PI*180.0);
		if(count++ % 10 == 0) printf("%2.1f \t %2.1f \t %3f\n",position.x,position.y,position.angle/PI*180.0);

    Task::delay(5);
	}
}

bool basePause = false;
void pauseBase(bool pause){
	basePause = pause;
}
void baseMotorControl(void * ignore){
  double powerL=0,powerR=0;
	//printf("Left \t Right\n");
  while(competition::is_autonomous()){
    double diffL = motorTargetL - powerL;
    if(fabs(diffL) > rampingPow) powerL += diffL>0?rampingPow:-rampingPow;
    else powerL = motorTargetL;


    double diffR = motorTargetR - powerR;
    if(fabs(diffR) > rampingPow) powerR += diffR>0?rampingPow:-rampingPow;
    else powerR = motorTargetR;

    if(powerL > UNLIMITED_POWER) powerL = UNLIMITED_POWER;
    else if(powerL < -UNLIMITED_POWER) powerL = -UNLIMITED_POWER;
    if(powerR > UNLIMITED_POWER) powerR = UNLIMITED_POWER;
    else if(powerR < -UNLIMITED_POWER) powerR = -UNLIMITED_POWER;

		if(!basePause){
			FL.move(powerL);
			BL.move(powerL);
			FR.move(powerR);
			BR.move(powerR);
		}
		//printf("%4.0f \t %4.0f\n",powerL,powerR);

    Task::delay(15);
  }
}

void timerBase(double powerL, double powerR, double time){
	double start = millis();
	pauseBase(true);
	FL.move(powerL);
	BL.move(powerL);
	FR.move(powerR);
	BR.move(powerR);
	while(millis()-start<time)delay(25);
	FL.move(0);
	BL.move(0);
	FR.move(0);
	BR.move(0);
	pauseBase(false);
}

bool forceStraight = false;
void baseStraight(bool state){
	 forceStraight = state;
}

void baseControl(void * ignore){
  double prevErrorL = 0, prevErrorR = 0;
  while(competition::is_autonomous()){
    double errorL = targetL - BL.get_position();
    double errorR = targetR - BR.get_position();

		if(!forceStraight){
	    double deltaErrorL = errorL - prevErrorL;
	    double deltaErrorR = errorR - prevErrorR;

	    prevErrorL = errorL;
	    prevErrorR = errorR;

	    motorTargetL = kP*errorL+kD*deltaErrorL;
	    motorTargetR = kP*errorR+kD*deltaErrorR;
		}
		else{
			double combErrorP = (errorL+errorR)/2*kP;

			prevErrorL = errorL;
	    prevErrorR = errorR;

			motorTargetL = errorL>0?combErrorP:-combErrorP;
	    motorTargetR = errorR>0?combErrorP:-combErrorP;
		}

		//printf("Error: %f %f\n",errorL,errorR);

    Task::delay(15);
  }
}



void waitBase(double cutoff){
	double start = millis();
	while(fabs(targetL-BL.get_position())>DISTANCE_LEEWAY && fabs(targetR-BR.get_position())>DISTANCE_LEEWAY && millis()-start < cutoff){
		//printf("TL:%3f, FL:%3f, TR:%3f, FR:%3f\n",targetL,FL.get_position(),targetR,FR.get_position());
		//printf("~\t  \t%4.0f \t %4.0f\n",targetL-FL.get_position(),targetR-FR.get_position());
		delay(25);
	}
	FL.move_relative(0, 100);
	BL.move_relative(0, 100);
	FR.move_relative(0, 100);
	BR.move_relative(0, 100);
}

void baseMove(double dis, double p, double d){
  targetR += dis/inPerDeg;
	targetL += dis/inPerDeg;

  kP = p;
  kD = d;
}

void baseMove(double dis){
  baseMove(dis,0.40,1.4);
}

void baseMove(double x, double y, double p, double d){
	double errY = y-position.y;
	double errX = x-position.x;
	double distance = sqrt(errY*errY + errX*errX);
	double targAngle = atan2(errX,errY);
	//printf("Angle: %f %f", targAngle, position.angle);
	//printf("Distance: %f",distance);

	int negator = 1;
  if(fabs(targAngle-position.angle) >= PI/2) negator = -1;

  targetL += distance/inPerDeg*negator;
  targetR += distance/inPerDeg*negator;

  kP = p;
  kD = d;
}

void baseMove(double x, double y){
	baseMove(x,y,0.5,1.3);
}

void baseTurn(double angle, double p, double d){
	double error = angle/180.00000*PI - position.angle;
	double diff = error*baseWidth/inPerDeg/2.0000;
	targetL += diff;
	targetR += -diff;

	kP = p;
	kD = d;
}

void baseTurn(double angle){
  baseTurn(angle, 0.4, 0.0);
}

void baseTurnRelative(double angle, double p, double d){
  double diff = angle/180.00000*PI*baseWidth/inPerDeg/2;
  targetL += diff;
  targetR += -diff;

  kP = p;
  kD = d;
}

void baseTurnRelative(double angle){
  baseTurnRelative(angle, 0.7, 0.3);
}

void baseTurn(double x, double y, double p, double d){
	double targAngle = atan2((x-position.x),(y-position.y));
  double diff = (targAngle - position.angle)*baseWidth/inPerDeg/2;
  targetL += diff;
  targetR += -diff;
  kP = p;
  kD = d;
}

void baseTurn(double x, double y, double p, double d, bool inverted){
	double targAngle = atan2((x-position.x),(y-position.y));
	printf("HI: %f\n",targAngle);
	if(inverted) targAngle += PI;
	printf("HI: %f\n",targAngle);
	if(targAngle-position.angle > PI) targAngle -= twoPI;
	printf("HI: %f\n",targAngle);
  double diff = (targAngle - position.angle)*baseWidth/inPerDeg/2;
  targetL += diff;
  targetR += -diff;
  kP = p;
  kD = d;
}

void baseTurn(double x, double y){
  baseTurn(x,y,0.7,0.3);
}

void baseTurn(double x, double y, bool inverted){
  baseTurn(x,y,0.7,0.3,inverted);
}

void resetCoord(double x, double y, double angleInDeg){
	position.x = x;
	position.y = y;
	position.angle = angleInDeg/180*PI;
	lastResetAngle = angleInDeg/180*PI;

	FL.tare_position();
	FR.tare_position();
	BL.tare_position();
	BR.tare_position();

	targetL = 0;
	targetR = 0;
}

double getY(){
	return position.y;
}

void base(double left, double right) {
	FL.move(left);
	BL.move(left);
	FR.move(right);
	BR.move(right);
}
