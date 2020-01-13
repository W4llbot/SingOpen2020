#ifndef _BASE_LIB_HPP_
#define _BASE_LIB_HPP_

//Tasks for controlling base movements
void baseOdometry(void * ignore);     //Calculates coordinates using odometry
void baseMotorControl(void * ignore); //Ramping for motors
void baseControl(void * ignore);      //Motor control using PD Loop

void waitBase(double cutoff);            //While loop to wait for base to finish

//Straight moves in inches
void baseMove(double dis);  //Default PD
void baseMove(double dis, double p, double d);

//Calculate straight moves using coordinates
void baseMove(double x, double y);
void baseMove(double x, double y, double p, double d);

//Turning using global angle
void baseTurn(double angle);
void baseTurn(double angle, double p, double d);

//Turning using relative angle
void baseTurnRelative(double angle);
void baseTurnRelative(double angle, double p, double d);

//Turning towards point using global angle
void baseTurn(double x, double y);
void baseTurn(double x, double y, double p, double d);
void baseTurn(double x, double y, bool inverted);
void baseTurn(double x, double y, double p, double d, bool inverted);

void resetCoord(double x, double y, double angleInDeg);

double getY();
void baseStraight(bool state);
void timerBase(double powerL, double powerR, double time);
void pauseBase(bool pause);
void base(double left, double right);

#endif
