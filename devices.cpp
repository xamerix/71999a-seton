#include "main.h"

int lF = 8;
int lBt = 7;
int lBb = 6;
int rF = 16;
int rBt = 5;
int rBb = 14;
int cata1Port = 17;
int cata2Port = 9;
Motor leftFront(lF,E_MOTOR_GEARSET_06, true);
Motor leftBackTop(lBt, E_MOTOR_GEARSET_06, false); 
Motor leftBackBottom(lBb, E_MOTOR_GEARSET_06, true);
Motor rightFront(rF, E_MOTOR_GEARSET_06, false);
Motor rightBackTop(rBt, E_MOTOR_GEARSET_06, true);
Motor rightBackBottom(rBb, E_MOTOR_GEARSET_06, false);
Motor cataMotor1(cata1Port, E_MOTOR_GEARSET_36, true);
Motor cataMotor2(cata2Port, E_MOTOR_GEARSET_36, false);
Rotation cataRotate(4,true);
ADIDigitalOut clamper('A');
ADIDigitalOut wings('E');
ADIDigitalOut blocker('F');
ADIEncoder vertWheel('C','D',true);
ADIEncoder horizWheel('G','H',true);
ADIDigitalIn cataLimit('B');