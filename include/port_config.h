#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#include <vex.h>

namespace port{
	using namespace vex;

	#define LEFT_MOTOR1      PORT10
	#define LEFT_MOTOR2      PORT6
	#define LEFT_MOTOR3		 PORT2   

	#define RIGHT_MOTOR1     PORT16 
	#define RIGHT_MOTOR2     PORT17 
	#define RIGHT_MOTOR3     PORT19 

	#define INTAKE           PORT4

	#define LB1				 PORT8 
	#define LB2				 PORT18 

	#define COLOR 			 PORT3
	#define LB_DISTANCE  PORT5

	#define RADIO            PORT20
};
#endif