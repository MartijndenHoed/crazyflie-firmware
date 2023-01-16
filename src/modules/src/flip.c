//#include <flip.h>
#include <math.h>
#include <stdbool.h>

#include "crtp_commander.h"
#include "config.h"
#include "commander.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "num.h"
#include "position_controller.h"


static int flip_timer = 0;
static int flip_state = 0;


//runflip returns false while it is running and true when done
bool runFlip(setpoint_t *setpoint) //the runflip function is executed at 100 Hertz
{
	
	switch(flip_state)
	{
		case 0: //lift up
			setpoint->thrust = 55000;
			
			setpoint->mode.pitch = modeAbs;
			setpoint->attitudeRate.pitch = 0;
			setpoint->attitude.pitch = 0;
			
			setpoint->mode.roll = modeAbs;
			setpoint->attitudeRate.roll = 0;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeVelocity;
			setpoint->attitude.yaw = 0;
			
			
			flip_timer++;
			
			if(flip_timer>60)
			{
				flip_timer = 0;
				flip_state = 1;
			}

		break;


		case 1: //set roll rate
			setpoint->thrust = 40000;
			
			setpoint->thrust = 40000;
			
			setpoint->mode.pitch = modeAbs;
			setpoint->attitudeRate.pitch = 0;
			setpoint->attitude.pitch = 0;
			
			setpoint->mode.roll = modeVelocity;
			setpoint->attitudeRate.roll = 900;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeVelocity;
			setpoint->attitude.yaw = 0;
			
			flip_timer++;
			
			if(flip_timer>55)
			{
				flip_timer = 0;
				flip_state = 2;
			}



		break;


		case 2: //stabilize
			setpoint->thrust = 55000;
			
			setpoint->mode.pitch = modeAbs;
			setpoint->attitudeRate.pitch = 0;
			setpoint->attitude.pitch = 0;
			
			setpoint->mode.roll = modeAbs;
			setpoint->attitudeRate.roll = 0;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeVelocity;
			setpoint->attitude.yaw = 0;
			
			flip_timer++;
			
			if(flip_timer>40)
			{
				flip_timer = 0;
				flip_state = 100;
			}



		break;






	}


	if(flip_state!=100)
	{
		return false;
	}
	else
	{
		flip_timer = 0;
		flip_state = 0;
		return true;
	}
}
