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
bool runFlip(setpoint_t *setpoint,int flip_type) //the runflip function is executed at 100 Hertz
{
	if(flip_timer==0 && flip_state==0) //flip init check
	{
		//flip_state:
		//0: roll flip
		//10: pitch flip
		//20: evasive turn

		//flip_type:
		//0: switch down
		//1: switch middle
		//2: switch up


		if(flip_type==0) //enable correct flip type
		{
			flip_state = 20;
		}
		
		if(flip_type==1)
		{
			flip_state = 0;
		}
		
		if(flip_type==2)
		{
			flip_state = 10;
		}
		
		
	}
	
	switch(flip_state)
	{
			
			//roll flip
		case 0: // lift up
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

			
			//pitch flip
		case 10: //lift up
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
				flip_state = 11;
			}

		break;


		case 11: //set pitch rate
			setpoint->thrust = 40000;
			
			setpoint->thrust = 40000;
			
			setpoint->mode.pitch = modeVelocity;
			setpoint->attitudeRate.pitch = 900;
			setpoint->attitude.pitch = 0;
			
			setpoint->mode.roll = modeVelocity;
			setpoint->attitudeRate.roll = 0;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeDisable;
			setpoint->attitude.yaw = 0;
			
			flip_timer++;
			
			if(flip_timer>65)
			{
				flip_timer = 0;
				flip_state = 12;
			}



		break;


		case 12: //stabilize
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
			
			if(flip_timer>20)
			{
				flip_timer = 0;
				flip_state = 100;
			}



		break;


	
	//sharp turn
	case 20: //accelerate forward
			setpoint->thrust = 35000;
			
			setpoint->mode.pitch = modeAbs;
			setpoint->attitudeRate.pitch = 0;
			setpoint->attitude.pitch = -50;
			
			setpoint->mode.roll = modeAbs;
			setpoint->attitudeRate.roll = 0;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeVelocity;
			setpoint->attitude.yaw = 0;
			
			
			flip_timer++;
			
			if(flip_timer>40)
			{
				flip_timer = 0;
				flip_state = 21;
			}

		break;
			
			
		case 21: //roll pitch sideways
			setpoint->thrust = 38000;
			
			setpoint->mode.pitch = modeVelocity;
			setpoint->attitudeRate.pitch = -450;
			setpoint->attitude.pitch = 0;
			
			setpoint->mode.roll = modeVelocity;
			setpoint->attitudeRate.roll = 400;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeDisable;
			setpoint->attitude.yaw = 0;
			
			
			flip_timer++;
			
			if(flip_timer>30)
			{
				flip_timer = 0;
				flip_state = 23;
			}
			break;
			
			
		case 22: //stabilize the velocity, forward flight
			setpoint->thrust = 38000;
			
			setpoint->mode.pitch = modeVelocity;
			setpoint->attitudeRate.pitch = 0;
			setpoint->attitude.pitch = 0;
			
			setpoint->mode.roll = modeVelocity;
			setpoint->attitudeRate.roll = 0;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeVelocity;
			setpoint->attitude.yaw = 0;
			
			flip_timer++;
			
			if(flip_timer>20)
			{
				flip_timer = 0;
				flip_state = 23;
			}
			break;
					case 23: //stabilize, forward flight
			setpoint->thrust = 37000;
			
			setpoint->mode.pitch = modeAbs;
			setpoint->attitudeRate.pitch = 0;
			setpoint->attitude.pitch = -30;
			
			setpoint->mode.roll = modeAbs;
			setpoint->attitudeRate.roll = 0;
			setpoint->attitude.roll = 0;
			
			setpoint->mode.yaw = modeVelocity;
			setpoint->attitude.yaw = 0;
			
			flip_timer++;
			
			if(flip_timer>30)
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
	else //end of maneouvre
	{
		flip_timer = 0;
		flip_state = 0;
		return true;
	}
}
