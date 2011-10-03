#define deg 0.017

#include <stddef.h>
#include "stm32f10x.h"
#include "physix_m.h"
#include <math.h>
#include "main.h"
#include <stdlib.h>



float v0_x=0, v0_y=0, x_0=160, y_0=120, g=10, alpha_x=0, alpha_y=0;
float a_x,a_y;
float Vx=0, Vy=0, rebound=0.5f, stop=1.0f;
uint8_t bouncedX=0, bouncedY=0;


void get_pos(float dt, int tilt_x, int tilt_y, uint16_t* posX, uint16_t* posY)
{

	dt=dt/30;

    alpha_x=(tilt_x*90)/70;
    alpha_y=(tilt_y*90)/70;

    a_x = g * sin(alpha_x*deg);
    a_y = g * sin(alpha_y*deg);

	*posX += v0_x*dt + (a_x*((dt)*(dt)))/2;
	*posY += v0_y*dt + (a_y*((dt)*(dt)))/2;
	


	if(*posX+BALL_SIZE >= 319)
	{
		v0_x = -v0_x*rebound + (a_x * dt);
		*posX=319-BALL_SIZE;
		bouncedX=1;
	}
	else if(*posX-BALL_SIZE <    0)
	{
		v0_x = -v0_x*rebound + (a_x * dt);
		*posX=BALL_SIZE;
		bouncedX=1;
	}
	else if(bouncedX && abs(v0_x)<stop)
		{
			v0_x=0;
			bouncedX=0;
		}
	else
	{
		v0_x += (a_x * dt);
	}
	if(*posY+BALL_SIZE >= 239)
	{
		v0_y = -v0_y*rebound + (a_y * dt);
		*posY=239-BALL_SIZE;
		bouncedY=1;
	}
	else if(*posY-BALL_SIZE <    0)
	{
		v0_y = -v0_y*rebound + (a_y * dt);
		*posY=BALL_SIZE;
		bouncedY=1;
	}
	else if(bouncedY && abs(v0_y)<stop)
	{
		v0_y=0;
		bouncedY=0;
	}
	else
	{
		v0_y += (a_y * dt);
	}


}
