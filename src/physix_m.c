#define deg 0.017

#include "physix_m.h"
#include <math.h>
#include "main.h"
#include <stddef.h>
#include "stm32f10x.h"

float v0_x=0, v0_y=0, x_0=160, y_0=120, g=10, alpha_x=0, alpha_y=0;
float a_x,a_y;
float Vx=0, Vy=0, lAx=0, lAy=0, rebound=0.5f;
int pixelsPerMeter=500;

void get_pos(float dt, int tilt_x, int tilt_y, uint16_t* posX, uint16_t* posY)
{

	dt=dt/30;

    alpha_x=(tilt_x*90)/70;
    alpha_y=(tilt_y*90)/70;

    a_x = g * sin(alpha_x*deg);
    a_y = g * sin(alpha_y*deg);

	*posX += v0_x*dt + (a_x*((dt)*(dt)))/2;
	*posY += v0_y*dt + (a_y*((dt)*(dt)))/2;
	


	if     (*posX+BALL_SIZE >= 319)
	{
		v0_x = -v0_x*rebound;
		*posX=319-BALL_SIZE;
	}
	else if(*posX-BALL_SIZE <    0)
	{
		v0_x = -v0_x*rebound;
		*posX=BALL_SIZE;
	}
	if     (*posY+BALL_SIZE >= 239)
	{
		v0_y = -v0_y*rebound;
		*posY=239-BALL_SIZE;
	}
	else if(*posY-BALL_SIZE <    0)
	{
		v0_y = -v0_y*rebound;
		*posY=BALL_SIZE;
	}

	v0_x += (a_x * dt);
	v0_y += (a_y * dt);

}

void get_pos2(int dt, int tilt_x, int tilt_y, int* posX, int* posY)
{

	lAx=tilt_x;
	lAy=tilt_y;

	Vx += ((dt * lAx) / 1000) * pixelsPerMeter;
	Vy += ((dt * lAy) / 1000) * pixelsPerMeter;

	        // update the position
	        // (velocity is meters/sec, so divide by 1000 again)
	*posX += ((Vx * dt) / 1000) * pixelsPerMeter;
	*posY += ((Vy * dt) / 1000) * pixelsPerMeter;

	  if(*posX+BALL_SIZE >= 319) { Vx = -60; *posX=319-BALL_SIZE; }
	  if(*posX-BALL_SIZE <=   0) { Vx = 60; *posX=BALL_SIZE;   }
	  if(*posY+BALL_SIZE >= 239) { Vy = -60; *posY=239-BALL_SIZE; }
	  if(*posY-BALL_SIZE <=   0) { Vy = 60; *posY=BALL_SIZE; }

	        int bouncedX = 0;
	        int bouncedY = 0;
	     /*   float STOP_BOUNCING_VELOCITY=2.0f;

	        if (*posY - BALL_SIZE <= 1) {
	        	*posY = BALL_SIZE;
	            Vy = -Vy * rebound;
	            bouncedY = 1;
	        } else if (*posY + BALL_SIZE >= 239) {
	            *posY = 239 - BALL_SIZE;
	            Vy = -Vy * rebound;
	            bouncedY = 1;
	        }
	        if (bouncedY && abs(Vy) < STOP_BOUNCING_VELOCITY) {
	            Vy = 0;
	            bouncedY = 0;
	        }

	        if (*posX - BALL_SIZE <= 1) {
	        	*posX = BALL_SIZE;
	        	Vx = -Vx * rebound;
	        	bouncedX = 1;
	        } else if (*posX + BALL_SIZE >= 319) {
	            *posX = 319 - BALL_SIZE;
	            Vx = -Vx * rebound;
	            bouncedX = 1;
	        }
	        if (bouncedX && abs(Vx) < STOP_BOUNCING_VELOCITY) {
	        	Vx = 0;
	        	bouncedX = 0;
	        }*/
}
