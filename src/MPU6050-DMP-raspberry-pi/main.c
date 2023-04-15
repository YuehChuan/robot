#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "MotionSensor.h"

#define delay_ms(a) usleep(a*1000)

int main() 
{
	//init DMP
	ms_open();
	float yaw_offset=2.5;
	float pitch_offset=4.3;
	float roll_offset=0.3;
	float temp_offset=7.0;
	do{
		ms_update();
		printf("yaw = %2.1f\tpitch = %2.1f\troll = %2.1f\ttemperature = %2.1f\tcompass = %2.1f, %2.1f, %2.1f\n",
		 ypr[YAW]+yaw_offset, ypr[PITCH]+pitch_offset, ypr[ROLL]+roll_offset, temp-temp_offset, compass[0], compass[1], compass[2]);
		delay_ms(5);
	}while(1);

	return 0;
}
