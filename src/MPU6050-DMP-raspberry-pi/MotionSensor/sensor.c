#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "helper_3dmath.h"
#include "../MotionSensor.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "sensor.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define delay_ms(a)    usleep(a*1000)

// MPU control/status vars
uint8_t devStatus;      // return status after each device operation
//(0 = success, !0 = error)
uint8_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t a[3];              // [x, y, z]            accel vector
int16_t g[3];              // [x, y, z]            gyro vector
int32_t _q[4];
int32_t t;
int16_t c[3];

VectorFloat gravity;    // [x, y, z]            gravity vector

int r;
int initialized = 0;
int dmpReady = 0;
float lastval[3];
int16_t sensors;

float ypr[3];
Quaternion q; 
float temp;
float gyro[3];
float accel[3];
float compass[3];

uint8_t rate = 40;

int ms_open() 
	{
	dmpReady=1;
	initialized = 0;
	for (int i=0;i<DIM;i++){
		lastval[i]=10;
	}

	// initialize device
	printf("Init MPU6050...\n");
	if (mpu_init(NULL) != 0) {
		printf("MPU init fail.\n");
		return -1;
	}
	printf("MPU init suceed.");

	printf("Set up MPU6050...\n");
        dump_init();
	if (mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("MPU6050 setup fail...\n");
		return -1;
	}
	printf("MPU6050 setup succed.\n");

	printf("Setup Gyro...\n");
	if (mpu_set_gyro_fsr(2000)!=0) {
		printf("Gyro setup fail.\n");
		return -1;
	}
	printf("Gyro setup suceed.\n");

	printf("Setup acc...\n");
	if (mpu_set_accel_fsr(2)!=0) {
		printf("Setup acc fail\n");
		return -1;
	}
	printf("Acc setup suceed.\n");

	// verify connection
	printf("Power MPU6050...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU6050 Connect!\n" : "MPU6050 connect fail. %u\n",devStatus);

	//fifo config
	printf("Setup MPU6050 FIFO...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("MPU6050 fifo fail.\n");
		return -1;
	}
	printf("MPU fifo suceed.\n");


	// load and configure the DMP
	printf("Read DMP...\n");
	if (dmp_load_motion_driver_firmware()!=0) {
		printf("DMP open fail.\n");
		return -1;
	}
	printf("DMP suceed!\n");


	printf("Init DMP...\n");
	if (mpu_set_dmp_state(1)!=0) {
		printf("DMP init fail.\n");
		return -1;
	}
	printf("DMP init suceed.\n");

	//dmp_set_orientation()
	//if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
	printf("Setup DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0) {
		printf("DMP feature extract fail.\n");
		return -1;
	}
	printf("DMP feature extract suceed.\n");

	printf("Setup DMP FIFO sample rate...\n");
	if (dmp_set_fifo_rate(rate)!=0) {
		printf("DMP FIFO sample rate fail...\n");
		return -1;
	}
	printf("DMP FIFO samplerate set!.\n");


	printf("Reset FIFO...\n");
	if (mpu_reset_fifo()!=0) {
		printf("FIFO setup fail.\n");
		return -1;
	}
	printf("FIFO setup suceed.\n");

	printf("Measure... ");
	do {
		delay_ms(1000/rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
		r=dmp_read_fifo(g,a,_q,&sensors,&fifoCount);
	} while (r!=0 || fifoCount<5); //packtets!!!
	printf("Done.\n");

	initialized = 1;
	return 0;
}

int ms_update() {
	if (!dmpReady) {
		printf("ERRO: DMP .\n");
		return -1;
	}

	while (dmp_read_fifo(g,a,_q,&sensors,&fifoCount)!=0); //gyro and accel can be null because of being disabled in the efeatures
	q = _q;
	GetGravity(&gravity, &q);
	GetYawPitchRoll(ypr, &q, &gravity);

	mpu_get_temperature(&t);
	temp=(float)t/65536L;

	mpu_get_compass_reg(c);

	//scaling for degrees output
	for (int i=0;i<DIM;i++){
		ypr[i]*=180/M_PI;
	}

	//unwrap yaw when it reaches 180
	ypr[0] = wrap_180(ypr[0]);

	//change sign of Pitch, MPU is attached upside down
	ypr[1]*=-1.0;

	//0=gyroX, 1=gyroY, 2=gyroZ
	//swapped to match Yaw,Pitch,Roll
	//Scaled from deg/s to get tr/s
	for (int i=0;i<DIM;i++){
		gyro[i]   = (float)(g[DIM-i-1])/131.0/360.0;
		accel[i]   = (float)(a[DIM-i-1]);
		compass[i] = (float)(c[DIM-i-1]);
	}

	return 0;
}

int ms_close() {
	return 0;
}

uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
	v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
	v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
	v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
	return 0;
}

uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
	// yaw: (about Z axis)
	data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
	return 0;
}

