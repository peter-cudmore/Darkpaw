#include <stdio.h>
#include "sensors.h"
#include "pigpio/pigpio.h"
#include <unistd.h>

#define MPU6050_ADDRESS 0x68

#define GRAVITIY_MS2  9.80665

#define ACCEL_SCALE_MODIFIER_2G 16384.0
#define ACCEL_SCALE_MODIFIER_4G  8192.0
#define ACCEL_SCALE_MODIFIER_8G  4096.0
#define ACCEL_SCALE_MODIFIER_16G  2048.0

#define GYRO_SCALE_MODIFIER_250DEG  131.0
#define GYRO_SCALE_MODIFIER_500DEG  65.5
#define GYRO_SCALE_MODIFIER_1000DEG  32.8
#define GYRO_SCALE_MODIFIER_2000DEG  16.4


#define ACCEL_RANGE_2G  0x00
#define ACCEL_RANGE_4G  0x08
#define ACCEL_RANGE_8G  0x10
#define ACCEL_RANGE_16G 0x18

#define GYRO_RANGE_250DEG  0x00
#define GYRO_RANGE_500DEG  0x08
#define GYRO_RANGE_1000DEG  0x10
#define GYRO_RANGE_2000DEG  0x18

// # MPU - 6050 Registers
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2  0x6C

#define ACCEL_XOUT0  0x3B
#define ACCEL_YOUT0  0x3D
#define ACCEL_ZOUT0  0x3F

#define TEMP_OUT0  0x41

#define GYRO_XOUT0 0x43
#define GYRO_YOUT0 0x45
#define GYRO_ZOUT0 0x47

#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B


static int driver_fp = -1;

int init_mpu6050(void) {
    i2cWriteByteData(driver_fp, PWR_MGMT_1, 0x00);
    
    // set the acceleromter range to min.
    i2cWriteByteData(driver_fp, ACCEL_CONFIG, 0x00);
    i2cWriteByteData(driver_fp, ACCEL_CONFIG, ACCEL_RANGE_2G);

    // set the gyro range to the min aswell
    i2cWriteByteData(driver_fp, GYRO_CONFIG, 0x00);
    i2cWriteByteData(driver_fp, GYRO_CONFIG, GYRO_RANGE_250DEG);

    return 0;
};


int init_sensors(void) {
    if ((driver_fp = i2cOpen(1, MPU6050_ADDRESS, 0)) < 0) {
        // failed to get i2c bus
        fprintf(stderr, "Failed to get i2c device: MPU6050\n");
        return -1;
    }
	return 0;
};

void close_sensors(void) {
    if ((driver_fp > 0) && (i2cClose(driver_fp) != 0)) {
        fprintf(stderr, "Failed to close i2c device: MPU6050\n");
    }
};
