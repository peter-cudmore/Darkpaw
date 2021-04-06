#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "servos.h"
#include "pigpio/pigpio.h"


#define PCA9685_ADDRESS 0x40
#define MODE1           0x00
#define MODE2           0x01
#define SUBADR1         0x02
#define SUBADR2         0x03
#define SUBADR3         0x04
#define PRESCALE        0xFE
#define LED0_ON_L       0x06
#define LED0_ON_H       0x07
#define LED0_OFF_L      0x08
#define LED0_OFF_H      0x09
#define ALL_LED_ON_L    0xFA
#define ALL_LED_ON_H    0xFB
#define ALL_LED_OFF_L   0xFC
#define ALL_LED_OFF_H   0xFD

// Bits:
#define RESTART         0x80
#define SLEEP           0x10
#define ALLCALL         0x01
#define INVRT           0x10
#define OUTDRV          0x04
#define SWRST           0x06
#define RESET           0x00


#define CHANNEL_MAX 12

#define INVALID_SERVO_COMMAND -1;

static int driver_fp = -1;


int set_all_pwm(u16 on, u16 off) {
    return (i2cWriteByteData(driver_fp, ALL_LED_ON_L, (u8) on & 0xFF)
            | i2cWriteByteData(driver_fp, ALL_LED_ON_H,  (u8) on >> 8)
            | i2cWriteByteData(driver_fp, ALL_LED_OFF_L, (u8) off & 0xFF)
            | i2cWriteByteData(driver_fp, ALL_LED_OFF_L, (u8) off >> 0)
	   );   
}

int set_pwm_freq(float freq_hz) {
    float prescaleval = 25000000.0 / 4096.0 ;    // 25MHz
    unsigned prescale;
    unsigned oldmode, newmode;
    int result;

    prescaleval /= freq_hz;
    prescaleval -= 1.0;
    prescale = (unsigned)(prescaleval + 0.5);
    
    oldmode = i2cReadByteData(driver_fp, MODE1);
    newmode = (oldmode & 0x7F) | SLEEP;
    result = i2cWriteByteData(driver_fp, MODE1, newmode) 
        | i2cWriteByteData(driver_fp, PRESCALE, prescale) 
        | i2cWriteByteData(driver_fp, MODE1, oldmode);
    
    if (result < 0) {
        return -1;
    }
 
    usleep(1000);

    return i2cWriteByteData(driver_fp, MODE1, oldmode | RESTART);
}

int set_pwm(unsigned channel, u16 on, u16 off) {
    if ((off < SERVO_MIN) || (off > SERVO_MAX) || (channel >= CHANNEL_MAX))
        return INVALID_SERVO_COMMAND;

    return (
        i2cWriteByteData(driver_fp, LED0_ON_L + 4 * channel, (u8) (on & 0xFF))
        | i2cWriteByteData(driver_fp, LED0_ON_H + 4 * channel, (u8) (on >> 8))
        | i2cWriteByteData(driver_fp, LED0_OFF_L + 4 * channel, (u8) (off & 0xFF))
        | i2cWriteByteData(driver_fp, LED0_OFF_H + 4 * channel, (u8) (off >> 8))
        );
}


static int init_PCA9685(void) {
    
    int result, error_code;
    unsigned byte;
    
    if ((error_code = set_all_pwm(0, 0)) != 0) {
        fprintf(stderr, "Failed to set PWM values: Error %i", error_code);
        return error_code;
    }
    
    if ((error_code = i2cWriteByteData(driver_fp, MODE2, OUTDRV) | i2cWriteByteData(driver_fp, MODE1, ALLCALL)) != 0) {
        fprintf(stderr, "Failed to set PCA9685 mode: Error %i", error_code);
        return error_code;
    }
    
    usleep(1000);
    if ((result = i2cReadByteData(driver_fp, MODE1)) < 0) {
        // error
        return -1;
    }
    
    byte = (unsigned)result;
    
    if (i2cWriteByteData(driver_fp, MODE1, byte & !SLEEP) < 0) {
        return -1;
    };
    usleep(1000);
 
    return set_pwm_freq(50.0);
};

int set_pwms(u16* values){
    for (int i =0; i < CHANNEL_MAX; i++){
        if (set_pwm(i, 0, values[i]) < 0)   return -1;
    }
    return 0;
}

void reset_stance(void) {
	for (unsigned i = 0; i < 12; i++) {
		set_pwm(i, 0, 300);
    }
};


int init_servos(void) {
    
    if ((driver_fp = i2cOpen(1, PCA9685_ADDRESS, 0)) < 0) {
        // failed to get i2c bus
        fprintf(stderr, "Failed to get i2c device: PCA9685\n");
        return -1;
    }

    if (init_PCA9685() != 0) {
        fprintf(stderr, "Failed to initialise PCA9685\n");
        close_servos();
        return -1;
    }

    reset_stance();

    return 0;
}

void close_servos(void) {
    if (driver_fp > 0){
        i2cWriteByteData(driver_fp, MODE1, RESET);
        if (i2cClose(driver_fp) != 0)
            fprintf(stderr, "Failed to close PCA9685\n");
    }
}
