#ifndef LED_H
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include "rpi_ws281x/clk.h"
#include "rpi_ws281x/pwm.h"
#include "rpi_ws281x/ws2811.h"

#define LED_COUNT 6 
#define LED_PIN 12
#define LEG_FREQ 800000
#define LED_DMA 10
#define LED_BRIGHTNESS 255
#define LED_CHANNEL 0
#define LED_INVERT false

int init_leds(void);
void close_leds(void);


#define LED_H
#endif


