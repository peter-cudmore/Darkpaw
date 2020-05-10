#include "led.h"

ws2811_led_t dotcolors[] =
{
	0x00200000,  // red
	0x00201000,  // orange
	0x00202000,  // yellow
	0x00002000,  // green
	0x00002020,  // lightblue
	0x00000020,  // blue
	0x00100010,  // purple
	0x00200010,  // pink
};


ws2811_t ledstring = {
	.freq = LEG_FREQ,
	.dmanum = LED_DMA,
	.channel = {
		[0] = {
		.gpionum = LED_PIN,
		.count = LED_COUNT,
		.invert = 1,
		.brightness = 255,
		.strip_type = WS2811_STRIP_RGB,
		},
		[1] = {
		.gpionum = 0,
		.count = 0,
		.invert = 0,
		.brightness = 0,
		},
	},
};
ws2811_led_t dotcolors[] =
{
	0x00200000,  // red
	0x00002000,  // green
	0x00000020,  // blue
};
void set_colors(const int step) {

	for (int i = 0; i < 6; i++) {

		if (i == step % 6)
			ledstring.channel[0].leds[i] = 0x00200000;
		else
			ledstring.channel[0].leds[i] = 0x00000000;
	}
}

int render(void) {
	ws2811_return_t ret;

	if ((ret = ws2811_render(&ledstring)) != WS2811_SUCCESS) {
		fprintf(stderr, "ws2811_render_failed: %s\n", ws2811_get_return_t_str(ret));
		return (int) ret;
	}
	return 0;
}

int init_leds(void) {

	ws2811_return_t ret;
	if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS) {
		fprintf(stderr, "ws2811_init failed %s\n", ws2811_get_return_t_str(ret));
		return (int) ret;
	}
	return 0;
};

void close_leds(void) {
	ws2811_fini(&ledstring);
};