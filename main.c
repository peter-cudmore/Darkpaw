#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "led.h"


ws2811_t ledstring = {
	.freq = 800000,
	.dmanum= 10,
	.channel = {
		[0] = {
		.gpionum = 12,
		.count = 6,
		.invert = 0,
		.brightness = 255,
		.strip_type = WS2811_STRIP_GBR,
		},
		[1] = {
		.gpionum = 0,
		.count = 0,
		.invert = 0,
		.brightness = 0,
		},
	},
};

void set_colors(const int step){

  for (int i =0; i< 6; i++){

    if (i == step % 6) 
      ledstring.channel[0].leds[i] = 0x00200000;
    else
     ledstring.channel[0].leds [i] = 0x00000000;
  }
}

int main(int arvc, char* argv[]){

  ws2811_return_t ret;
  unsigned int i = 0;

  if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS){
  	fprintf(stderr, "ws2811_init failed %s\n", ws2811_get_return_t_str(ret));
	return ret;
  }

  while (true) {
	set_colors(i);
      if ((ret = ws2811_render(&ledstring)) != WS2811_SUCCESS){
         fprintf(stderr, "ws2811_render_failed: %s\n", ws2811_get_return_t_str(ret));
	 return ret;
      }	
      usleep(10000000 / 15);
      i++;
  }
  
  ws2811_fini(&ledstring);
  return 0;
} 
