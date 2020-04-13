#include "vga_output.h"
#include "global.h"

alt_up_pixel_buffer_dma_dev *pixel_buf;
alt_up_char_buffer_dev *char_buf;
char uptimeString[15];


#define FREQ_GRAPH_HEIGHT 140
#define FREQ_GRAPH_WIDTH 500
#define FREQ_GRAPH_X 75
#define FREQ_GRAPH_Y 50
#define MAX_GRAPH_FREQ 69
#define MIN_GRAPH_FREQ 1
unsigned char TEST = 'a';


unsigned int calculateFreqYpos(unsigned int freqval);


void initVGA(){
		pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);

		if(pixel_buf == NULL){
			printf("Cannot find pixel buffer device\n");
		}


		char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
			if(char_buf == NULL){
				printf("can't find char buffer device\n");
			}

		alt_up_char_buffer_clear(char_buf);
}

void drawBackground(){


	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);
	//Outer Rectangle
	alt_up_pixel_buffer_dma_draw_rectangle(pixel_buf, 0, 0, 630, 470, 0xFF, 0); //(pixel_buf, )

	//Freq Graph
	alt_up_pixel_buffer_dma_draw_rectangle(pixel_buf, FREQ_GRAPH_X, FREQ_GRAPH_Y, FREQ_GRAPH_X+FREQ_GRAPH_WIDTH, FREQ_GRAPH_Y+FREQ_GRAPH_HEIGHT, 0xFF, 0);
	alt_up_char_buffer_string(char_buf, "FREQ (HZ)", FREQ_GRAPH_X/8+1, (FREQ_GRAPH_Y/8)-1 );

	//Hz Markers on Freq graph
	char buffer[5];
	int i;
	for(i = 0; i < 7; i++){
		int yPos = calculateFreqYpos(i*10);
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, FREQ_GRAPH_X - 10, FREQ_GRAPH_X, yPos, 0xFF, 0);
		sprintf(buffer, "%d", i*10);
		alt_up_char_buffer_string(char_buf, buffer, FREQ_GRAPH_X/8 - 4, yPos/8 );
	}

	//Hz Range 0-70, hence 140 height


}
unsigned int calculateFreqYpos(unsigned int freqval){
	//Take Freq value, x2, subract from y + height


	return (FREQ_GRAPH_Y+FREQ_GRAPH_HEIGHT)-(freqval*(FREQ_GRAPH_HEIGHT /(MAX_GRAPH_FREQ - MIN_GRAPH_FREQ)));
}

void drawFrequencyGraph(unsigned int* freq_list, int length){

	//Clear graph area
	alt_up_pixel_buffer_dma_draw_box(pixel_buf, FREQ_GRAPH_X+1, FREQ_GRAPH_Y+1, FREQ_GRAPH_X+FREQ_GRAPH_WIDTH-1, FREQ_GRAPH_Y+FREQ_GRAPH_HEIGHT-1 ,0,0);

	unsigned int xPos = FREQ_GRAPH_X;
	unsigned int xStep = FREQ_GRAPH_WIDTH/length;
	int i;
	for(i = 0; i < length; i++){
		unsigned int current_freq = freq_list[i];

		//Boundary Checks
		if(current_freq > MAX_GRAPH_FREQ){
			current_freq = MAX_GRAPH_FREQ;
		}
		if(current_freq < MIN_GRAPH_FREQ){
			current_freq = MIN_GRAPH_FREQ;
		}

		unsigned int yPos = calculateFreqYpos(current_freq);

		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, xPos, xPos+xStep, yPos, 0xFFFF, 0);

		//Draws vertical line between last line and current
		if(i > 0){
			alt_up_pixel_buffer_dma_draw_vline(pixel_buf, xPos, yPos, calculateFreqYpos(freq_list[i-1]), 0xFFFF, 0);
		}
		xPos += xStep;
	}
}


void drawSystemUptime(unsigned int seconds)
{
	unsigned int days = seconds / (60 * 60 * 24);
	seconds -= days * (60 * 60 * 24);
	unsigned int hours = seconds / (60 * 60);
	seconds -= hours * (60 * 60);
	unsigned int minutes = seconds / 60;
	seconds -= minutes*60;

	sprintf(uptimeString, "%.2dD %.2dH %.2dM %.2dS", days, hours, minutes, seconds);
	alt_up_char_buffer_string(char_buf, uptimeString, 62, 57);
}
void drawTest(){
	alt_up_char_buffer_draw(char_buf, TEST, 10, 10);
	alt_up_char_buffer_draw(char_buf, TEST+1, 11, 11);
	TEST++;
}
