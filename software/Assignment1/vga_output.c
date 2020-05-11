#include "vga_output.h"
#include "global.h"

alt_up_pixel_buffer_dma_dev *pixel_buf;
alt_up_char_buffer_dev *char_buf;
char uptimeString[15];
char thresholdString[35];
char timingBuffer[40];
//Contants for frequency graph
#define FREQ_GRAPH_HEIGHT 140
#define FREQ_GRAPH_WIDTH 500
#define FREQ_GRAPH_X 75
#define FREQ_GRAPH_Y 50
#define FREQ_GRAPH_Y_RES 7
#define MAX_GRAPH_FREQ 60
#define MIN_GRAPH_FREQ 40


//Constants for ROC graph
#define ROC_GRAPH_HEIGHT 140
#define ROC_GRAPH_WIDTH 500
#define ROC_GRAPH_X 75
#define ROC_GRAPH_Y 210
#define ROC_GRAPH_Y_RES 5
#define ROC_GRAPH_MAX 14
#define ROC_GRAPH_MIN -14





int calculateFreqYpos(float freqval);
int calculateROCYpos(float roc_val);

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
		drawBackground();
}

void drawBackground(){



	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);
	//Outer Rectangle
	alt_up_pixel_buffer_dma_draw_rectangle(pixel_buf, 0, 0, 630, 470, 0xFF, 0); //(pixel_buf, )

	alt_up_char_buffer_string(char_buf, "COMPSYS 723 GROUP 24", 1, 1);

	//Freq Graph
	alt_up_pixel_buffer_dma_draw_rectangle(pixel_buf, FREQ_GRAPH_X, FREQ_GRAPH_Y, FREQ_GRAPH_X+FREQ_GRAPH_WIDTH, FREQ_GRAPH_Y+FREQ_GRAPH_HEIGHT, 0xFF, 0);
	alt_up_char_buffer_string(char_buf, "FREQ (HZ)", FREQ_GRAPH_X/8+1, (FREQ_GRAPH_Y/8)-1 );

	//ROC Graph
	alt_up_pixel_buffer_dma_draw_rectangle(pixel_buf, ROC_GRAPH_X, ROC_GRAPH_Y, ROC_GRAPH_X+ROC_GRAPH_WIDTH, ROC_GRAPH_Y+ROC_GRAPH_HEIGHT, 0xFF, 0);
	alt_up_char_buffer_string(char_buf, "Rate of Change (HZ/s)", ROC_GRAPH_X/8+1, (ROC_GRAPH_Y/8)-1 );

	//Hz Markers on Freq graph
	char buffer[5];
	int i;
	for(i = 60; i >= 40; i-=5){
		int yPos = calculateFreqYpos(i);
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, FREQ_GRAPH_X - 10, FREQ_GRAPH_X, yPos, 0xFF, 0);
		sprintf(buffer, "%d", i);
		alt_up_char_buffer_string(char_buf, buffer, FREQ_GRAPH_X/8 - 4, yPos/8 );
	}

	//Hz Range 0-70, hence 140 height
	for(i = 10; i >= -10;  i-= 5){
		int yPos = calculateROCYpos(i);
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, ROC_GRAPH_X - 10, ROC_GRAPH_X, yPos, 0xFF, 0);
		sprintf(buffer, "%d", i);
		alt_up_char_buffer_string(char_buf, buffer, ROC_GRAPH_X/8 - 4, yPos/8 );
	}

}
int calculateFreqYpos(float freqval){
	//Take Freq value, x2, subract from y + height

	//Boundary Checks
	if(freqval > MAX_GRAPH_FREQ){
		freqval = MAX_GRAPH_FREQ;
	}
	else if(freqval < MIN_GRAPH_FREQ){
		freqval = MIN_GRAPH_FREQ;
	}
	return (int)(FREQ_GRAPH_Y+FREQ_GRAPH_HEIGHT)-((freqval-MIN_GRAPH_FREQ)*FREQ_GRAPH_Y_RES);
}

int calculateROCYpos(float roc_val){
	if(roc_val > ROC_GRAPH_MAX){
		roc_val = ROC_GRAPH_MAX;
	}
	else if(roc_val < ROC_GRAPH_MIN){
		roc_val = ROC_GRAPH_MIN;
	}
	return (int)(ROC_GRAPH_Y+ROC_GRAPH_HEIGHT)-((roc_val-ROC_GRAPH_MIN)*ROC_GRAPH_Y_RES);
}


#define INTERPOLATE 1


float freq_list[FREQ_ARR_SIZE];
float roc_list[FREQ_ARR_SIZE];
void drawGraphs(float* freq_list_in, float* roc_list_in, int start_pos){

	//Copy in arrays so that datat does nto change while running
	int i;
	for(i = 0; i < FREQ_ARR_SIZE; i++){
		freq_list[i] = freq_list_in[i];
		roc_list[i] = roc_list_in[i];
	}

	//Clear graph area
	alt_up_pixel_buffer_dma_draw_box(pixel_buf, FREQ_GRAPH_X+1, FREQ_GRAPH_Y+1, FREQ_GRAPH_X+FREQ_GRAPH_WIDTH-1, FREQ_GRAPH_Y+FREQ_GRAPH_HEIGHT-1 ,0,0);
	alt_up_pixel_buffer_dma_draw_box(pixel_buf, ROC_GRAPH_X+1, ROC_GRAPH_Y+1, ROC_GRAPH_X+ROC_GRAPH_WIDTH-1, ROC_GRAPH_Y+ROC_GRAPH_HEIGHT-1 ,0,0);

	int xPos = FREQ_GRAPH_X;
	int xStep = FREQ_GRAPH_WIDTH/FREQ_ARR_SIZE;


	for(i = 0; i < FREQ_ARR_SIZE-1; i++){
		if(INTERPOLATE == 1)
		{
			int yPos1 = calculateFreqYpos(freq_list[(i+start_pos)%FREQ_ARR_SIZE]);
			int yPos2 = calculateFreqYpos(freq_list[(i+start_pos+1)%FREQ_ARR_SIZE]);
			alt_up_pixel_buffer_dma_draw_line(pixel_buf, xPos,  yPos1,xPos+xStep, yPos2, 0xFFFF, 0);




			//Draw the ROC Graph
			yPos1 = calculateROCYpos(roc_list[(i+start_pos)%FREQ_ARR_SIZE]);
			yPos2 =	calculateROCYpos(roc_list[(i+start_pos+1)%FREQ_ARR_SIZE]);
			alt_up_pixel_buffer_dma_draw_line(pixel_buf, xPos, yPos1,xPos+xStep, yPos2, 0xFFFF, 0);


		}
		else{

			//Draw the frequency graph
			int yPos = calculateFreqYpos(freq_list[(i+start_pos)%FREQ_ARR_SIZE]);
			alt_up_pixel_buffer_dma_draw_hline(pixel_buf, xPos, xPos+xStep, yPos, 0xFFFF, 0);

			//Draws vertical line between last line and current
			if(i > 0){
				alt_up_pixel_buffer_dma_draw_vline(pixel_buf, xPos, yPos, calculateFreqYpos(freq_list[(i-1+start_pos)%FREQ_ARR_SIZE]), 0xFFFF, 0);
			}


			//Draw the ROC Graph
			yPos = calculateROCYpos(roc_list[(i+start_pos)%FREQ_ARR_SIZE]);

			alt_up_pixel_buffer_dma_draw_hline(pixel_buf, xPos, xPos+xStep, yPos, 0xFFFF, 0);

			//Draws vertical line between last line and current
			if(i > 0){
				alt_up_pixel_buffer_dma_draw_vline(pixel_buf, xPos, yPos, calculateROCYpos(roc_list[(i-1+start_pos)%FREQ_ARR_SIZE]), 0xFFFF, 0);
			}

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
void drawThresholds(float instant, float roc)
{
	sprintf(thresholdString, "INSTANT: %4.1f  ROC: %4.1f", instant, roc);
	alt_up_char_buffer_string(char_buf, thresholdString, 50, 2);
	alt_up_char_buffer_string(char_buf, "LOAD SHEDDING THRESHOLD", 50, 1);
}
void drawTimings(int avg, int min, int max, int* history, int pos){
	if(min == INT_MAX){
		min = 0;
	}
	float min_ms = (float)min  /1000;
	float max_ms = (float) max / 1000;
	float avg_ms = (float)avg / 1000;
	int i;
	//Display past 5 readings
	float temp;

	alt_up_char_buffer_string(char_buf, "TIMINGS (MS)", 4, 57-12);
	for(i = 0; i < 5; i++){
		int arr_pos = (i+pos)%5;
		temp = (float)history[arr_pos] / 1000;
		sprintf(timingBuffer, "%d: %4.2f", 5-i, temp);
		alt_up_char_buffer_string(char_buf, timingBuffer, 1, 57-i*2);
	}

	alt_up_char_buffer_string(char_buf, "HISTORY", 1, 57-10);
	sprintf(timingBuffer, "AVG: %4.2f",avg_ms);
	alt_up_char_buffer_string(char_buf, timingBuffer, 11, 57-10);
	sprintf(timingBuffer, "MIN: %4.2f",min_ms);
	alt_up_char_buffer_string(char_buf, timingBuffer, 11, 57-8);
	sprintf(timingBuffer, "MAX: %4.2f",max_ms);
	alt_up_char_buffer_string(char_buf, timingBuffer, 11, 57-6);

}

