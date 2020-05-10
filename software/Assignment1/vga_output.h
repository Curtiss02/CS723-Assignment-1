#include <stdio.h>
#include <unistd.h>

#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"

void initVGA();
void drawBackground();
void drawTest();
void drawSystemUptime(unsigned int seconds);
void drawGraphs(float* freq_list, float* roc_list, int start_pos);
void drawThresholds(float instant, float roc);
void drawTimings(int avg, int min, int max, int* history, int pos);
