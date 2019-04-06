#define _USE_MATH_DEFINES
#include <math.h>

#include "sinegen.h"
#include "malloc.h"

int SineGen_init(SineGen **sg_out){
	if (sg_out) {
		SineGen *sg = (SineGen*)malloc(sizeof(SineGen));
		if (sg) {
			sg->sine_curve = (char*)malloc(REFSINE_RESOLUTION);
			if (sg->sine_curve) {
				for (int a = 0; a < REFSINE_RESOLUTION; a++) {
					double current_radian = (double)a / (double)REFSINE_RESOLUTION;
					sg->sine_curve[a] = (char)(127.0 * sin(2 * current_radian*M_PI));
				}
				sg->sine_step = 0.0;
				sg->pos_startidx = 0;
				sg->pos_numsteps = 0;
				*sg_out = sg;
				return 1;
			}
			free(sg);
		}
	}
	return 0;
}

void SineGen_destroy(SineGen *sg) {
	if (sg) {
		if (sg->sine_curve) free(sg->sine_curve);
		free(sg);
	}
}

void SineGen_configure(SineGen *sg, unsigned long samp_rate, unsigned long freq){
	if (sg) {
		unsigned long long crntidx = ((unsigned long long) sg->pos_startidx + (unsigned long long)(sg->pos_numsteps * sg->sine_step));
		sg->sine_step = ((double)freq * (double)REFSINE_RESOLUTION) / (double)samp_rate;
		sg->pos_startidx = crntidx % REFSINE_RESOLUTION;
		sg->pos_numsteps = 0;
	}
}

char SineGen_getSample(SineGen *sg){
	char smp = 0;
	if (sg && sg->sine_curve) {
		unsigned long long idx = ((unsigned long long) sg->pos_startidx + (unsigned long long)(sg->pos_numsteps++ * sg->sine_step));
		smp = sg->sine_curve[idx % REFSINE_RESOLUTION];
	}
	return smp;
}
