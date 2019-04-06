#ifndef INCLUDE_SINEGEN_H
	#define INCLUDE_SINEGEN_H

	#define REFSINE_RESOLUTION 16384

typedef struct _SineGen {
	char *sine_curve;
	double sine_step;
	unsigned long pos_startidx;
	unsigned long pos_numsteps;
} SineGen;

int  SineGen_init(SineGen **sg);
void SineGen_destroy(SineGen *sg);
void SineGen_configure(SineGen *sg, unsigned long samp_rate, unsigned long freq);
char SineGen_getSample(SineGen *sg);

#endif // INCLUDE_SINEGEN_H
