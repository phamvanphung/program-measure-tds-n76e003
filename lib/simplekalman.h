

typedef struct kalman
{
	float measure_e;
	float estimate_e;
	float q;
}_kalman;

float update_measure_kalman(_kalman *km, float mea);