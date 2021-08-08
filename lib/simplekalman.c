
#include <math.h>
#include "simplekalman.h"

float update_measure_kalman(_kalman *km, float mea)
{

	static float kalman_gain;
	static float current_est;
	static float last_est;
	kalman_gain = (km->estimate_e)/((km->estimate_e) + (km->measure_e));
	current_est = last_est + kalman_gain*(mea - last_est);
	km->estimate_e = (1.0 - kalman_gain) * (km->estimate_e) + fabs(last_est - current_est)*(km->q);
	last_est = current_est;
	return current_est;
	
}