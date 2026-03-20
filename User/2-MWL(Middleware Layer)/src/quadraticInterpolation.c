#include "quadraticInterpolation.h"


/**
 * @brief 二阶样条插值
* @param *quadraticInterpolation 数据指针
*  @备注：根据delta_theta=at^2+bt+c，已知theta0以及theta1,以及(delta_theta/2)'=无穷，(theta0)'=(theta1)'=0
*   可得出 a=(theta1 - theta0) / (T * T) T为总时间,b=0,c=theta0
*   需要在外部一直更新theta0
 */
void quadraticInterpolation(quadraticInterpolation_t *quadraticInterpolation_param)
{
    static double a;
	if(quadraticInterpolation_param->sustained_time <= quadraticInterpolation_param->total_time)
		{
			quadraticInterpolation_param->sustained_time +=1;
			a = (quadraticInterpolation_param->theta_set-quadraticInterpolation_param->theta_now)/(quadraticInterpolation_param->total_time)/(quadraticInterpolation_param->total_time);
			quadraticInterpolation_param->theta_out = a*quadraticInterpolation_param->sustained_time*quadraticInterpolation_param->sustained_time+quadraticInterpolation_param->theta_now;
		}
		else 
		{
			quadraticInterpolation_param->finish = 1;
		}
}

void quadraticInterpolation_clear_param(quadraticInterpolation_t *quadraticInterpolation_clear_param)
{
	quadraticInterpolation_clear_param->theta_now = 0;
  quadraticInterpolation_clear_param->sustained_time = 0;
  quadraticInterpolation_clear_param->theta_out = 0;
}

void quadraticInterpolation_clear_set(quadraticInterpolation_t *quadraticInterpolation_clear_set)
{
	quadraticInterpolation_clear_set->theta_set = 0;
	quadraticInterpolation_clear_set->total_time = 0;
}
