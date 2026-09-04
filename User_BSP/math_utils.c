#include "MATH_UTILS.H"



/*
 * 数值限幅：把 value 限制在 [minimum, maximum] 范围内。
 * 用于位置限幅、速度限幅、PID 输出限幅等。
 */
float Ball_Control_Clamp(float value, float minimum, float maximum)
{
	if (value < minimum)
		return minimum;
	if (value > maximum)
		return maximum;
	return value;
}


/*
 * 渐进逼近（斜率限制）：从 current 向 target 移动，但每步最多移动 maximum_step。
 *
 * 目标：防止电机指令突变。比如 PID 突然输出 +250 脉冲，但上一刻电机还在 -250，
 *       如果直接跳变会导致机械冲击。通过 Approach 限制每步变化量，实现平滑过渡。
 *
 * 总结,这是用响应速度换控制平滑度,并且如果maximum_step太小,会导致响应太慢
 */
float Ball_Control_Approach(float current, float target, float maximum_step)
{
	if (target > current + maximum_step)
		return current + maximum_step;
	if (target < current - maximum_step)
		return current - maximum_step;
	return target;
}


/*
 * 浮点数四舍五入转整数。
 * 用于将 PID 计算出的浮点脉冲值转为电机命令需要的整数脉冲。
 */
int32_t Ball_Control_Round_To_Int32(float value)
{
	return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}
