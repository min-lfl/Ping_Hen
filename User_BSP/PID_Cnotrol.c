#include "PID_Cnotrol.H"


//***用户函数区***
/**
 * PID 计算核心函数
 * @param pid      指向 PID 结构体的指针
 * @param target   目标值
 * @param measure  实际测量值
 * @param dt       计算时间间隔 (单位通常为秒，如 0.01s)
 */
float PID_Compute(PID_t *pid, float target, float measure, float dt){
	//计算当前误差
	pid->error=target-measure;	

	//误差消除算法
//	if(pid->error<0.05f && pid->error>-0.05f){
//		pid->error=0.0f;
//	}

	//误差过零时清零积分，防止积分滞后导致过冲
	if(pid->error * pid->last_error < 0.0f) {
		pid->integral = 0.0f;
	}
	
	//计算积分(误差累加)
	pid->integral+=pid->error*dt;
	
	
	//积分限幅
	if(pid->integral>pid->integral_max)pid->integral=pid->integral_max;
	if(pid->integral<-(pid->integral_max))pid->integral=-(pid->integral_max);
	
	//计算微分
	pid->differential=(pid->error-pid->last_error)/dt;
	pid->last_error=pid->error;//更新上次误差
	
	//融合计算输出
	pid->output=
		pid->error*pid->Kp    +
		pid->integral*pid->Ki   +
		pid->differential*pid->Kd;
	
	//输出限幅校验
	if(pid->output>pid->output_max)pid->output=pid->output_max;
	if(pid->output<-(pid->output_max))pid->output=-(pid->output_max);
	
	return pid->output;
}



/*
 * 清除 PID 历史状态。
 *
 * 目标：当控制关闭、激光无效、或进入死区时，彻底清除 PID 的内部状态。
 *
 * 为什么要清除？
 *   PID 的积分项会累积历史误差。如果不清除，下次启动控制时，
 *   旧的积分值会让电机瞬间大幅倾斜，可能导致小球飞出。
 *   last_error 也会影响微分项，不清除也会导致瞬态输出。
 *
 * 所有字段都清零：error, last_error, integral, differential, output。
 */
void Ball_Control_Reset_PID(PID_t *pid)
{
	pid->error = 0.0f;
	pid->last_error = 0.0f;
	pid->integral = 0.0f;
	pid->differential = 0.0f;
	pid->output = 0.0f;
}




