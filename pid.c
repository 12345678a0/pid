#include "pid.h"

#ifndef PID_SAMPLE_PERIOD 
    #define PID_SAMPLE_PERIOD	1000  //采样周期 1000ms
#endif

#ifndef PID_PWM_OUT_GRADIENT
    #define PID_PWM_OUT_GRADIENT    10  // 梯度
#endif

#ifndef PID_PWM_OUT_PERIOD
	#define PID_PWM_OUT_PERIOD 1000 //PWM输出周期 1000ms 
#endif


#define PID_OUT_VAL_MAX  	PID_PWM_OUT_PERIOD / PID_PWM_OUT_GRADIENT


typedef struct {
	float setVal;          //设定值
	float actualVal;       //实际值
	float err;             //偏差值
	float err_next;        //上一个的偏差值
	float err_last;        //上上一个的偏差值
	float Kp, Ki, Kd;      //比例、积分、微分系数
	float out;             //输出值
} pid_param_st;

/* pid 参数 */
static pid_param_st g_pid_param;
/* pid 时钟 */
static uint16_t g_pid_tick = 0;


 /**
  * @brief   pid param init
  *
  * @param   null
  *
  * @return  null
  */
static void pid_param_init()
{
	g_pid_param.setVal = 0.0;
	g_pid_param.actualVal = 0.0;
	g_pid_param.err = 0.0;
	g_pid_param.err_last = 0.0;
	g_pid_param.err_next = 0.0;
	g_pid_param.Kp = 1.0;
	g_pid_param.Ki = 0.0;
	g_pid_param.Kd = 0.0;
	g_pid_param.out = 0.0;
}



/**
  * @brief   获取开关状态
  *
  * @param   null
  *
  * @return  bool sw_state: 0(off)   1(on)
  */
static uint8_t pid_sw_state_get()
{
	uint8_t sw_state = 0;
	
	return sw_state;
}


/**
  * @brief   获取实际值
  *
  * @param   null
  *
  * @return  int set_temperature
  */
static float pid_actualVal_get()
{
	float actualVal = 0;

	return actualVal;
}


/**
  * @brief   获取设定值
  *
  * @param   nul
  *
  * @return  null
  */
static float pid_setVal_get()
{
	float setVal = 0;

	return setVal;
}


/**
  * @brief   pid 控制输出
  *
  * @param   int out_val: 输出值
  *
  * @return  null
  */
static void pid_out_ctrl(int outVal)
{

}


/**
 * @brief	pid 输出值计算(增量式PID)
 *
 * @param	float setVal: 设定值(目标值)
 *
 * @return	float g_pid_param.out: 输出值
 */
static float pid_outVal_calculate(float setVal)
{
	float increment_val;
	
	g_pid_param.setVal = setVal;
	g_pid_param.err = g_pid_param.setVal - g_pid_param.actualVal;

	increment_val = g_pid_param.Kp * (g_pid_param.err - g_pid_param.err_next)    \
		             + g_pid_param.Ki * g_pid_param.err                          \
		             + g_pid_param.Kd * (g_pid_param.err - 2 * g_pid_param.err_next + g_pid_param.err_last);  

	g_pid_param.out  += increment_val;
	g_pid_param.err_last = g_pid_param.err_next;
	g_pid_param.err_next = g_pid_param.err;

	 //增量输出
	return g_pid_param.out;  
}


/**
  * @brief   pid处理子函数
  *
  * @param   nul
  *
  * @return  null
  */
static void pid_handle_sub()
{
	int outVal = 0;
	static uint8_t s_state = 0;

	if (pid_sw_state_get())
	{
	    g_pid_param.actualVal = pid_actualVal_get();

	    outVal = pid_outVal_calculate(pid_setVal_get());

		if (outVal < 0)
		{
			outVal = 0;
		}
		else if (outVal > PID_OUT_VAL_MAX)
		{
			outVal = PID_OUT_VAL_MAX;
		}
	}
	else
	{
	    pid_param_init();
	    outVal = 0;
	}
	
	pid_out_ctrl(outVal);
}


/**
  * @brief   pid处理
  *
  * @param   nul
  *
  * @return  null
  */
void pid_handle() 
{
	if (g_pid_tick >= PID_SAMPLE_PERIOD)
	{
		g_pid_tick = 0;
		
		pid_handle_sub();
	}
}

void pid_run_tick(uint8_t base)
{
	g_pid_tick += base;
}

 /**
  * @brief   pid init
  *
  * @param   null
  *
  * @return  null
  */
void pid_init()
{
	pid_param_init();	
}


