#include "stm32f10x.h"
#include "Timer.h"

/*==============================================================================
 * Timer.c —— TIM2 产生 1ms 时基（S1）
 *----------------------------------------------------------------------------
 * 时钟推算：系统 72MHz，APB1 分频 2（PCLK1=36MHz），定时器时钟自动 ×2 = 72MHz
 *   72MHz ÷ (PSC+1=72) ÷ (ARR+1=1000) = 1000Hz = 1ms
 *
 * 中断里只做两件事：tick +1、置 1ms 标志；所有逻辑都在主循环里。
 * 中断优先级设为抢占 2（比串口低），保证串口收字节优先。
 *
 * C8T6 板定时器资源分配（H 的引脚图）：
 *   TIM1 = 蜂鸣器 PWM（PA8）       TIM2 = 本文件 1ms 时基
 *   TIM4 = 电机 PWM（PB6/PB7）     TIM3 = 编码器 A（PB4/PB5，部分重映射 + 关 JTAG）
 *   ⚠️ 图上编码器 B 用的是 TIM2（PA15/PB3，部分重映射1 + 关 JTAG）：
 *      如果 S3 真要用编码器 B，就必须把 1ms 时基从 TIM2 挪到 SysTick，
 *      而 S2 的 Delay_us 也占用 SysTick → 需要三个人一起改，别单独动。
 *      当前方案是开环（不依赖编码器），所以 TIM2 归 S1 时基，不冲突。
 *============================================================================*/

static volatile uint32_t s_tick_ms  = 0;	/* 开机毫秒计数 */
static volatile uint8_t  s_flag_1ms = 0;	/* 1ms 到点标志（主循环取走后清零） */

/**
  * 函    数：时基初始化（TIM2，1ms 中断）
  */
void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef t;
	NVIC_InitTypeDef n;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* 72MHz / 72 = 1MHz 计数；计 1000 个数 = 1ms */
	TIM_InternalClockConfig(TIM2);
	t.TIM_ClockDivision     = TIM_CKD_DIV1;
	t.TIM_CounterMode       = TIM_CounterMode_Up;
	t.TIM_Period            = 1000 - 1;
	t.TIM_Prescaler         = 72 - 1;
	t.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &t);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* 抢占 2：比串口（抢占 1）低，收字节不被打断 */
	n.NVIC_IRQChannel = TIM2_IRQn;
	n.NVIC_IRQChannelCmd = ENABLE;
	n.NVIC_IRQChannelPreemptionPriority = 2;
	n.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&n);

	s_tick_ms = 0;
	s_flag_1ms = 0;

	TIM_Cmd(TIM2, ENABLE);
}

/**
  * 函    数：TIM2 中断（每 1ms 一次）
  * 说    明：只累加计数 + 置标志，不做任何耗时操作
  */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		s_tick_ms++;
		s_flag_1ms = 1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

/**
  * 函    数：取开机毫秒数
  */
uint32_t Timer_GetMs(void)
{
	return s_tick_ms;
}

/**
  * 函    数：取开机秒数
  */
uint32_t Timer_GetSec(void)
{
	return s_tick_ms / 1000U;
}

/**
  * 函    数：取 1ms 到点标志
  * 返 回 值：这一毫秒还没被处理过 → 返回 1 并清标志；否则返回 0
  * 说    明：主循环写成 if (Timer_TakeFlag1ms()) { ... }，
  *           即使某一毫秒被耽误了，也只补一次，不会"补跑好几毫秒"
  */
uint8_t Timer_TakeFlag1ms(void)
{
	if (s_flag_1ms != 0)
	{
		s_flag_1ms = 0;
		return 1;
	}
	return 0;
}
