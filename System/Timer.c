#include "stm32f10x.h"
#include "Timer.h"

/*==============================================================================
 * 文件：Timer.c
 * 作用：用 TIM2 产生 1ms 系统时基（S1 负责）
 *----------------------------------------------------------------------------
 * 整体思路（生产者-消费者模型）：
 *   生产者：TIM2 中断——每 1ms 自动执行一次，只做两件小事：
 *           ① 总毫秒数 +1   ② 把"1ms 到点"标志置 1
 *   消费者：主循环——通过 Timer_TakeFlag1ms() 取走标志，然后执行 1ms 任务
 *
 *   为什么中断里不直接把活干了？
 *     中断要尽量短。若在中断里跑消抖/闪烁/打印，会拖慢其它中断（串口会丢字节），
 *     而且时序和业务逻辑混在一起极难调试。所以"中断只打点，主循环干活"。
 *
 * 时钟推算（重要，答辩常问）：
 *   系统 72MHz → APB1 分频 2（PCLK1 = 36MHz）
 *   但 STM32 的定时器时钟有 ×2 机制：APB1 分频≠1 时，定时器时钟 = PCLK1 × 2 = 72MHz
 *   72MHz ÷ (PSC+1 = 72) = 1MHz        → 计 1 个数 = 1 微秒
 *   1MHz ÷ (ARR+1 = 1000) = 1000Hz     → 每 1000 微秒 = 1ms 溢出一次
 *============================================================================*/

/*----------------------------------------------------------------------------
 * 模块内部变量（static：只在本文件可见，外部只能通过上面的函数访问）
 * 为什么要 volatile：
 *   这两个变量会被"中断"修改、被"主循环"读取。如果不加 volatile，
 *   编译器可能把它们优化进寄存器，导致主循环永远看不到中断的修改（死循环）。
 *----------------------------------------------------------------------------*/
static volatile uint32_t s_tick_ms  = 0;	/* 开机以来的毫秒计数（中断里 ++） */
static volatile uint8_t  s_flag_1ms = 0;	/* 1ms 到点标志：1=新的一毫秒到了 */

/**
  * 函数：Timer_Init
  * 作用：配置 TIM2 为 1ms 周期中断，并启动
  * 调用：整个程序启动时调用一次（main 的初始化部分）
  */
void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef t;		/* 定时器时基配置结构体 */
	NVIC_InitTypeDef n;				/* 中断控制器配置结构体 */

	/* 第 1 步：打开 TIM2 的时钟
	 * 任何外设在使用前都必须先开时钟，否则写它的寄存器全部无效（新手最常踩的坑）。*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* 第 2 步：选择时钟源为内部时钟（72MHz）
	 * 也可以用外部时钟/触发模式，本工程用最简单的内部时钟。*/
	TIM_InternalClockConfig(TIM2);

	/* 第 3 步：填时基参数 */
	t.TIM_ClockDivision     = TIM_CKD_DIV1;			/* 时钟分频：不分频（只影响采样，不影响计数） */
	t.TIM_CounterMode       = TIM_CounterMode_Up;	/* 向上计数：0 → ARR，溢出后回 0 */
	t.TIM_Period            = 1000 - 1;				/* ARR：数到 999 就溢出 → 1000 次 = 1ms */
	t.TIM_Prescaler         = 72 - 1;				/* PSC：72MHz ÷ 72 = 1MHz（1 微秒计 1 次） */
	t.TIM_RepetitionCounter = 0;					/* 高级定时器才用，TIM2 填 0 */
	TIM_TimeBaseInit(TIM2, &t);						/* 把参数写进 TIM2 寄存器 */

	/* 第 4 步：配置"溢出中断"
	 * 先清掉可能残留的更新标志，再允许更新中断。顺序反了会出现"一开中断立刻进一次"的怪现象。*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);			/* 清更新标志 */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		/* 允许更新（溢出）中断 */

	/* 第 5 步：告诉 NVIC（内核中断控制器）允许 TIM2 中断
	 * 注意：外设允许中断 ≠ 内核会响应，NVIC 这里还要再开一次。*/
	n.NVIC_IRQChannel                   = TIM2_IRQn;	/* 中断号 */
	n.NVIC_IRQChannelCmd                = ENABLE;		/* 使能该中断通道 */
	n.NVIC_IRQChannelPreemptionPriority = 2;			/* 抢占优先级 2：比串口(1)低，保证收字节优先 */
	n.NVIC_IRQChannelSubPriority        = 0;			/* 子优先级 */
	NVIC_Init(&n);										/* 写入 NVIC */

	/* 第 6 步：启动定时器（START 位置 1 后开始计数） */
	TIM_Cmd(TIM2, ENABLE);
}

/**
  * 函数：TIM2_IRQHandler
  * 作用：TIM2 溢出中断服务函数，每 1ms 自动被硬件调用一次
  * 说明：函数名必须和启动文件里向量表的名字一模一样，写错就不会进中断。
  */
void TIM2_IRQHandler(void)
{
	/* 判断是不是"更新"中断（确认是溢出触发的，而非其它事件） */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		s_tick_ms++;		/* ① 总毫秒数 +1：给 Timer_GetMs 用 */
		s_flag_1ms = 1;		/* ② 举起旗子：告诉主循环"新的一毫秒到了" */

		/* ③ 必须手动清中断标志，否则退出中断后会立刻再次进入（一直重复进中断）。 */
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

/**
  * 函数：Timer_GetMs
  * 作用：返回"开机以来经过的毫秒数"
  * 用法：uint32_t t0 = Timer_GetMs();  之后做 (Timer_GetMs() - t0) 得到经过的毫秒数
  * 说明：测"经过的时间"要用"当前值 - 起点值"，不要用绝对时刻直接比较。
  */
uint32_t Timer_GetMs(void)
{
	return s_tick_ms;
}

/**
  * 函数：Timer_GetSec
  * 作用：返回"开机以来经过的整秒数"
  * 说明：整数除法直接砍掉小数：999ms → 0 秒，1999ms → 1 秒（不会四舍五入）。
  *       需要精确时间请用 Timer_GetMs()。
  */
uint32_t Timer_GetSec(void)
{
	return s_tick_ms / 1000U;		/* 1000U 中的 U 表示无符号常量，避免有/无符号混算提示 */
}

/**
  * 函数：Timer_TakeFlag1ms
  * 作用：取"1ms 到点"标志（取走式：取一次就清一次）
  * 返回：1 = 有新的一毫秒（本次已取走）；0 = 还没到
  * 说明：这是本模块最关键的接口。
  *       如果只读不清，标志会一直是 1，主循环每秒几十万次地执行 1ms 任务，
  *       消抖计时、闪烁周期、长按判断全部错乱 —— 所以必须"取走即清"。
  */
uint8_t Timer_TakeFlag1ms(void)
{
	if (s_flag_1ms != 0)		/* 看看这一毫秒的旗子有没有被举起 */
	{
		s_flag_1ms = 0;			/* 清旗子（表示这一毫秒已经被消费掉） */
		return 1;				/* 告诉调用者：有新的一毫秒，给你了 */
	}

	return 0;					/* 没到 1ms */
}
