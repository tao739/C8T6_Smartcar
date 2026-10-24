#include "stm32f10x.h"
#include "Buzzer.h"

/*==============================================================================
 * 文件：Buzzer.c
 * 作用：蜂鸣器驱动（S1）—— 【有源蜂鸣器】版本，GPIO 开关 + 非阻塞节奏
 * 硬件：蜂鸣器模块 I/O 接 PA8，低电平触发（给低电平就响）
 *----------------------------------------------------------------------------
 * 和"无源蜂鸣器"的区别：
 *   无源：必须给 PWM 方波，频率决定音调（要用定时器）
 *   有源：给电平就响，音调固定（只用 GPIO，代码更简单）
 *
 * 节奏播放（非阻塞）：
 *   Buzzer_Play() 只是把"响几声、每声多久、间隔多久"记下来，
 *   真正的"响→停→响"由 Buzzer_Tick1ms() 每 1ms 推进，CPU 不被占用。
 *
 * 状态机：
 *   IDLE（空闲）──Buzzer_Play()──> ON（正在响）
 *   ON：响够 on_ms → 关掉；若还有声数 → 进入 OFF（间隔）
 *   OFF：等够 off_ms → 再响 → 回到 ON
 *   声数减到 0 → 回 IDLE
 *============================================================================*/

/*----------------------------------------------------------------------------
 * 引脚配置
 *----------------------------------------------------------------------------*/
#define BUZ_CLK          RCC_APB2Periph_GPIOA	/* PA8 所在端口 GPIOA 的时钟 */
#define BUZ_PORT         GPIOA					/* 端口 */
#define BUZ_PIN          GPIO_Pin_8				/* 引脚号：PA8 */

/* 播放阶段的三个状态 */
#define BUZ_PHASE_IDLE   0		/* 空闲：没在播放 */
#define BUZ_PHASE_ON     1		/* 正在响这一声 */
#define BUZ_PHASE_OFF    2		/* 两声之间的静音间隔 */

/*----------------------------------------------------------------------------
 * 模块内部变量（static：只在本文件可见）
 *----------------------------------------------------------------------------*/
static uint32_t s_ms = 0;					/* 模块内部 1ms 计数（每次 Tick1ms +1） */
static uint8_t  s_phase = BUZ_PHASE_IDLE;	/* 当前播放阶段 */
static uint32_t s_phase_tick = 0;			/* 本阶段开始的时刻（s_ms 值） */
static uint8_t  s_count = 0;				/* 还剩几声没响 */
static uint16_t s_on_ms = 0;				/* 每声响多久（毫秒） */
static uint16_t s_off_ms = 0;				/* 两声之间静音多久（毫秒） */

/*----------------------------------------------------------------------------
 * 底层函数：把"响/停"写到引脚
 *   本模块是低电平触发：
 *       要响 → 输出低电平（Bit_RESET）
 *       要停 → 输出高电平（Bit_SET）
 *   ⚠ 如果实际模块是高电平触发，把下面两行的 Bit_RESET / Bit_SET 对调。
 *----------------------------------------------------------------------------*/
static void Buzzer_WritePin(uint8_t on)
{
	if (on != 0)	/* 要响 */
	{
		GPIO_WriteBit(BUZ_PORT, BUZ_PIN, Bit_RESET);	/* 输出低电平 → 蜂鸣器响 */
	}
	else			/* 要停 */
	{
		GPIO_WriteBit(BUZ_PORT, BUZ_PIN, Bit_SET);		/* 输出高电平 → 蜂鸣器停 */
	}
}

/**
  * 函数：Buzzer_Init
  * 作用：把 PA8 配成推挽输出，并让蜂鸣器默认不响
  * 调用：程序启动时调用一次
  */
void Buzzer_Init(void)
{
	GPIO_InitTypeDef g;		/* GPIO 配置结构体 */

	RCC_APB2PeriphClockCmd(BUZ_CLK, ENABLE);	/* 开 GPIOA 时钟（不开时钟配置无效） */

	g.GPIO_Mode  = GPIO_Mode_Out_PP;			/* 推挽输出：能主动输出高低电平 */
	g.GPIO_Speed = GPIO_Speed_50MHz;			/* 输出速度（蜂鸣器用不着这么快，填默认值） */
	g.GPIO_Pin   = BUZ_PIN;						/* 引脚：PA8 */
	GPIO_Init(BUZ_PORT, &g);					/* 写入配置 */

	s_ms = 0;						/* 内部毫秒计数清零 */
	s_phase = BUZ_PHASE_IDLE;		/* 播放状态：空闲 */
	s_phase_tick = 0;				/* 阶段计时起点 */
	s_count = 0;					/* 没有待播放的声数 */
	s_on_ms = 0;					/* 每声时长 */
	s_off_ms = 0;					/* 间隔时长 */

	Buzzer_Off();					/* 上电先不响 */
}

/**
  * 函数：Buzzer_On
  * 作用：立即开始响（输出触发电平）
  */
void Buzzer_On(void)
{
	Buzzer_WritePin(1);		/* 输出低电平，蜂鸣器响 */
}

/**
  * 函数：Buzzer_Off
  * 作用：立即停止响（输出非触发电平）
  */
void Buzzer_Off(void)
{
	Buzzer_WritePin(0);		/* 输出高电平，蜂鸣器停 */
}

/**
  * 函数：Buzzer_Stop
  * 作用：立即静音并终止正在播放的节奏
  * 用途：状态切换（急停/复位）时立刻打断提示音
  */
void Buzzer_Stop(void)
{
	Buzzer_Off();				/* 先关声音 */
	s_phase = BUZ_PHASE_IDLE;	/* 播放状态回空闲 */
	s_count = 0;				/* 剩余声数清零 */
}

/**
  * 函数：Buzzer_Play
  * 作用：开始播放一段节奏（非阻塞，立即返回）
  * 参数：count  响几声
  *       on_ms  每声响多久（毫秒）
  *       off_ms 两声之间静音多久（毫秒）
  * 说明：有源蜂鸣器没有音调参数，只控制"响多久、响几声、间隔多久"。
  */
void Buzzer_Play(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
	Buzzer_Stop();					/* 先停掉上一段节奏，避免状态混乱 */

	if (count == 0) { return; }		/* 响 0 声 = 什么都不做 */

	s_count   = count;				/* 记录总声数 */
	s_on_ms   = on_ms;				/* 每声时长 */
	s_off_ms  = off_ms;				/* 间隔时长 */
	Buzzer_On();					/* 立即开始响第一声 */
	s_phase = BUZ_PHASE_ON;			/* 状态切到"正在响" */
	s_phase_tick = s_ms;			/* 记录这一声的开始时刻 */
}

/**
  * 函数：Buzzer_Tick1ms
  * 作用：节奏推进，必须每 1ms 调用一次
  * 说明：只做计数比较，几微秒就返回
  */
void Buzzer_Tick1ms(void)
{
	s_ms++;							/* 内部毫秒计数 +1 */

	/* 空闲状态：什么都不用做 */
	if (s_phase == BUZ_PHASE_IDLE) { return; }

	if (s_phase == BUZ_PHASE_ON)	/* 当前"正在响" */
	{
		/* 这一声响够 on_ms 了吗？ */
		if ((s_ms - s_phase_tick) >= s_on_ms)
		{
			Buzzer_Off();			/* 关掉声音 */
			s_count--;				/* 剩余声数 -1 */

			if (s_count == 0)		/* 全部响完了 */
			{
				s_phase = BUZ_PHASE_IDLE;	/* 回空闲 */
				return;
			}

			s_phase = BUZ_PHASE_OFF;	/* 还有下一声：进入静音间隔 */
			s_phase_tick = s_ms;		/* 记录间隔开始时刻 */
		}
	}
	else /* BUZ_PHASE_OFF：两声之间的静音 */
	{
		/* 静音够 off_ms 了吗？ */
		if ((s_ms - s_phase_tick) >= s_off_ms)
		{
			Buzzer_On();				/* 开始下一声 */
			s_phase = BUZ_PHASE_ON;		/* 回到"正在响" */
			s_phase_tick = s_ms;		/* 记录这一声开始时刻 */
		}
	}
}

/**
  * 函数：Buzzer_IsBusy
  * 作用：查询是否还在播放节奏
  * 说明：本项目暂未用到，留给以后"等提示音播完再切状态"之类的需求
  */
uint8_t Buzzer_IsBusy(void)
{
	if (s_phase != BUZ_PHASE_IDLE)
	{
		return 1;		/* 还在播放 */
	}
	return 0;			/* 空闲 */
}
