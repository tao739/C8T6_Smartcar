#include "stm32f10x.h"
#include "LED.h"

/*==============================================================================
 * 文件：LED.c
 * 作用：单颗指示灯驱动（S1）—— 非阻塞闪烁 + 一次性脉冲
 * 硬件：LED = PB12（板上标"LED 板载"）
 *
 * 硬件接法（固定）：
 *     3.3V ──[限流电阻]── LED 阳极 → 阴极 ── PB12
 *   PB12 输出低电平 → 灯亮；输出高电平 → 灯灭。
 *
 * 实现思路：
 *   不用 Delay 死等。把"亮多久、灭多久"记成数字，每 1ms 由 LED_Tick1ms()
 *   检查一次"时间到了没"，到了就翻转一次引脚电平。
 *============================================================================*/

/*----------------------------------------------------------------------------
 * 引脚配置
 *----------------------------------------------------------------------------*/
#define LED_CLK          RCC_APB2Periph_GPIOB	/* PB12 所在端口 GPIOB 的时钟 */
#define LED_PORT         GPIOB					/* 端口 */
#define LED_PIN          GPIO_Pin_12			/* 引脚号：PB12 */

/* 闪烁半周期（亮多久 = 灭多久） */
#define LED_SLOW_HALF_MS 500	/* 慢闪：亮 500ms，灭 500ms（1 秒闪一次） */
#define LED_FAST_HALF_MS 100	/* 快闪：亮 100ms，灭 100ms（1 秒闪 5 次） */

/*----------------------------------------------------------------------------
 * 模块内部变量（static：只在本文件可见）
 *----------------------------------------------------------------------------*/
static LedPattern_t s_pat = LED_PAT_OFF;		/* 当前花样：灭/亮/慢闪/快闪 */
static uint8_t  s_on = 0;						/* 当前亮灭：1=亮，0=灭 */
static uint16_t s_half_ms = LED_SLOW_HALF_MS;	/* 当前花样的半周期（500 或 100） */
static uint32_t s_tick = 0;						/* 本次"亮/灭"开始的时刻 */
static uint32_t s_ms = 0;						/* 模块内部 1ms 计数 */
static uint16_t s_pulse_ms = 0;					/* 脉冲剩余毫秒（0 = 没有脉冲） */

/*----------------------------------------------------------------------------
 * 底层函数：把"亮/灭"写到引脚
 *   本板是低电平点亮，所以：
 *       要亮 → 输出低电平（Bit_RESET）
 *       要灭 → 输出高电平（Bit_SET）
 *----------------------------------------------------------------------------*/
static void LED_WritePin(uint8_t on)
{
	if (on != 0)	/* 要亮 */
	{
		GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);	/* 输出低电平 → 灯亮 */
	}
	else			/* 要灭 */
	{
		GPIO_WriteBit(LED_PORT, LED_PIN, Bit_SET);		/* 输出高电平 → 灯灭 */
	}
}

/**
  * 函数：LED_Init
  * 作用：把 PB12 配成推挽输出，并让灯默认熄灭
  * 调用：程序启动时调用一次
  */
void LED_Init(void)
{
	GPIO_InitTypeDef g;		/* GPIO 配置结构体 */

	RCC_APB2PeriphClockCmd(LED_CLK, ENABLE);	/* 开 GPIOB 时钟（不开时钟配置无效） */

	g.GPIO_Mode  = GPIO_Mode_Out_PP;			/* 推挽输出：能主动输出高低电平 */
	g.GPIO_Speed = GPIO_Speed_50MHz;			/* 输出速度（LED 用不着这么快，填默认值） */
	g.GPIO_Pin   = LED_PIN;						/* 引脚：PB12 */
	GPIO_Init(LED_PORT, &g);					/* 写入配置 */

	s_pat = LED_PAT_OFF;			/* 初始花样：灭 */
	s_on = 0;						/* 初始状态：不亮 */
	s_half_ms = LED_SLOW_HALF_MS;	/* 默认半周期 500ms */
	s_tick = 0;						/* 计时起点 */
	s_ms = 0;						/* 内部毫秒计数清零 */
	s_pulse_ms = 0;					/* 没有脉冲 */

	LED_WritePin(0);				/* 上电先把灯熄灭 */
}

/**
  * 函数：LED_AllOff
  * 作用：立即熄灭，并取消可能正在进行的脉冲
  */
void LED_AllOff(void)
{
	s_pat = LED_PAT_OFF;	/* 花样设为常灭 */
	s_on = 0;				/* 状态改为不亮 */
	s_pulse_ms = 0;			/* 取消脉冲（否则脉冲结束时还会再写一次引脚） */
	LED_WritePin(0);		/* 输出高电平，灯灭 */
}

/**
  * 函数：LED_Set
  * 作用：常亮 或 常灭
  * 参数：on = 1 常亮；on = 0 常灭
  */
void LED_Set(uint8_t on)
{
	s_pulse_ms = 0;		/* 取消可能存在的脉冲 */

	if (on != 0)		/* 传 1：常亮 */
	{
		s_pat = LED_PAT_ON;		/* 记录花样：常亮 */
		s_on = 1;				/* 状态：亮 */
		LED_WritePin(1);		/* 输出低电平，灯亮 */
	}
	else				/* 传 0：常灭 */
	{
		s_pat = LED_PAT_OFF;	/* 记录花样：常灭 */
		s_on = 0;				/* 状态：灭 */
		LED_WritePin(0);		/* 输出高电平，灯灭 */
	}
}

/**
  * 函数：LED_SetPattern
  * 作用：切换闪烁花样（慢闪 / 快闪）
  * 参数：pat = LED_PAT_SLOW（慢闪）或 LED_PAT_FAST（快闪）
  *       传 LED_PAT_OFF / LED_PAT_ON 时会自动转去调用 LED_Set
  *
  * 说明：本函数只"设定"花样和周期，不计时；
  *       真正的翻转由 LED_Tick1ms() 每 1ms 检查一次。
  */
void LED_SetPattern(LedPattern_t pat)
{
	/* 情况 1：要的是"常灭"或"常亮" → 交给 LED_Set 处理，直接返回 */
	if (pat == LED_PAT_OFF || pat == LED_PAT_ON)
	{
		if (pat == LED_PAT_ON)
		{
			LED_Set(1);		/* 常亮 */
		}
		else
		{
			LED_Set(0);		/* 常灭 */
		}
		return;				/* 结束本函数 */
	}

	/* 情况 2：要的是慢闪或快闪 */
	s_pulse_ms = 0;			/* 先取消可能存在的脉冲，避免两个模式打架 */

	s_pat = pat;			/* 记录花样（慢闪或快闪） */

	if (pat == LED_PAT_FAST)	/* 快闪：半周期 100ms */
	{
		s_half_ms = LED_FAST_HALF_MS;
	}
	else						/* 慢闪：半周期 500ms */
	{
		s_half_ms = LED_SLOW_HALF_MS;
	}

	s_tick = s_ms;			/* 从现在开始计时（本次亮/灭的起点） */
	s_on = 1;				/* 状态：亮 */
	LED_WritePin(1);		/* 立即点亮：给用户最快反馈，之后 500ms 才第一次灭 */
}

/**
  * 函数：LED_Pulse
  * 作用：点亮 ms 毫秒后自动熄灭（一次性提示，不改变后续花样）
  * 参数：ms = 亮的时长（毫秒）；传 0 等于直接熄灭
  * 用途：项目二/三"每经过 A、B、C、D 点"时亮一下就灭
  */
void LED_Pulse(uint16_t ms)
{
	if (ms == 0)			/* 时长为 0：直接灭 */
	{
		LED_AllOff();
		return;
	}

	s_pat = LED_PAT_ON;		/* 脉冲期间当作"常亮"处理 */
	s_on = 1;				/* 状态：亮 */
	s_pulse_ms = ms;		/* 记录还剩多少毫秒 */
	LED_WritePin(1);		/* 立即点亮 */
}

/**
  * 函数：LED_Tick1ms
  * 作用：推进闪烁/脉冲，必须每 1ms 调用一次
  * 说明：只做计数比较，几微秒就返回，不影响其它任务
  */
void LED_Tick1ms(void)
{
	s_ms++;		/* 内部毫秒计数 +1 */

	/* ① 如果正在执行"脉冲"（亮一下） */
	if (s_pulse_ms > 0)
	{
		s_pulse_ms--;				/* 剩余时间 -1ms */
		if (s_pulse_ms == 0)		/* 到时间了 → 熄灭 */
		{
			s_pat = LED_PAT_OFF;	/* 花样恢复为常灭 */
			s_on = 0;
			LED_WritePin(0);		/* 输出高电平，灯灭 */
		}
		return;		/* 脉冲期间不处理闪烁 */
	}

	/* ② 只有慢闪/快闪需要翻转；常亮/常灭直接返回 */
	if (s_pat != LED_PAT_SLOW && s_pat != LED_PAT_FAST)
	{
		return;
	}

	/* ③ 距离上次翻转是否已经够半个周期 */
	if ((s_ms - s_tick) >= s_half_ms)
	{
		s_tick = s_ms;		/* 新的计时起点 */

		if (s_on != 0)		/* 原来是亮的 → 灭 */
		{
			s_on = 0;
		}
		else				/* 原来是灭的 → 亮 */
		{
			s_on = 1;
		}

		LED_WritePin(s_on);	/* 把新状态写到引脚 */
	}
}
