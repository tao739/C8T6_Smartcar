#include "stm32f10x.h"
#include "LED.h"

/*==============================================================================
 * LED.c —— 单颗指示灯驱动（S1）
 *----------------------------------------------------------------------------
 * C8T6 过渡板：LED = PB12（板上标"LED 板载"）
 *
 * 两种工作模式：
 *   ① 花样模式：LED_Set / LED_SetPattern 设定常亮/常灭/慢闪/快闪
 *   ② 脉冲模式：LED_Pulse(ms) 亮一下、到时自动灭（用于"经过标志点"的瞬时提示）
 *   脉冲优先级高于花样：脉冲期间花样暂停，脉冲结束后回到灭。
 *
 * 每 1ms 由 LED_Tick1ms() 推进一次 —— 不用 Delay，也不占用定时器。
 *============================================================================*/

/*------------------------- 引脚配置（改接线只改这里） ---------------------*/
#define LED_CLK          RCC_APB2Periph_GPIOB
#define LED_PORT         GPIOB
#define LED_PIN          GPIO_Pin_12
#define LED_ACTIVE_LOW   1				/* 1 = 低电平点亮；若灯反了改成 0 */

/* 花样参数 */
#define LED_SLOW_HALF_MS 500
#define LED_FAST_HALF_MS 100

static uint8_t  s_pat = LED_PAT_OFF;			/* 当前花样 */
static uint8_t  s_on = 0;						/* 当前亮灭 */
static uint16_t s_half_ms = LED_SLOW_HALF_MS;	/* 半个周期 */
static uint32_t s_tick = 0;						/* 本次亮/灭开始的时刻 */
static uint32_t s_ms = 0;						/* 内部 1ms 计数 */
static uint16_t s_pulse_ms = 0;					/* 脉冲剩余毫秒数，0=无脉冲 */

/* 底层：把"亮灭"写到引脚（考虑低电平点亮） */
static void LED_WritePin(uint8_t on)
{
	BitAction level;

#if (LED_ACTIVE_LOW != 0)
	level = (on != 0) ? Bit_RESET : Bit_SET;
#else
	level = (on != 0) ? Bit_SET : Bit_RESET;
#endif
	GPIO_WriteBit(LED_PORT, LED_PIN, level);
}

/**
  * 函    数：指示灯初始化（上电全灭）
  */
void LED_Init(void)
{
	GPIO_InitTypeDef g;

	RCC_APB2PeriphClockCmd(LED_CLK, ENABLE);

	g.GPIO_Mode  = GPIO_Mode_Out_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Pin   = LED_PIN;
	GPIO_Init(LED_PORT, &g);

	s_pat = LED_PAT_OFF;
	s_on = 0;
	s_half_ms = LED_SLOW_HALF_MS;
	s_tick = 0;
	s_ms = 0;
	s_pulse_ms = 0;

	LED_WritePin(0);
}

/**
  * 函    数：全部熄灭（同时取消脉冲）
  */
void LED_AllOff(void)
{
	s_pat = LED_PAT_OFF;
	s_on = 0;
	s_pulse_ms = 0;
	LED_WritePin(0);
}

/**
  * 函    数：常亮 / 常灭
  */
void LED_Set(uint8_t on)
{
	s_pulse_ms = 0;
	s_pat = (on != 0) ? LED_PAT_ON : LED_PAT_OFF;
	s_on = on;
	LED_WritePin(on);
}

/**
  * 函    数：切换闪烁花样
  */
void LED_SetPattern(LedPattern_t pat)
{
	if ((pat == LED_PAT_OFF) || (pat == LED_PAT_ON))
	{
		LED_Set((uint8_t)((pat == LED_PAT_ON) ? 1 : 0));
		return;
	}

	s_pulse_ms = 0;
	s_pat = (uint8_t)pat;
	s_half_ms = (uint16_t)((pat == LED_PAT_FAST) ? LED_FAST_HALF_MS : LED_SLOW_HALF_MS);
	s_tick = s_ms;
	s_on = 1;
	LED_WritePin(1);
}

/**
  * 函    数：一次性点亮 ms 毫秒后自动熄灭（"经过标志点"的瞬时提示）
  * 说    明：脉冲期间忽略花样；ms=0 时等价于熄灭
  */
void LED_Pulse(uint16_t ms)
{
	if (ms == 0)
	{
		LED_AllOff();
		return;
	}

	s_pat = LED_PAT_ON;			/* 脉冲期间视为"常亮"，但由 s_pulse_ms 计时 */
	s_on = 1;
	s_pulse_ms = ms;
	LED_WritePin(1);
}

/**
  * 函    数：闪烁/脉冲推进，每 1ms 调用一次
  */
void LED_Tick1ms(void)
{
	s_ms++;

	/* ① 脉冲模式：亮 ms 毫秒后自动灭 */
	if (s_pulse_ms > 0)
	{
		s_pulse_ms--;
		if (s_pulse_ms == 0)
		{
			s_pat = LED_PAT_OFF;
			s_on = 0;
			LED_WritePin(0);
		}
		return;
	}

	/* ② 花样模式：慢闪/快闪 */
	if ((s_pat != LED_PAT_SLOW) && (s_pat != LED_PAT_FAST)) { return; }

	if ((s_ms - s_tick) >= s_half_ms)
	{
		s_tick = s_ms;
		s_on = (uint8_t)((s_on != 0) ? 0 : 1);
		LED_WritePin(s_on);
	}
}
