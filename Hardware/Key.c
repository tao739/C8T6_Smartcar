#include "stm32f10x.h"
#include "Key.h"

/*==============================================================================
 * Key.c —— 按键消抖与短按/长按识别（S1）
 *----------------------------------------------------------------------------
 * C8T6 过渡板：KEY = PB9（图上标 KEY2），按下接地 = 低电平，内部上拉
 *
 * 识别逻辑（全部非阻塞，每 1ms 推进一次）：
 *   ① 20ms 计数消抖：电平连续 20 次一致才认
 *   ② 刚按下：清零按住计时
 *   ③ 按住期间累计毫秒数，满 1000ms 立刻产生"长按"事件（只触发一次）
 *   ④ 刚松开：如果这次没触发过长按 → 产生"短按"事件
 *============================================================================*/

/*------------------------- 引脚配置（改接线只改这里） ---------------------*/
#define KEY_CLK          RCC_APB2Periph_GPIOB
#define KEY_PORT         GPIOB
#define KEY_PIN          GPIO_Pin_9
#define KEY_ACTIVE       0			/* 0 = 按下低电平（按键另一端接 GND） */

#define KEY_DEBOUNCE_MS  20			/* 消抖时间 */
#define KEY_LONG_MS      1000		/* 长按判定时间 */

static uint8_t    s_stable = 0;		/* 消抖后的状态：1=按下 */
static uint8_t    s_cnt = 0;		/* 候选电平连续次数 */
static uint16_t   s_press_ms = 0;	/* 本次按住已持续毫秒数 */
static uint8_t    s_long_fired = 0;	/* 本次按住是否已触发长按 */
static KeyEvent_t s_evt = KEY_EVT_NONE;

/**
  * 函    数：按键初始化
  */
void Key_Init(void)
{
	GPIO_InitTypeDef g;

	RCC_APB2PeriphClockCmd(KEY_CLK, ENABLE);

	/* 低电平有效 → 内部上拉；高电平有效 → 内部下拉 */
	g.GPIO_Mode  = (KEY_ACTIVE != 0) ? GPIO_Mode_IPD : GPIO_Mode_IPU;
	g.GPIO_Pin   = KEY_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_PORT, &g);

	s_stable = 0;
	s_cnt = 0;
	s_press_ms = 0;
	s_long_fired = 0;
	s_evt = KEY_EVT_NONE;
}

/**
  * 函    数：读按键的"按下状态"（1=按下）
  */
static uint8_t Key_RawPressed(void)
{
	uint8_t level = (uint8_t)((GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) != 0) ? 1 : 0);

	if (KEY_ACTIVE != 0) { return level; }
	return (uint8_t)((level == 0) ? 1 : 0);
}

/**
  * 函    数：按键状态机，每 1ms 调用一次
  */
void Key_Tick1ms(void)
{
	uint8_t raw = Key_RawPressed();

	/* ① 消抖：连续 KEY_DEBOUNCE_MS 次一致才认 */
	if (raw != s_stable)
	{
		s_cnt++;
		if (s_cnt >= KEY_DEBOUNCE_MS)
		{
			s_cnt = 0;
			s_stable = raw;

			if (raw != 0)			/* 刚按下：重新计时 */
			{
				s_press_ms = 0;
				s_long_fired = 0;
			}
			else					/* 刚松开：本次没触发长按 → 短按事件 */
			{
				if ((s_long_fired == 0) && (s_evt == KEY_EVT_NONE))
				{
					s_evt = KEY_EVT_START;
				}
			}
		}
	}
	else
	{
		s_cnt = 0;
	}

	/* ② 按住期间计时：满 KEY_LONG_MS 立即产生长按事件（只发一次） */
	if (s_stable != 0)
	{
		if (s_press_ms < 30000U) { s_press_ms++; }

		if ((s_long_fired == 0) && (s_press_ms >= KEY_LONG_MS))
		{
			s_long_fired = 1;
			if (s_evt == KEY_EVT_NONE)
			{
				s_evt = KEY_EVT_STOP;
			}
		}
	}
	else
	{
		s_press_ms = 0;
	}
}

/**
  * 函    数：取按键事件（取走后清空）
  */
KeyEvent_t Key_GetEvent(void)
{
	KeyEvent_t e = s_evt;
	s_evt = KEY_EVT_NONE;
	return e;
}

/**
  * 函    数：清空未处理事件（启动/复位时防残留误触发）
  */
void Key_Flush(void)
{
	s_evt = KEY_EVT_NONE;
}

/**
  * 函    数：当前是否按住（调试用）
  */
uint8_t Key_IsPressed(void)
{
	return s_stable;
}
