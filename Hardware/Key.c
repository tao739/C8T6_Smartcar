#include "stm32f10x.h"
#include "Key.h"

/*==============================================================================
 * 文件：Key.c
 * 作用：按键消抖 + 短按/长按识别（S1 负责）
 * 硬件：KEY = PB9，另一端接 GND（按下 = 低电平），内部上拉
 *----------------------------------------------------------------------------
 * 工作流程（全部非阻塞，每 1ms 被 Key_Tick1ms 推进一次）：
 *
 *   ┌─ 采一次电平 raw
 *   │
 *   ├─ raw 和 s_stable（消抖后的状态）一样吗？
 *   │    一样 → 计数清零 s_cnt = 0（说明稳定）
 *   │    不一样 → 计数 +1；若连续 20 次都不一样 → 才真的改状态 s_stable
 *   │             刚按下(raw=1)：按住计时清零、长按标志清零
 *   │             刚松开(raw=0)：如果这次没发过长按 → 发短按事件
 *   │
 *   └─ 如果正按着：按住毫秒数 +1；满 1000ms 且没发过 → 发长按事件（只发一次）
 *
 * 关键概念：
 *   状态（s_stable）是"一直持续的"（按键现在按没按着）；
 *   事件（s_evt）是"一次性发生的"（刚刚发生了什么），主循环取走后清空。
 *============================================================================*/

/*----------------------------------------------------------------------------
 * 引脚配置区（换板子/换按键只改这里）
 * 如果按键改为接 3.3V（按下高电平）：把 KEY_ACTIVE 改成 1 即可，
 * 下面的 GPIO_Mode 会自动从"上拉"切成"下拉"，读逻辑也会自动反转。
 *----------------------------------------------------------------------------*/
#define KEY_CLK          RCC_APB2Periph_GPIOB	/* PB9 所在端口 GPIOB 的时钟 */
#define KEY_PORT         GPIOB					/* 端口 */
#define KEY_PIN          GPIO_Pin_9				/* 引脚号 */
#define KEY_ACTIVE       0						/* 0=按下低电平(接GND)  1=按下高电平(接3.3V) */

/* 时间参数 */
#define KEY_DEBOUNCE_MS  20			/* 消抖时间：电平连续 20ms 一致才承认（机械抖动一般 <10ms） */
#define KEY_LONG_MS      1000		/* 长按门槛：按住 1000ms 判定为长按 */

/*----------------------------------------------------------------------------
 * 模块内部状态（static = 只在本文件可见；别人不能直接改，只能通过函数）
 *----------------------------------------------------------------------------*/
static uint8_t    s_stable = 0;		/* 消抖后的状态：1=按下，0=松开 */
static uint8_t    s_cnt = 0;		/* 候选电平的连续次数（消抖计数器 0~20） */
static uint16_t   s_press_ms = 0;	/* 本次按住已经持续了多少毫秒（长按计时） */
static uint8_t    s_long_fired = 0;	/* 本次按住过程中，长按事件是否已经发过（防重复） */
static KeyEvent_t s_evt = KEY_EVT_NONE;	/* 事件信箱：放着"刚刚发生了什么"，没事件时为 NONE */

/**
  * 函数：Key_Init
  * 作用：把 PB9 配置成输入（内部上拉/下拉由 KEY_ACTIVE 自动决定），并清空状态
  * 调用：程序启动时调用一次
  */
void Key_Init(void)
{
	GPIO_InitTypeDef g;		/* GPIO 配置结构体 */

	/* 开 GPIOB 时钟（不开时钟配置无效） */
	RCC_APB2PeriphClockCmd(KEY_CLK, ENABLE);

	/* 选择输入模式：
	 *   KEY_ACTIVE != 0（按下高电平，按键接 3.3V）→ GPIO_Mode_IPD（内部下拉，平时被拉低）
	 *   KEY_ACTIVE == 0（按下低电平，按键接 GND） → GPIO_Mode_IPU（内部上拉，平时被拉高）
	 * 为什么要上/下拉：按键松开时引脚是"悬空"的，电平随机，
	 *   必须用内部电阻把它拉到确定电平，才能可靠判断"没按"。*/
	g.GPIO_Mode  = (KEY_ACTIVE != 0) ? GPIO_Mode_IPD : GPIO_Mode_IPU;
	g.GPIO_Pin   = KEY_PIN;				/* 引脚：PB9 */
	g.GPIO_Speed = GPIO_Speed_50MHz;	/* 输入模式下速度无实际意义，填默认值 */
	GPIO_Init(KEY_PORT, &g);			/* 写入 GPIOB 配置寄存器 */

	/* 清空内部状态，保证上电第一次扫描是干净的 */
	s_stable = 0;			/* 初始认为"没按下" */
	s_cnt = 0;				/* 消抖计数清零 */
	s_press_ms = 0;			/* 按住时间清零 */
	s_long_fired = 0;		/* 长按标志清零 */
	s_evt = KEY_EVT_NONE;	/* 事件信箱清空 */
}

/**
  * 函数：Key_RawPressed
  * 作用：读一次引脚的"按下状态"（不做消抖）
  * 返回：1 = 按下，0 = 松开
  * 说明：static 函数，只在本文件内部使用
  */
static uint8_t Key_RawPressed(void)
{
	/* 读引脚电平：1=高电平，0=低电平 */
	uint8_t level = (uint8_t)((GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) != 0) ? 1 : 0);

	/* 根据"按下是什么电平"换算成统一的"1=按下"：
	 *   KEY_ACTIVE=1：高电平就是按下 → 直接返回 level
	 *   KEY_ACTIVE=0：低电平才是按下 → 取反返回 */
	if (KEY_ACTIVE != 0) { return level; }
	return (uint8_t)((level == 0) ? 1 : 0);
}

/**
  * 函数：Key_Tick1ms
  * 作用：按键状态机，必须每 1ms 调用一次
  * 说明：本函数不做任何等待/延时，执行时间只有几微秒
  */
void Key_Tick1ms(void)
{
	uint8_t raw = Key_RawPressed();		/* 采一次当前电平（1=按下，0=松开） */

	/*========================== 第一部分：消抖 ==========================*/
	if (raw != s_stable)				/* 读到的和"认定的状态"不一样 */
	{
		s_cnt++;						/* 候选电平连续次数 +1 */

		if (s_cnt >= KEY_DEBOUNCE_MS)	/* 连续 20 次（20ms）都一样 → 接受这次变化 */
		{
			s_cnt = 0;					/* 计数清零，准备下次变化 */
			s_stable = raw;				/* 更新"认定的状态" */

			if (raw != 0)				/* 刚刚被确认"按下" */
			{
				s_press_ms = 0;			/* 按住时间从 0 开始计 */
				s_long_fired = 0;		/* 这一次按住还没发过长按 */
			}
			else						/* 刚刚被确认"松开" */
			{
				/* 如果这次按住过程中没触发过长按 → 现在补发一个"短按事件"
				 * （如果触发过长按，就什么都不发，避免长按后又误触发短按） */
				if ((s_long_fired == 0) && (s_evt == KEY_EVT_NONE))
				{
					s_evt = KEY_EVT_START;
				}
			}
		}
	}
	else								/* 电平没变（稳定） */
	{
		s_cnt = 0;						/* 计数清零：只要抖回去一次，前面的计数就作废 */
	}

	/*==================== 第二部分：按住期间的长按计时 ====================*/
	if (s_stable != 0)					/* 当前处于"按下"状态 */
	{
		if (s_press_ms < 30000U) { s_press_ms++; }	/* 按住毫秒数 +1（封顶 30 秒，防 uint16 溢出） */

		/* 满 1000ms 且这次还没发过 → 立即产生长按事件
		 * 注意是"按住时"触发，不是松手时，所以急停响应最快。
		 * s_long_fired 保证一次按住只发一次，不会每毫秒都发。 */
		if ((s_long_fired == 0) && (s_press_ms >= KEY_LONG_MS))
		{
			s_long_fired = 1;			/* 标记：这次按住的"长按"已经发过 */
			if (s_evt == KEY_EVT_NONE)	/* 信箱空着才放（不覆盖未取走的事件） */
			{
				s_evt = KEY_EVT_STOP;
			}
		}
	}
	else								/* 当前是"松开"状态 */
	{
		s_press_ms = 0;					/* 按住时间清零 */
	}
}

/**
  * 函数：Key_GetEvent
  * 作用：取走一个事件（如果没有事件返回 KEY_EVT_NONE）
  * 说明：取走后信箱立即清空 —— 保证一次事件只会被处理一次
  *       （如果不取走就重复读，会重复执行启动/复位动作）
  */
KeyEvent_t Key_GetEvent(void)
{
	KeyEvent_t e = s_evt;	/* 先把当前事件保存下来 */
	s_evt = KEY_EVT_NONE;	/* 清空信箱 */
	return e;				/* 把事件交给调用者 */
}

/**
  * 函数：Key_Flush
  * 作用：清空未处理的事件
  * 用途：启动、复位时调一下，防止把"启动过程中残留的按键事件"带到下一个状态
  */
void Key_Flush(void)
{
	s_evt = KEY_EVT_NONE;
}

/**
  * 函数：Key_IsPressed
  * 作用：查询当前是否按住（消抖后的状态）
  * 用途：调试/自检时在串口打印出来看按键是否正常
  */
uint8_t Key_IsPressed(void)
{
	return s_stable;
}
