#include "stm32f10x.h"
#include "Buzzer.h"

/*==============================================================================
 * Buzzer.c —— 无源蜂鸣器（S1）
 *----------------------------------------------------------------------------
 * 时钟路径：系统 72MHz → APB2 → TIM1；PSC=71 → 计数时钟 1MHz
 *           ARR 决定频率（1MHz/(ARR+1)），CCR 决定占空比（这里固定 50%）
 *           发声中 CCR = (ARR+1)/2；不响时 CCR = 0（PWM1 模式下输出恒低）
 *
 * 节奏播放是一个小状态机（响 → 间隔 → 响 …），由 Buzzer_Tick1ms() 推进，
 * 内部自带 1ms 计数，不依赖其它模块。
 *============================================================================*/

/*------------------------- 引脚配置（改接线只改这里） ---------------------*/
#define BUZ_CLK          RCC_APB2Periph_GPIOA
#define BUZ_TIM          TIM1
#define BUZ_TIM_CLK      RCC_APB2Periph_TIM1
#define BUZ_PORT         GPIOA
#define BUZ_PIN          GPIO_Pin_8

/* 播放状态 */
#define BUZ_PHASE_IDLE   0
#define BUZ_PHASE_ON     1
#define BUZ_PHASE_OFF    2

static uint32_t s_ms = 0;			/* 内部 1ms 计数 */
static uint16_t s_arr = 499;		/* 当前 ARR（对应 2kHz） */
static uint8_t  s_phase = BUZ_PHASE_IDLE;
static uint32_t s_phase_tick = 0;
static uint8_t  s_count = 0;		/* 还要响几声 */
static uint16_t s_on_ms = 0;
static uint16_t s_off_ms = 0;

/**
  * 函    数：蜂鸣器初始化（PA8 = TIM1_CH1）
  */
void Buzzer_Init(void)
{
	GPIO_InitTypeDef g;
	TIM_TimeBaseInitTypeDef t;
	TIM_OCInitTypeDef oc;

	RCC_APB2PeriphClockCmd(BUZ_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(BUZ_TIM_CLK, ENABLE);

	/* PA8 复用推挽输出 */
	g.GPIO_Mode  = GPIO_Mode_AF_PP;
	g.GPIO_Pin   = BUZ_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BUZ_PORT, &g);

	/* 时基：72MHz/72 = 1MHz；ARR=499 → 2kHz */
	TIM_InternalClockConfig(BUZ_TIM);
	t.TIM_ClockDivision     = TIM_CKD_DIV1;
	t.TIM_CounterMode       = TIM_CounterMode_Up;
	t.TIM_Period            = 499;
	t.TIM_Prescaler         = 72 - 1;
	t.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(BUZ_TIM, &t);

	/* 通道 1：PWM1 模式，初始占空比 0（不响） */
	oc.TIM_OCMode       = TIM_OCMode_PWM1;
	oc.TIM_OutputState  = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Disable;
	oc.TIM_Pulse        = 0;
	oc.TIM_OCPolarity   = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(BUZ_TIM, &oc);
	TIM_OC1PreloadConfig(BUZ_TIM, TIM_OCPreload_Enable);

	/* 高级定时器必须开主输出，否则引脚上没有波形 */
	TIM_CtrlPWMOutputs(BUZ_TIM, ENABLE);
	TIM_Cmd(BUZ_TIM, ENABLE);

	s_arr = 499;
	s_phase = BUZ_PHASE_IDLE;
	s_count = 0;
	s_ms = 0;
	Buzzer_Off();
}

/**
  * 函    数：设置音调（100~6000Hz，超出自动限幅）
  */
void Buzzer_SetFreq(uint16_t hz)
{
	uint32_t arr;

	if (hz < 100)  { hz = 100; }
	if (hz > 6000) { hz = 6000; }

	arr = 1000000UL / hz;
	if (arr > 0) { arr--; }
	s_arr = (uint16_t)arr;
	TIM_SetAutoreload(BUZ_TIM, s_arr);
}

/**
  * 函    数：开始发声（50% 占空比）
  */
void Buzzer_On(void)
{
	TIM_SetCompare1(BUZ_TIM, (uint16_t)((s_arr + 1) / 2));
}

/**
  * 函    数：停止发声
  */
void Buzzer_Off(void)
{
	TIM_SetCompare1(BUZ_TIM, 0);
}

/**
  * 函    数：立即静音并停止节奏
  */
void Buzzer_Stop(void)
{
	Buzzer_Off();
	s_phase = BUZ_PHASE_IDLE;
	s_count = 0;
}

/**
  * 函    数：播放一段节奏
  * 参    数：count 响几声；on_ms 每声时长；off_ms 间隔；hz 音调
  */
void Buzzer_Play(uint8_t count, uint16_t on_ms, uint16_t off_ms, uint16_t hz)
{
	Buzzer_Stop();
	if (count == 0) { return; }

	s_count   = count;
	s_on_ms   = on_ms;
	s_off_ms  = off_ms;
	Buzzer_SetFreq(hz);
	Buzzer_On();
	s_phase = BUZ_PHASE_ON;
	s_phase_tick = s_ms;
}

/**
  * 函    数：节奏推进，每 1ms 调用一次
  */
void Buzzer_Tick1ms(void)
{
	s_ms++;

	if (s_phase == BUZ_PHASE_IDLE) { return; }

	if (s_phase == BUZ_PHASE_ON)
	{
		if ((s_ms - s_phase_tick) >= s_on_ms)
		{
			Buzzer_Off();
			s_count--;
			if (s_count == 0)
			{
				s_phase = BUZ_PHASE_IDLE;
				return;
			}
			s_phase = BUZ_PHASE_OFF;
			s_phase_tick = s_ms;
		}
	}
	else /* BUZ_PHASE_OFF */
	{
		if ((s_ms - s_phase_tick) >= s_off_ms)
		{
			Buzzer_On();
			s_phase = BUZ_PHASE_ON;
			s_phase_tick = s_ms;
		}
	}
}

/**
  * 函    数：是否还在响
  */
uint8_t Buzzer_IsBusy(void)
{
	return (uint8_t)((s_phase != BUZ_PHASE_IDLE) ? 1 : 0);
}
