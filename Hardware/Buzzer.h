#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f10x.h"

/*==============================================================================
 * Buzzer.h —— 无源蜂鸣器驱动（S1）
 *----------------------------------------------------------------------------
 * 硬件：PA8 = TIM1_CH1，输出 PWM 方波（无源蜂鸣器必须给方波才会响）。
 *       改音调 = 改 PWM 频率；响/停 = 改占空比（50% / 0）。
 *
 * 用法（非阻塞）：
 *   Buzzer_Play(3, 120, 120, 2200);   // 响 3 声，每声 120ms，间隔 120ms，2200Hz
 *   Buzzer_Tick1ms();                 // 每 1ms 调用一次，推进节奏
 *   Buzzer_Stop();                    // 立即静音
 *
 * 注意：TIM1 是高级定时器，初始化里必须调用 TIM_CtrlPWMOutputs() 打开主输出，
 *       否则 PA8 上永远没有波形（这是第一次用 TIM1 最容易踩的坑）。
 *============================================================================*/

void    Buzzer_Init(void);
void    Buzzer_On(void);						/* 开始发声（50% 占空比） */
void    Buzzer_Off(void);						/* 停止发声（占空比 0） */
void    Buzzer_SetFreq(uint16_t hz);			/* 设置音调 */
void    Buzzer_Play(uint8_t count, uint16_t on_ms, uint16_t off_ms, uint16_t hz);
void    Buzzer_Stop(void);						/* 立即静音并停止节奏 */
void    Buzzer_Tick1ms(void);					/* 每 1ms 调用一次 */
uint8_t Buzzer_IsBusy(void);					/* 1=还在响 */

#endif
