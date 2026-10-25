#ifndef __TASK1_H
#define __TASK1_H

#include "stm32f10x.h"

/*
 * 任务一：A -> B 定点停车
 *
 * 这个模块只做“感知 + 状态判断”，不直接驱动电机/蜂鸣器。
 * 原因：电机、蜂鸣器、按键引脚由 S1/S3 的整车工程决定，
 * S2 不能猜引脚，否则可能烧板。
 *
 * S1/S3 需要做：
 *   1. 一键启动时调用 Task1_Start()
 *   2. 每 10ms 调用 Task1_Update()
 *   3. 若 Task1_StopRequested()==1 -> 停车
 *   4. 若 Task1_GetEvent()!=NONE -> 鸣笛/提示，然后 Task1_ClearEvent()
 */

#define TASK1_UPDATE_MS                 10U
#define TASK1_TIMEOUT_MS                13500U   /* 软件10ms节拍有开销，留足余量保证实际<=15s */
#define TASK1_WHITE_CONFIRM_FRAMES      5U       /* 连续 5 帧全白，确认离开 A */
#define TASK1_BLACK_CONFIRM_FRAMES      3U       /* 连续 3 帧任意黑，确认 B */

typedef enum {
    TASK1_IDLE = 0,
    TASK1_WAIT_A_CLEAR,     /* 在 A 黑区，等全白 */
    TASK1_RUN_TO_B,         /* A-B 白道直行，等 B 黑 */
    TASK1_AT_B,             /* 已到 B，请求停车/鸣笛 */
    TASK1_TIMEOUT           /* 超时，请求停车/报警 */
} Task1_State;

typedef enum {
    TASK1_EVENT_NONE = 0,
    TASK1_EVENT_START,
    TASK1_EVENT_AT_B,
    TASK1_EVENT_TIMEOUT
} Task1_Event;

void Task1_Init(void);
void Task1_Start(void);
void Task1_Update(void);            /* 必须每 10ms 调用一次 */

Task1_State Task1_GetState(void);
Task1_Event Task1_GetEvent(void);
void Task1_ClearEvent(void);

uint8_t Task1_StopRequested(void);  /* AT_B 或 TIMEOUT 时为 1 */
uint32_t Task1_GetElapsedMs(void);

#endif
