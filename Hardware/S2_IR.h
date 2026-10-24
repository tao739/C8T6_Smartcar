#ifndef __S2_IR_H
#define __S2_IR_H

#include "stm32f10x.h"

#define S2_IR_COUNT        8U
#define S2_IR_PITCH_MM     8      /* 探头中心间距，先按8mm；实测不同就改这里 */

typedef struct {
    uint16_t raw[S2_IR_COUNT];      /* 8路原始值 */
    uint8_t  active_mask;           /* bit0=D1 ... bit7=D8，1=黑线 */
    uint8_t  line_found;            /* 1=看到线 */
    int16_t  position_mm;           /* 左负右正，中心0 */
    uint8_t  valid;                 /* 1=数据可信 */
    uint8_t  confidence;            /* 0~100，越大越可信 */
} S2_IR_State;

void S2_IR_Init(void);
void S2_IR_Update(void);            /* 建议每10ms调用一次 */
const S2_IR_State *S2_IR_Get(void);

/*
 * 任务一 B 点检测用：
 *   S2_IR_AllWhite()  -> A-B 白道确认，用来“武装”B 检测
 *   S2_IR_AnyBlack()  -> 任意一路黑，包含 8 路全黑
 *
 * 注意：不要用 line_found 判断 B 点。
 * 8 路全黑时 line_found=0，但 AnyBlack 仍然返回 1。
 */
uint8_t S2_IR_AnyBlack(void);
uint8_t S2_IR_AllWhite(void);

#endif
