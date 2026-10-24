#include "S2_IR.h"
#include "AD.h"

/*
 * 11mm 高度、C8T6 测试板的阈值。
 * 黑线 = raw 变大；每路用自己的阈值，不用全局阈值。
 */
static const uint16_t k_threshold[S2_IR_COUNT] = {
    3980, 3952, 3938, 3949,
    3943, 3937, 3970, 3977
};

/* 回差，防止阈值附近抖动 */
#define S2_IR_HYST  30

static S2_IR_State s_state;
static uint8_t s_active[S2_IR_COUNT];

void S2_IR_Init(void)
{
    uint8_t i;

    for (i = 0U; i < S2_IR_COUNT; i++) {
        s_state.raw[i] = 0U;
        s_active[i] = 0U;
    }

    s_state.active_mask = 0U;
    s_state.line_found = 0U;
    s_state.position_mm = 0;
    s_state.valid = 0U;
    s_state.confidence = 0U;
}

void S2_IR_Update(void)
{
    uint8_t i;
    uint8_t count = 0U;
    uint32_t sum_w = 0U;
    uint32_t sum_iw = 0U;
    uint8_t mask = 0U;

    for (i = 0U; i < S2_IR_COUNT; i++) {
        uint16_t raw = AD_Value[i];
        s_state.raw[i] = raw;

        /* 带回差的二值化：进入黑线要 > th+30，离开要 < th-30 */
        if (raw > (uint16_t)(k_threshold[i] + S2_IR_HYST)) {
            s_active[i] = 1U;
        } else if (raw < (uint16_t)(k_threshold[i] - S2_IR_HYST)) {
            s_active[i] = 0U;
        }

        if (s_active[i] != 0U) {
            uint16_t w = (uint16_t)(raw - k_threshold[i]);

            mask |= (uint8_t)(1U << i);
            count++;
            sum_w += w;
            sum_iw += (uint32_t)w * i;
        }
    }

    s_state.active_mask = mask;

    /*
     * 1~7 路为黑：正常压线。
     * 0 路：丢线。
     * 8 路全黑：可能是宽黑区/十字/起点，先判为不可信，交给S1处理。
     */
    if ((count >= 1U) && (count <= 7U) && (sum_w > 0U)) {
        int32_t pos;

        s_state.line_found = 1U;
        s_state.valid = 1U;

        /* 加权平均：索引0在最左，索引7在最右 */
        pos = (int32_t)((sum_iw * S2_IR_PITCH_MM) / sum_w);
        pos -= (int32_t)(((S2_IR_COUNT - 1U) * S2_IR_PITCH_MM) / 2U);
        s_state.position_mm = (int16_t)pos;

        /* 简单置信度：参与加权的黑度总和，800以上算100 */
        if (sum_w >= 800U) {
            s_state.confidence = 100U;
        } else {
            s_state.confidence = (uint8_t)(sum_w / 8U);
        }
    } else {
        s_state.line_found = 0U;
        s_state.valid = 0U;
        s_state.position_mm = 0;
        s_state.confidence = 0U;
    }
}

const S2_IR_State *S2_IR_Get(void)
{
    return &s_state;
}

/*
 * 任务一用：任意一路黑。
 * 包含 8 路全黑；不依赖 line_found。
 */
uint8_t S2_IR_AnyBlack(void)
{
    return (s_state.active_mask != 0U) ? 1U : 0U;
}

/*
 * 任务一用：全白。
 * A 点黑区起步后，先等 AllWhite 连续几帧，再允许 B 检测。
 */
uint8_t S2_IR_AllWhite(void)
{
    return (s_state.active_mask == 0U) ? 1U : 0U;
}
