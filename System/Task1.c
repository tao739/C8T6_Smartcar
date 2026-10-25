#include "Task1.h"
#include "S2_IR.h"

static Task1_State s_state;
static Task1_Event s_event;
static uint32_t s_elapsed_ms;
static uint8_t s_white_count;
static uint8_t s_black_count;

void Task1_Init(void)
{
    s_state = TASK1_IDLE;
    s_event = TASK1_EVENT_NONE;
    s_elapsed_ms = 0U;
    s_white_count = 0U;
    s_black_count = 0U;
}

void Task1_Start(void)
{
    s_state = TASK1_WAIT_A_CLEAR;
    s_event = TASK1_EVENT_START;
    s_elapsed_ms = 0U;
    s_white_count = 0U;
    s_black_count = 0U;
}

void Task1_Update(void)
{
    if (s_state == TASK1_IDLE) {
        return;
    }

    s_elapsed_ms += TASK1_UPDATE_MS;

    switch (s_state) {
    case TASK1_WAIT_A_CLEAR:
        /*
         * A 点本身是黑的，不能在这里判 B。
         * 必须连续全白，确认已经离开 A 黑区。
         */
        if (S2_IR_AllWhite()) {
            s_white_count++;
            if (s_white_count >= TASK1_WHITE_CONFIRM_FRAMES) {
                s_state = TASK1_RUN_TO_B;
                s_white_count = 0U;
                s_black_count = 0U;
            }
        } else {
            s_white_count = 0U;
        }
        break;

    case TASK1_RUN_TO_B:
        /*
         * 任意一路黑都算 B：
         * 中间、左边、右边都可能，取决于小车是否完全直行。
         * 8 路全黑也算，所以用 AnyBlack，不用 line_found。
         */
        if (S2_IR_AnyBlack()) {
            s_black_count++;
            if (s_black_count >= TASK1_BLACK_CONFIRM_FRAMES) {
                s_state = TASK1_AT_B;
                s_event = TASK1_EVENT_AT_B;
                s_black_count = 0U;
            }
        } else {
            s_black_count = 0U;
        }
        break;

    case TASK1_AT_B:
    case TASK1_TIMEOUT:
    default:
        break;
    }

    /* 超时兜底：B 优先于超时；只有还在找 B 时才判超时 */
    if (((s_state == TASK1_WAIT_A_CLEAR) || (s_state == TASK1_RUN_TO_B)) &&
        (s_elapsed_ms >= TASK1_TIMEOUT_MS)) {
        s_state = TASK1_TIMEOUT;
        s_event = TASK1_EVENT_TIMEOUT;
    }
}

Task1_State Task1_GetState(void)
{
    return s_state;
}

Task1_Event Task1_GetEvent(void)
{
    return s_event;
}

void Task1_ClearEvent(void)
{
    s_event = TASK1_EVENT_NONE;
}

uint8_t Task1_StopRequested(void)
{
    return ((s_state == TASK1_AT_B) || (s_state == TASK1_TIMEOUT)) ? 1U : 0U;
}

uint32_t Task1_GetElapsedMs(void)
{
    return s_elapsed_ms;
}
