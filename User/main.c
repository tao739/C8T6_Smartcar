/*==============================================================================
 * 文件名：main.c
 * 作用  ：S1（系统与软件负责人）主控程序
 *          项目一：A → B 直线行驶 + 定点停车 + 停车声光提示
 * 平台  ：STM32F103C8T6（过渡板）+ 标准外设库 SPL
 * 工程  ：D:\SmartCar\C8T6_Smartcar\project.uvprojx
 *----------------------------------------------------------------------------
 * 一、本板引脚分配（来自 H 的《STM32F103C8T6 引脚分配图》）
 *
 *   功能            引脚              说明 / 谁负责
 *   ------------    ----------------  --------------------------------------
 *   红外 D1~D7      PA1 ~ PA7         ADC1_IN1~IN7，S2 负责采集
 *   红外 D8         PB1               ADC1_IN9，S2 负责采集
 *   按键            PB9               按下 = 低电平（另一端接 GND），S1
 *   LED             PB12              板载 LED，低电平点亮，S1
 *   蜂鸣器          PA8               有源蜂鸣器，低电平触发（GPIO 开关），S1
 *   1ms 时基        TIM2              只用定时器，不占引脚，S1
 *   串口日志        PA9 / PA10        USART1 115200，S2 的 Serial.c
 *   电机 PWM        PB6 / PB7         TIM4_CH1 / CH2（S3 负责，本文件不实现）
 *   电机方向        PB13/PB14/PB15/PB0  AIN1/AIN2/BIN1/BIN2（S3）
 *   电机 STBY       PB8               高电平使能（S3）
 *   编码器（可选）  PB4/PB5、PA15/PB3 需要重映射 + 关 JTAG（S3，本项目暂不用）
 *   下载调试        PA13 / PA14       SWD，任何人不许占用
 *
 * 二、本文件负责什么（S1 的分工）
 *   · 一键启动（短按 PB9）
 *   · 整车状态机（待启动 / 运行 / 完成 / 超时 / 急停）
 *   · 声光提示（蜂鸣器 + LED 的组合，满足考核"停车有明确声光提示"）
 *   · 异常处理（13.5 秒未到 B 自动停车报警；长按 PB9 急停）
 *   · 复位流程（完成/异常后短按 PB9 回到待启动）
 *   · 1ms / 10ms / 500ms 三种节拍的调度
 *   · 串口调试指令（RUN / STOP / RESET / STATUS，按键没接好时的后门）
 *
 * 三、依赖的模块（都是别人写好的，本文件只调用，不改它们）
 *   S2：AD.c（ADC 采集）、IR.c（红外使能）、Serial.c（串口）、
 *       S2_IR.c（阈值/位置/AnyBlack/AllWhite）、Task1.c（到没到 B）、Delay.c
 *   S3：Motor.c（电机，还没交付 → 第 0 节 MOTOR_READY 先设 0）
 *   S1：Key.c（按键）、LED.c（灯）、Buzzer.c（蜂鸣器）、Timer.c（时基）
 *
 * 四、操作流程（只有一个按键 PB9，灯按考核要求设计）
 *   上电           → LED 慢闪（表示已上电、可以按键，这不算"运行"）
 *   短按 PB9       → 启动：一声长音 + LED 亮 300ms 后熄灭
 *   行驶中         → LED 灭（考核要求：正常行驶时不需要灯）
 *   到 B 停车      → 三短音 + LED 常亮保持（明显的"停车声光提示"）
 *   13.5 秒未到 B  → 停车 + 五声急促 + LED 快闪（超时报警）
 *   运行中长按 PB9 → 立即停车（急停，五声急促 + LED 快闪）
 *   完成/异常后 短按 PB9 → 复位回待启动
 *
 * 五、串口调试（115200 8N1，发送时勾"加回车换行"）
 *   RUN     启动（等同短按 PB9）
 *   STOP    停车（急停）
 *   RESET   复位（等同完成后短按 PB9）
 *   R       RESET 的简写
 *   STATUS  打印当前状态 / Task1 状态 / 8 路位图 / 位置 / PWM
 *
 * 六、三个节拍分别干什么（都由 TIM2 的 1ms 中断驱动）
 *   每 1ms  ：按键扫描、LED 推进、蜂鸣器节奏推进、按键事件分发
 *   每 10ms ：S2_IR_Update（更新感知）+ Task1_Update（判定到 B / 超时）+ 状态机
 *   每 500ms：串口打印一行进度日志（t=已用时 st=Task1状态 mask=位图 pos=位置）
 *
 * 七、宏定义速查（字母太多记不住时看这里）
 *   MOTOR_READY    0/1       0=电机代码未接入（只记录不驱动）；1=已接入
 *   TASK1_PWM      500       直行速度（0~1000，1000=满速），S3 接入后要标定
 *   BOARD_HAS_IR_EN 0/1      0=C8T6 板（没有红外使能脚）；1=VET6 板
 *   CTRL_MS        10        控制周期（毫秒），10ms = 100Hz
 *   CLI_LINE_MAX   16        串口指令最大长度（字符数）
 *   SM_IDLE        0         状态机：待启动
 *   SM_RUN         1         状态机：运行中
 *   SM_FINISH      2         状态机：已完成（停在 B）
 *   SM_TIMEOUT     3         状态机：超时
 *   SM_ESTOP       4         状态机：急停
 * （更完整的对照表见《S1_宏定义速查表.md》）
 *============================================================================*/

/*----------------------------------------------------------------------------
 * 包含头文件
 *   每个 .h 里声明了自己模块对外提供的函数，包含后才能调用
 *----------------------------------------------------------------------------*/
#include "stm32f10x.h"		/* 芯片外设寄存器/类型定义（标准库总头文件） */
#include "Serial.h"			/* S2：USART1 串口（Serial_Printf / 接收标志） */
#include "IR.h"				/* S2：红外发射管使能（C8T6 板不用，见第 0 节开关） */
#include "AD.h"				/* S2：ADC1 8 通道 + DMA，数据在 AD_Value[8] */
#include "S2_IR.h"			/* S2：8 路红外处理（阈值/位图/位置/AnyBlack） */
#include "Task1.h"			/* S2：项目一判定状态机（离开A / 到B / 超时） */
#include "Delay.h"			/* S2：Delay_ms / Delay_us（初始化时用） */
#include "Key.h"			/* S1：按键 PB9（短按/长按事件） */
#include "LED.h"			/* S1：LED PB12（常亮/慢闪/快闪/脉冲） */
#include "Buzzer.h"			/* S1：蜂鸣器 PA8（发声 + 节奏） */
#include "Timer.h"			/* S1：TIM2 1ms 时基（GetMs / TakeFlag1ms） */

/*==============================================================================
 * 第 0 节  编译开关（改功能先看这里）
 *============================================================================*/
/* 电机代码是否已接入：
 *   0 = S3 还没交 Motor.c，本程序只记录"想给多少 PWM"，车不动（安全，可先调逻辑）
 *   1 = S3 已实现 Motor_Init / Motor_Straight / Motor_Stop，真正驱动电机
 * 怎么用：S3 交付后在 Hardware/Motor.c 写好 3 个同名函数，把这里改成 1，重新编译即可。*/
#define MOTOR_READY        0

/* 直行速度：0~1000（1000 = 满速）。
 * 项目一要求 A→B（100cm）在 15 秒内完成 → 建议实际用时 8~10 秒，
 * 所以先给 500，等 S3 接上电机后现场标定（跑一遍看用时再调）。*/
#define TASK1_PWM          500

/* 是否有"红外使能"控制脚：
 *   C8T6 过渡板：红外模块直接 3.3V 常亮供电，没有 IR_EN 脚，
 *                 而且 D8 就接在 PB1（ADC 输入）上，绝不能再把 PB1 配成输出，
 *                 否则 D8 会恒读 3.3V。
 *   VET6 正式板：D8=PC0、IR_EN=PB1，可正常控制红外通断。
 * 所以 C8T6 填 0（main 里跳过 IR_Init），VET6 填 1。*/
#define BOARD_HAS_IR_EN    0

/*==============================================================================
 * 第 1 节  参数
 *============================================================================*/
/* 控制/感知周期：10 毫秒。
 * 为什么是 10ms：S2 的 Task1 内部是按"每调用一次算 10ms"累计时间的，
 * 所以必须严格每 10ms 调一次 Task1_Update()，否则超时判断就不准。*/
#define CTRL_MS             10

/*==============================================================================
 * 第 2 节  基础工具函数
 *============================================================================*/

/**
  * 函数：S1_StrEq
  * 作用：比较两个字符串是否完全相同（自己实现的 strcmp 简化版）
  * 参数：a、b = 要比较的两个字符串（必须以 '\0' 结尾）
  * 返回：1 = 完全相同；0 = 不同（或指针为空）
  * 为什么要自己写：串口指令解析只需要"完全相等"这一种比较，
  *   SPL 工程里不强制依赖 string.h，避免引入多余库。
  */
static uint8_t S1_StrEq(const char *a, const char *b)
{
	uint8_t i = 0;						/* 字符下标，从第 0 个字符开始 */

	if ((a == 0) || (b == 0)) { return 0; }	/* 空指针保护：防止程序跑飞 */

	/* 逐个字符比较，直到某个字符串结束 */
	while ((a[i] != '\0') && (b[i] != '\0'))
	{
		if (a[i] != b[i]) { return 0; }	/* 一旦发现不同，立刻返回"不相等" */
		i++;							/* 看下一个字符 */
	}

	/* 循环结束时：如果两个字符串同时结束（都是 '\0'），才算完全相同 */
	return (uint8_t)(((a[i] == '\0') && (b[i] == '\0')) ? 1 : 0);
}

/*==============================================================================
 * 第 3 节  声光事件映射（把"整车事件"翻译成"灯 + 声音"）
 *----------------------------------------------------------------------------
 * 考核要求：
 *   项目一："停车时有明显声光提示"
 *   项目二/三："每经过 A、B、C、D 点均须声光提示"
 *   附加题："完成提示要与途经提示不同"
 *
 * 所以灯光策略是"平时灭、到点才亮"：
 *   待启动      → 慢闪（只是表示"可以按键"，不属于运行）
 *   启动瞬间    → 亮 300ms 后自动灭（"出发"提示）
 *   运行中      → 灭（考核不要求行驶中亮灯）
 *   经过标志点  → 亮 400ms 后自动灭（以后用 LED_Pulse，与蜂鸣器组成声光提示）
 *   完成/停车   → 常亮保持（与"亮一下"明显不同，直到复位）
 *   异常/急停   → 快闪（配合五声急促）
 *
 * 本节的 5 个 S1_Fb_xxx 函数就是"事件 → 声光"的对照表，
 * 以后加项目二/三/附加题，只需要在这里再添加对应的提示函数。
 *============================================================================*/

/**
  * 函数：S1_Fb_Idle
  * 作用：待启动状态的提示 —— LED 慢闪
  * 说明：慢闪 = "板子活着，可以按键启动"，这是操作提示，不是考核要求的提示。
  */
static void S1_Fb_Idle(void)
{
	Buzzer_Stop();						/* 先停掉可能还在播的提示音 */
	LED_SetPattern(LED_PAT_SLOW);		/* 慢闪：500ms 亮 / 500ms 灭 */
}

/**
  * 函数：S1_Fb_Start
  * 作用：启动瞬间的提示 —— 一声长音 + LED 亮 300ms 后自动灭
  * 说明：为什么用"脉冲"而不是常亮：出发后车在行驶，考核不要求亮灯，
  *       所以亮一下就熄，正好让"运行中 LED 灭"。
  */
static void S1_Fb_Start(void)
{
	Buzzer_Play(1, 250, 0);		/* 蜂鸣器：响 1 声，每声 250ms，无间隔（有源蜂鸣器没有音调参数） */
	LED_Pulse(300);						/* LED：点亮 300ms 后自动熄灭 */
}

/**
  * 函数：S1_Fb_Key
  * 作用：按键反馈 —— 滴一声
  * 说明：用于"长按了但当前不在运行状态"这种情况，给用户一个听觉反馈。
  */
static void S1_Fb_Key(void)
{
	Buzzer_Play(1, 30, 0);		/* 响 1 声，30ms：短促的"嘀" */
}

/**
  * 函数：S1_Fb_AtB
  * 作用：到 B 点停车的提示 —— 三短音 + LED 常亮保持
  * 说明：这是项目一考核要求的"明显声光提示"：
  *       声音三短音、灯光从"灭"变成"常亮"，听觉视觉都能明确感知。
  *       常亮一直保持到复位，便于裁判确认"车确实停在 B 点并完成了任务"。
  */
static void S1_Fb_AtB(void)
{
	Buzzer_Play(3, 120, 120);		/* 响 3 声，每声 120ms，间隔 120ms */
	LED_Set(1);							/* LED 常亮（保持，不复位不灭） */
}

/**
  * 函数：S1_Fb_Fault
  * 作用：异常/超时/急停的提示 —— 五声急促 + LED 快闪
  * 说明：与"完成"的三短音+常亮明显不同，裁判一眼/一耳就能区分成功与失败。
  */
static void S1_Fb_Fault(void)
{
	Buzzer_Play(5, 70, 70);		/* 响 5 声，每声 70ms，间隔 70ms（急促） */
	LED_SetPattern(LED_PAT_FAST);		/* 快闪：100ms 亮 / 100ms 灭 */
}

/* 以后做项目二/三时，加下面这个函数用于"每经过 A/B/C/D 点的声光提示"：
 * static void S1_Fb_Pass(void)
 * {
 *     Buzzer_Play(1, 80, 0);         // 短促一声
 *     LED_Pulse(400);                // 亮 400ms 后自动灭
 * }
 * 现在不加，是因为还没人调用它，编译会报"函数未使用"的警告。
 */

/*==============================================================================
 * 第 4 节  电机接口（S3 的活，这里只是"夹层"）
 *----------------------------------------------------------------------------
 * 为什么要在 main.c 里包一层 S1_Motor_xxx：
 *   状态机、声光、按键都不应该直接认识电机模块。把它们统一成 3 个薄函数后，
 *   将来 S3 的接口变了（函数名/单位/方向不同），只改这 3 个函数体即可，
 *   状态机逻辑一行都不用动。
 *
 * S3 需要实现的 3 个函数（写在 Hardware/Motor.c 里，名字必须一致）：
 *   void Motor_Init(void);               初始化 TB6612 + TIM4 PWM + 方向脚 + STBY
 *   void Motor_Straight(int16_t pwm);    两轮同速前进，pwm 0~1000
 *   void Motor_Stop(void);               停车
 *
 * C8T6 板上的电机引脚（H 的图，S3 用）：
 *   PWMA = PB6 (TIM4_CH1)   PWMB = PB7 (TIM4_CH2)
 *   AIN1 = PB13  AIN2 = PB14  BIN1 = PB15  BIN2 = PB0   STBY = PB8
 * 注意：C8T6 没有 TIM8，不要照 VET6 的 PC6/PC7 写。
 *============================================================================*/
static int16_t g_pwm_cmd = 0;		/* 当前想给电机的 PWM 值（0~1000），只用于调试显示 */
static uint8_t g_motor_cmd = 0;		/* 1=正在给直行指令，0=已停（只用于调试显示） */

#if (MOTOR_READY != 0)
/* 只有 S3 交付后（MOTOR_READY=1）才声明这 3 个外部函数，否则链接会找不到符号 */
extern void Motor_Init(void);				/* 电机初始化 */
extern void Motor_Straight(int16_t pwm);	/* 两轮同速前进 */
extern void Motor_Stop(void);				/* 停车 */
#endif

/**
  * 函数：S1_Motor_Init
  * 作用：电机初始化（S3 没接入时什么都不做，只清变量）
  */
static void S1_Motor_Init(void)
{
#if (MOTOR_READY != 0)
	Motor_Init();			/* 调用 S3 的初始化 */
#endif
	g_pwm_cmd = 0;			/* 记录：当前 PWM = 0 */
	g_motor_cmd = 0;		/* 记录：当前没有直行指令 */
}

/**
  * 函数：S1_Motor_Straight
  * 作用：给"两轮同速前进"的指令
  * 参数：pwm = 0~1000（超出范围自动限幅，防止 S3 那边越界）
  */
static void S1_Motor_Straight(int16_t pwm)
{
	if (pwm < 0)    { pwm = 0; }		/* 下限保护：速度不能为负 */
	if (pwm > 1000) { pwm = 1000; }		/* 上限保护：速度最多 1000 */

	g_pwm_cmd = pwm;			/* 记录下来（STATUS 指令和进度日志会显示） */
	g_motor_cmd = 1;			/* 标记"正在给直行指令" */

#if (MOTOR_READY != 0)
	Motor_Straight(g_pwm_cmd);	/* 真正调用 S3 的电机函数（现在还没接入） */
#endif
}

/**
  * 函数：S1_Motor_Stop
  * 作用：让车停下（清指令，并调用 S3 的停车函数）
  */
static void S1_Motor_Stop(void)
{
	g_pwm_cmd = 0;			/* PWM 归零 */
	g_motor_cmd = 0;		/* 清除直行标记 */

#if (MOTOR_READY != 0)
	Motor_Stop();			/* 真正调用 S3 的停车函数 */
#endif
}

/*==============================================================================
 * 第 5 节  串口调试指令（按键没接好时的后门）
 *----------------------------------------------------------------------------
 * 作用：不依赖按键也能控制小车，方便桌面调试、也方便现场演示。
 * 接收方式：S2 的 Serial.c 每收到一个字节就置标志，这里轮询把字节攒成一行，
 *           收到回车/换行后解析执行。
 * 支持的指令（不区分大小写）：
 *   RUN / STOP / RESET / R / STATUS
 *============================================================================*/
#define CLI_LINE_MAX   16		/* 一行指令最多 16 个字符（含结尾 '\0'），超了自动丢弃重来 */

static char    g_cli_buf[CLI_LINE_MAX];	/* 一行指令的缓冲区（存用户输入） */
static uint8_t g_cli_len = 0;			/* 当前已经收到多少个字符 */

/* 下面 4 个函数在"第 6 节 状态机"里实现，这里先声明，
 * 否则编译器不认识（C 语言要求"先声明后使用"）。*/
static void S1_StartRequest(void);		/* 请求启动 */
static void S1_StopRequest(void);		/* 请求停车（急停） */
static void S1_ResetRequest(void);		/* 请求复位 */
static void S1_PrintStatus(void);		/* 打印当前状态 */

/**
  * 函数：S1_Cli_Reset
  * 作用：清空串口指令缓冲区（启动/复位时防止半截指令残留）
  */
static void S1_Cli_Reset(void)
{
	g_cli_len = 0;		/* 长度归零，缓冲区内容就不重要了（下次写入会覆盖） */
}

/**
  * 函数：S1_Upper
  * 作用：把一个小写字母转成大写（让指令不区分大小写：run / RUN 都行）
  * 返回：大写字符；非小写字母原样返回
  */
static char S1_Upper(char c)
{
	/* 小写字母 'a'~'z' 的 ASCII 码比大写 'A'~'Z' 大 32，所以 -32 */
	if ((c >= 'a') && (c <= 'z')) { return (char)(c - 32); }
	return c;
}

/**
  * 函数：S1_Cli_Exec
  * 作用：解析并执行一行指令
  * 参数：line = 收到的原始指令字符串
  * 处理流程：去首部空格 → 全部转大写 → 去尾部空格 → 逐个比较并执行
  */
static void S1_Cli_Exec(char *line)
{
	uint8_t i = 0;				/* 输入字符串的下标 */
	uint8_t j = 0;				/* 处理后的字符串下标 */
	char buf[CLI_LINE_MAX];		/* 临时存放处理后的指令 */

	/* ① 跳过开头的空格和制表符（防止手滑多打空格识别失败） */
	while ((line[i] == ' ') || (line[i] == '\t')) { i++; }

	/* ② 把剩下字符复制到 buf，同时统一转大写 */
	while ((line[i] != '\0') && (j < (CLI_LINE_MAX - 1)))
	{
		buf[j] = S1_Upper(line[i]);
		j++;
		i++;
	}
	buf[j] = '\0';				/* 手动补字符串结束符 */

	/* ③ 去掉结尾的空格（在 buf 里往前删） */
	while ((j > 0) && (buf[j - 1] == ' '))
	{
		j--;
		buf[j] = '\0';
	}

	if (j == 0) { return; }		/* 空行：什么都不做 */

	/* ④ 逐个匹配指令，匹配成功就执行并返回 */
	if (S1_StrEq(buf, "RUN"))    { S1_StartRequest(); return; }		/* 启动 */
	if (S1_StrEq(buf, "STOP"))   { S1_StopRequest();  return; }		/* 停车 */
	if (S1_StrEq(buf, "RESET") || S1_StrEq(buf, "R")) { S1_ResetRequest(); return; }	/* 复位 */
	if (S1_StrEq(buf, "STATUS")) { S1_PrintStatus();  return; }		/* 查状态 */

	Serial_Printf("ERR %s\r\n", buf);	/* 不认识的指令：回一个 ERR，方便排查 */
}

/**
  * 函数：S1_Cli_Poll
  * 作用：轮询串口，把收到的字节攒成一行并执行
  * 调用：主循环每一轮都调用（不依赖 1ms 节拍，保证 115200 不丢字节）
  * 说明：Serial_GetRxFlag() 在有新字节时返回 1，Serial_GetRxData() 取走该字节。
  */
static void S1_Cli_Poll(void)
{
	while (Serial_GetRxFlag() != 0)		/* 只要还有未处理的字节就继续取 */
	{
		uint8_t ch = Serial_GetRxData();	/* 取出一个字符 */

		if ((ch == '\r') || (ch == '\n'))	/* 收到回车或换行 = 一行结束 */
		{
			if (g_cli_len > 0)				/* 空行直接忽略 */
			{
				g_cli_buf[g_cli_len] = '\0';	/* 补字符串结束符 */
				S1_Cli_Exec(g_cli_buf);			/* 解析并执行 */
				g_cli_len = 0;					/* 清空，准备接收下一行 */
			}
		}
		else if (ch >= 0x20)				/* 只收可见字符（过滤控制字符） */
		{
			if (g_cli_len < (CLI_LINE_MAX - 1))	/* 缓冲区没满就存起来 */
			{
				g_cli_buf[g_cli_len] = (char)ch;
				g_cli_len++;
			}
			else							/* 超长：丢弃整行，避免溢出 */
			{
				g_cli_len = 0;
			}
		}
	}
}

/*==============================================================================
 * 第 6 节  整车状态机（S1 的核心）
 *----------------------------------------------------------------------------
 *   IDLE（待启动） ──短按 PB9 / 串口 RUN──> RUN（运行中）
 *   RUN ──Task1 判定"到 B"──> FINISH（完成）
 *   RUN ──Task1 判定"超时"──> TIMEOUT（超时）
 *   RUN ──长按 PB9 / 串口 STOP──> ESTOP（急停）
 *   FINISH / TIMEOUT / ESTOP ──短按 PB9 / 串口 RESET──> IDLE
 *
 * 每个状态该干什么：
 *   IDLE    ：电机停、Task1 复位、LED 慢闪，等按键
 *   RUN     ：每 10ms 更新感知 + 推进 Task1，保持直行
 *   FINISH  ：电机停、三短音 + LED 常亮，等待复位
 *   TIMEOUT ：电机停、五声急促 + LED 快闪，等待复位
 *   ESTOP   ：电机停、五声急促 + LED 快闪，等待复位
 *
 * 状态值定义（数字只是编号，写代码时用名字更直观）：
 *============================================================================*/
#define SM_IDLE      0		/* State Machine = IDLE：待启动 */
#define SM_RUN       1		/* RUN：运行中 */
#define SM_FINISH    2		/* FINISH：已完成（停在 B 点） */
#define SM_TIMEOUT   3		/* TIMEOUT：超时未到 B */
#define SM_ESTOP     4		/* ESTOP：急停 */

static uint8_t  g_sm = SM_IDLE;		/* 当前整车状态，初值 = 待启动 */
static uint32_t g_run_tick = 0;		/* 本次启动的时刻（Timer_GetMs 值），用来算已用时 */

/**
  * 函数：S1_StateName
  * 作用：把状态编号翻译成字符串，供串口打印使用
  * 返回：状态名（如 "IDLE" / "RUN"）
  */
static const char *S1_StateName(uint8_t st)
{
	switch (st)						/* 根据状态编号分支 */
	{
		case SM_IDLE:    return "IDLE";		/* 0 → 待启动 */
		case SM_RUN:     return "RUN";		/* 1 → 运行中 */
		case SM_FINISH:  return "FINISH";	/* 2 → 完成 */
		case SM_TIMEOUT: return "TIMEOUT";	/* 3 → 超时 */
		case SM_ESTOP:   return "ESTOP";	/* 4 → 急停 */
		default:         return "?";		/* 异常值（正常不会出现） */
	}
}

/**
  * 函数：S1_EnterIdle
  * 作用：进入"待启动"状态，做统一的收尾动作
  * 参数：why = 进入原因（打印到串口，方便看是复位还是刚上电）
  */
static void S1_EnterIdle(const char *why)
{
	g_sm = SM_IDLE;			/* ① 状态切到待启动 */
	S1_Motor_Stop();		/* ② 确保电机停转 */
	Task1_Init();			/* ③ 把 S2 的判定状态机复位（内部状态、计时清零） */
	S1_Fb_Idle();			/* ④ 声光提示：LED 慢闪 */

	Serial_Printf("\r\n== IDLE == %s\r\n", why);	/* ⑤ 打印进入待启动的原因 */
	Serial_Printf("短按PB9启动 串口RUN也可\r\n");	/* ⑥ 提示操作方式 */
}

/**
  * 函数：S1_StartRequest
  * 作用：请求启动（按键短按 / 串口 RUN / 复位后再次启动都会走到这里）
  * 说明：做了防重入处理：
  *       已在运行 → 忽略；
  *       在完成/超时/急停 → 先复位再启动（避免状态卡死）。
  */
static void S1_StartRequest(void)
{
	if (g_sm == SM_RUN)					/* 已经在运行 */
	{
		Serial_Printf("已在运行\r\n");	/* 提示一下，不做任何动作 */
		return;
	}
	if (g_sm != SM_IDLE)				/* 处于完成/超时/急停状态 */
	{
		S1_ResetRequest();				/* 先复位回待启动，再由用户按一次启动 */
		return;
	}

	/* 以下是从 IDLE 真正启动的流程 */
	Key_Flush();						/* 丢掉启动瞬间残留的按键事件，防误触发 */
	S1_Cli_Reset();						/* 清空串口半截指令 */
	g_run_tick = Timer_GetMs();			/* 记录启动时刻（用于计算已用时） */

	Task1_Start();						/* 让 S2 的判定进入"等离开 A 点"阶段 */
	S1_Motor_Straight(TASK1_PWM);		/* 给电机直行指令（MOTOR_READY=0 时只记录） */
	S1_Fb_Start();						/* 声光提示：一声长音 + LED 亮 300ms */
	g_sm = SM_RUN;						/* 状态切到运行中 */

	Serial_Printf("\r\n== START == A->B\r\n");	/* 打印启动信息 */
	Serial_Printf("电机:%s PWM:%d\r\n",
				  (MOTOR_READY != 0) ? "S3已接" : "未接只打印",
				  (int)TASK1_PWM);				/* 提示电机是否已接入 */
}

/**
  * 函数：S1_StopRequest
  * 作用：请求停车（长按 PB9 / 串口 STOP）
  * 说明：只有运行中才允许急停，其它状态忽略。
  */
static void S1_StopRequest(void)
{
	if (g_sm != SM_RUN)					/* 不在运行中 */
	{
		Serial_Printf("当前不是运行态\r\n");
		return;
	}

	S1_Motor_Stop();		/* ① 立即停车（安全第一） */
	S1_Fb_Fault();			/* ② 声光提示：五声急促 + LED 快闪 */
	g_sm = SM_ESTOP;		/* ③ 状态切到急停 */
	Serial_Printf("\r\n== ESTOP == 按PB9复位\r\n");
}

/**
  * 函数：S1_ResetRequest
  * 作用：请求复位（完成后短按 PB9 / 串口 RESET）
  * 说明：任何状态都可以复位，最终都回到"待启动"。
  */
static void S1_ResetRequest(void)
{
	S1_Motor_Stop();		/* ① 确保电机停 */
	Buzzer_Stop();			/* ② 立即停掉正在播的提示音 */
	Key_Flush();			/* ③ 清掉残留按键事件 */
	S1_Cli_Reset();			/* ④ 清掉串口半截指令 */
	S1_EnterIdle("人工复位");	/* ⑤ 进入待启动（内部会复位 Task1、打印提示） */
}

/**
  * 函数：S1_PrintStatus
  * 作用：串口打印当前状态（STATUS 指令调用），用于现场排查
  */
static void S1_PrintStatus(void)
{
	/* 取 S2 感知模块的当前数据（只读，不改） */
	const S2_IR_State *ir = S2_IR_Get();

	Serial_Printf("状态:%s\r\n", S1_StateName(g_sm));			/* 整车状态 */

	/* Task1：状态编号 + 已用时（Task1_GetElapsedMs 由 S2 提供） */
	Serial_Printf("Task1:%u 用时:%lu ms\r\n",
				  (unsigned)Task1_GetState(),
				  (unsigned long)Task1_GetElapsedMs());

	/* 8 路位图 mask：哪几路看到黑线（bit0=D1 … bit7=D8）；
	 * pos：加权算出的横向位置（毫米，左负右正）；line：1=看到线 */
	Serial_Printf("mask:0x%02X pos:%d line:%u\r\n",
				  (unsigned)ir->active_mask,
				  (int)ir->position_mm,
				  (unsigned)ir->line_found);

	/* 电机：当前想给的 PWM + 是否在给直行指令 */
	Serial_Printf("PWM:%d cmd:%u\r\n", (int)g_pwm_cmd, (unsigned)g_motor_cmd);
}

/**
  * 函数：S1_RunTick10ms
  * 作用：运行状态下每 10ms 执行一次 —— 推进 S2 的 Task1，并按结果决定下一步
  * 调用：只在 SM_RUN 状态下由主循环每 10ms 调用一次
  * 说明：这是"项目一"的核心逻辑，只有 40 行；以后项目二/三会是另一个类似函数。
  */
static void S1_RunTick10ms(void)
{
	Task1_State st;			/* 用来存 Task1 当前的状态 */

	Task1_Update();			/* ① 让 S2 的判定状态机走一步（内部累计 10ms 并判到B/超时） */
	st = Task1_GetState();	/* ② 取出它当前处在哪个阶段 */

	/* 阶段 1、2：还在"等离开 A"或"等 B"，说明还没到 → 保持直行 */
	if ((st == TASK1_WAIT_A_CLEAR) || (st == TASK1_RUN_TO_B))
	{
		S1_Motor_Straight(TASK1_PWM);	/* 继续直行 */
		return;							/* 本次处理结束 */
	}

	/* 阶段 3：AT_B —— 已经到 B 点 */
	if (st == TASK1_AT_B)
	{
		S1_Motor_Stop();								/* ① 立即停车 */
		S1_Fb_AtB();									/* ② 声光提示：三短音 + LED 常亮 */
		g_sm = SM_FINISH;								/* ③ 状态切到"完成" */
		Serial_Printf("\r\n== FINISH == 到B停车\r\n");
		Serial_Printf("用时 %lu ms (限 15000)\r\n",
					  (unsigned long)Task1_GetElapsedMs());	/* ④ 打印用时，便于核对是否 ≤15s */
		Serial_Printf("按PB9复位\r\n");
		return;
	}

	/* 阶段 4：TIMEOUT —— 13.5 秒内没找到 B */
	if (st == TASK1_TIMEOUT)
	{
		S1_Motor_Stop();								/* ① 停车 */
		S1_Fb_Fault();									/* ② 声光提示：五声急促 + LED 快闪 */
		g_sm = SM_TIMEOUT;								/* ③ 状态切到"超时" */
		Serial_Printf("\r\n== TIMEOUT == 13.5s没到B\r\n");
		Serial_Printf("按PB9复位\r\n");
		return;
	}
}

/**
  * 函数：S1_Key_Dispatch
  * 作用：按键事件分发（每 1ms 在主循环里调用一次）
  * 说明：C8T6 板只有一个按键 PB9：
  *       短按（松开时产生）    → 启动 / 复位
  *       长按满 1 秒（按住时产生）→ 运行中急停
  */
static void S1_Key_Dispatch(void)
{
	KeyEvent_t e = Key_GetEvent();		/* 取走一个按键事件（没有则返回 NONE） */

	if (e == KEY_EVT_START)				/* 短按事件 */
	{
		if (g_sm == SM_IDLE) { S1_StartRequest(); return; }	/* 待启动 → 启动 */
		if (g_sm == SM_RUN)  { return; }					/* 运行中 → 忽略（防误触） */
		S1_ResetRequest();									/* 完成/异常 → 复位 */
		return;
	}

	if (e == KEY_EVT_STOP)				/* 长按事件（满 1 秒时触发） */
	{
		if (g_sm == SM_RUN) { S1_StopRequest(); }			/* 运行中 → 急停 */
		else                { S1_Fb_Key(); }				/* 其它状态 → 只"滴"一声 */
	}
}

/*==============================================================================
 * 第 7 节  开机信息 + 进度日志
 *============================================================================*/

/**
  * 函数：S1_Banner
  * 作用：上电时打印版本/功能/按键说明（现场调试时一眼确认程序跑的是哪一版）
  */
static void S1_Banner(void)
{
	Serial_Printf("\r\n========================\r\n");
	Serial_Printf("S1 主控 V1.0 项目一\r\n");
	Serial_Printf("A->B 直行定点停车\r\n");
	Serial_Printf("限时15s 软件13.5s\r\n");
	Serial_Printf("短按PB9启动 长按PB9急停\r\n");
	Serial_Printf("串口 RUN STOP RESET\r\n");
	Serial_Printf("       STATUS\r\n");
	Serial_Printf("========================\r\n");
}

/**
  * 函数：S1_Log_Progress
  * 作用：运行中每 500ms 打印一行进度，方便现场判断小车跑到了哪一步
  * 输出格式：t已用时(ms) stTask1状态 mask位图 pos位置 pwm当前PWM
  * 示例    ：t1500 st2 mask00 pos0 pwm500
  */
static void S1_Log_Progress(void)
{
	const S2_IR_State *ir = S2_IR_Get();	/* 取感知数据 */

	if (g_sm != SM_RUN) { return; }			/* 只在运行中打印 */

	Serial_Printf("t%lu st%u mask%02X pos%d pwm%d\r\n",
				  (unsigned long)(Timer_GetMs() - g_run_tick),	/* 已运行毫秒数 */
				  (unsigned)Task1_GetState(),					/* Task1 状态（1=等A,2=等B,3=到B,4=超时） */
				  (unsigned)ir->active_mask,					/* 8 路位图 */
				  (int)ir->position_mm,							/* 横向位置 */
				  (int)g_pwm_cmd);								/* 当前 PWM */
}

/*==============================================================================
 * 第 8 节  main
 *----------------------------------------------------------------------------
 * 初始化顺序的原则：
 *   ① 先初始化"别人的模块"（S2 的串口/ADC/感知/判定），因为后面可能要用它们的输出
 *   ② 再初始化"自己的驱动"（按键/灯/蜂鸣器/电机夹层）
 *   ③ 最后才开 1ms 时基（如果时基先开，中断可能在初始化没做完时就进来，造成混乱）
 *   ④ 打印开机信息 → 更新一帧感知 → 进入待启动
 *
 * 主循环的结构：
 *   高速轮询串口（不依赖节拍，保证不丢字节）
 *   每 1ms ：按键、LED、蜂鸣器、按键事件分发
 *   每 10ms：感知更新 + Task1 推进（运行中）
 *   每 500ms：进度日志
 *============================================================================*/
int main(void)
{
	uint8_t c10 = 0;		/* 10ms 分频计数器：数到 10（即 10 次 1ms）执行一次 10ms 任务 */
	uint8_t c500 = 0;		/* 500ms 分频计数器：数到 50 执行一次日志 */

	/*------------------------------------------------------------------
	 * 1. 初始化 S2 的模块（顺序参考 S2 自己的测试 main）
	 *----------------------------------------------------------------*/
	Serial_Init();			/* 串口：USART1，PA9=TX / PA10=RX，115200 */

#if (BOARD_HAS_IR_EN != 0)	/* C8T6 板没有红外使能脚 → 这段不编译 */
	IR_Init();				/* 把 PB1 拉高，给红外发射管供电（VET6 板才用） */
#endif

	AD_Init();				/* ADC1 扫描 8 个通道 + DMA 循环搬运到 AD_Value[8] */
	S2_IR_Init();			/* S2 的红外数据处理：阈值/回差/位图/位置 */
	Task1_Init();			/* S2 的项目一判定状态机复位 */
	Delay_ms(20);			/* 等 ADC 稳定几帧（上电瞬间读数会漂） */

	/*------------------------------------------------------------------
	 * 2. 初始化 S1 的驱动模块
	 *----------------------------------------------------------------*/
	Key_Init();				/* 按键 PB9：输入上拉 + 20ms 消抖 */
	LED_Init();				/* LED PB12：推挽输出，默认灭 */
	Buzzer_Init();			/* 蜂鸣器 PA8：有源，GPIO 低电平触发，默认不响 */
	S1_Motor_Init();		/* 电机夹层（S3 未接入时只清变量） */
	Timer_Init();			/* TIM2 1ms 时基：最后开，避免初始化期间中断干扰 */

	/*------------------------------------------------------------------
	 * 3. 开机提示 + 进入待启动
	 *----------------------------------------------------------------*/
	S1_Banner();			/* 串口打印开机信息 */
	S2_IR_Update();			/* 先更新一帧感知，避免首帧全 0 影响判断 */
	S1_EnterIdle("上电完成");	/* 进入待启动：LED 慢闪，等待按键 */

	/*------------------------------------------------------------------
	 * 4. 后台主循环（永不退出）
	 *----------------------------------------------------------------*/
	while (1)
	{
		/* (a) 串口高频轮询：
		 *     115200 下 1 个字节 87 微秒就到，主循环一轮只要几微秒，
		 *     所以每轮都来取一次，保证不丢字节。*/
		S1_Cli_Poll();

		/* (b) 等 1ms 节拍：
		 *     没到 1ms 就回去继续转（继续轮询串口）；
		 *     到了 1ms 才往下执行 1ms 任务。
		 *     注意：Timer_TakeFlag1ms 是"取走式"，取一次清零一次。*/
		if (Timer_TakeFlag1ms() == 0) { continue; }

		/* 以下是"每 1ms 执行一次"的任务 */
		Key_Tick1ms();			/* 按键扫描：消抖 + 短按/长按判定 */
		LED_Tick1ms();			/* LED 推进：闪烁/脉冲倒计时 */
		Buzzer_Tick1ms();		/* 蜂鸣器推进：节奏"响-停-响" */
		S1_Key_Dispatch();		/* 处理按键事件（启动/复位/急停） */

		/* (c) 10ms 任务：把 1ms 的节拍再分频 10 次 = 10ms */
		c10++;
		if (c10 >= CTRL_MS)		/* CTRL_MS = 10 */
		{
			c10 = 0;			/* 重新计数 */

			S2_IR_Update();		/* 更新 8 路感知（100Hz） */

			if (g_sm == SM_RUN)	/* 运行中：推进 Task1 并处理结果 */
			{
				S1_RunTick10ms();
			}
			else				/* 非运行状态：确保电机保持停止 */
			{
				S1_Motor_Stop();
			}
		}

		/* (d) 500ms 任务：进度日志（数 50 次 1ms） */
		c500++;
		if (c500 >= 50)
		{
			c500 = 0;
			S1_Log_Progress();	/* 运行中才打印，其余状态内部会直接返回 */
		}
	}
}
