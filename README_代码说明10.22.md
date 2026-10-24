# S2_IR_8CH_Keil · VET6 最终工程说明

> 适用：**STM32F103VET6** 正式主控板  
> 已用 Keil 实际编译：**0 Error(s), 0 Warning(s)**  
> Code=7068，RO-data=420，RW-data=32，ZI-data=1064

---

## 1. 引脚

| 模块 | VET6 引脚 | 说明 |
|---|---|---|
| VCC | **3.3V** | 不要 5V |
| GND | GND | 共地 |
| IR | PB1 | 高电平使能 |
| D1 | PA1 | ADC1_IN1 |
| D2 | PA2 | ADC1_IN2 |
| D3 | PA3 | ADC1_IN3 |
| D4 | PA4 | ADC1_IN4 |
| D5 | PA5 | ADC1_IN5 |
| D6 | PA6 | ADC1_IN6 |
| D7 | PA7 | ADC1_IN7 |
| D8 | **PC0** | **ADC1_IN10** |

串口：

```text
PA9  -> USB-TTL RX
PA10 -> USB-TTL TX
GND  -> GND
115200，8N1
```

SWD：

```text
PA13 -> SWDIO
PA14 -> SWCLK
GND  -> GND
```

---

## 2. 文件结构

```text
Hardware/
├─ AD.c / AD.h          ADC1 扫描 8 通道 + DMA1 循环
├─ IR.c / IR.h          PB1 高电平，使能红外管
└─ Serial.c / Serial.h  USART1 + printf

System/
├─ Delay.c / Delay.h
└─ S2_IR.c / S2_IR.h    阈值、mask、line_found、position_mm

User/
└─ main.c               10ms 更新，100ms 打印
```

`mainB1.c` 已移除；不要和 `main.c` 同时加进工程。

---

## 3. S2_IR 接口

```c
typedef struct {
    uint16_t raw[8];
    uint8_t  active_mask;   // bit0=D1 ... bit7=D8
    uint8_t  line_found;
    int16_t  position_mm;   // 左负右正
    uint8_t  valid;
    uint8_t  confidence;
} S2_IR_State;

void S2_IR_Init(void);
void S2_IR_Update(void);          // 每10ms调用
const S2_IR_State *S2_IR_Get(void);
```

串口输出：

```text
t_ms,raw0,...,raw7,found,pos,mask,conf
```

---

## 4. 阈值注意事项

`S2_IR.c` 里的阈值目前是 **C8T6 测试板 + 11mm 高度** 标定的：

```text
3980, 3952, 3938, 3949, 3943, 3937, 3970, 3977
```

换到 VET6 正式板、装车支架后：

- 至少复测 D1/D4/D8 的白/黑；
- 如果差值/基线变化，重新算阈值；
- 不要直接假设 C8T6 的阈值在 VET6 上完全不变。

---

## 5. 赛道和 B 点（必须按 PDF）

考核图：

- **A–B 是白色直道，100cm，没有黑线**；
- **C–D 也是白色直道，60cm**；
- 只有左右两条半圆弧是黑色，半径 40cm，线宽 1.8cm；
- **B 点是右侧半圆弧的起点/切点，不是分叉，也没有 B 点辅线**；
- 场地除两个半圆弧外，不得添加其他标记。

控制逻辑：

```text
A-B：开环直行，mask=0，line_found=0
B 点：A-B 全白后，右弧从右侧进入 -> mask 非0
      -> 连续 N 帧确认 -> 切弧线循迹
B-C：用 S2_IR.position_mm 循迹
C-D：开环直行
D-A：左弧循迹
```

**不要用 `line_found` 在 A-B 上等 B 点。**  
`line_found` 只用于弧线循迹；B 点由 S1 用 `active_mask` 的“全白 -> 右弧出现”序列判断。

---

## 6. 构建

```text
Device: STM32F103VE
Startup: startup_stm32f10x_hd.s
Define: STM32F10X_HD,USE_STDPERIPH_DRIVER
Use MicroLIB: 1
Flash: 512KB
```

输出：

```text
Objects/S2_IR_8CH.hex
```
