#include "heartRate.h"

int16_t IR_AC_Max = 500;  // 初始设置为 500，提高敏感度
int16_t IR_AC_Min = -500; // 初始设置为 -500
int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous = 0;
int16_t IR_Average_Estimated = 0;
int32_t ir_avg_reg = 0;
uint8_t offset = 0;

static const uint16_t FIRCoeffs[6] = {321, 927, 1858, 2916, 3768, 4096}; // 优化后的系数

// 检测心跳
bool checkForBeat(int32_t sample)
{
  bool beatDetected = false;
  IR_AC_Signal_Previous = IR_AC_Signal_Current;

  IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

  if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0))
  {
    IR_AC_Max = IR_AC_Signal_Current > IR_AC_Max ? IR_AC_Signal_Current : IR_AC_Max;
    IR_AC_Min = IR_AC_Signal_Current < IR_AC_Min ? IR_AC_Signal_Current : IR_AC_Min;

    if ((IR_AC_Max - IR_AC_Min) > 500)
    {
      beatDetected = true;
      IR_AC_Max = 500;
      IR_AC_Min = -500;
    }
  }

  return beatDetected;
}

// 优化后的 DC 估算器
inline int16_t averageDCEstimator(int32_t *p, uint16_t x)
{
  *p += (((x << 4) - *p) >> 4); // 简化移位
  return (*p >> 4);
}

// 优化后的低通 FIR 滤波器
int16_t lowPassFIRFilter(int16_t din)
{
  static int16_t cbuf[6]; // 使用 6 个系数
  cbuf[offset] = din;

  int32_t z = mul16(FIRCoeffs[5], cbuf[(offset - 5) & 0x07]);
  for (uint8_t i = 0; i < 5; i++)
  {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x07] + cbuf[(offset - 10 + i) & 0x07]);
  }

  offset = (offset + 1) & 0x07; // 环绕优化
  return (z >> 14);             // 降低移位精度
}

// 优化后的整数乘法器
inline int32_t mul16(int16_t x, int16_t y)
{
  return ((int32_t)x * y);
}