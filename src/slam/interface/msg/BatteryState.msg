# BatteryState.msg - 电池状态
uint8 percentage       # SOC 0-100%
float32 voltage        # 总电压 V
float32 current        # 电流 mA (正=充电, 负=放电)
int8[2] temperature    # 电池温度 degrees C
uint8 status           # 0=normal, 1=charging, 2=low, 3=critical
uint16 cycle_count     # 充放电循环次数
