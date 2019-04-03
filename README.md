# STM32F0-TGAM
## About
IC:STM32F030F4PX

利用STM32CubeMX生成的EWARM V8工具链的工程

|  管脚       |          复用           |  
|-------------|-------------------------|
|PA2、PA3     |用于和TGAM通信获取EEG数据|
|PB1、PA7、PA6|PWM控制RGB灯             |

RGB分别对应EEG的Low Alpha、Low Beta、Low Gamma波段，在主循环中可以修改
