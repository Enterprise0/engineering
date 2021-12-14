## to do:
+ 复位按键  G

## 5/11
1. //障碍块、救援 按键
#define BARRIER_KEY		KEY_PRESSED_OFFSET_Z
#define SAVER_KEY		KEY_PRESSED_OFFSET_X

2. //升降、抓手 按键
#define UP_DOWN_KEY			KEY_PRESSED_OFFSET_C
#define CATCH_PITCH_KEY		KEY_PRESSED_OFFSET_Q
#define CATCH_ROTATION_KEY	KEY_PRESSED_OFFSET_E	// 抓手pitch 往前

3. 
抓手前伸    右键
抓手抓取    左键

## 5/12
- 删除 I2C2、SPI2，与电磁阀GPIO口冲突
- 删除OLED
- 删除 test_task
+ 注释 detect_task中关于OLED_com_reset的语句
+ 添加 电磁阀 GPIO 控制


## 5/16
- 有一些can2的线接收不到c620数据

+ #define SILVER_HALF_KEY	KEY_PRESSED_OFFSET_R	//银矿对位
+ #define GOLD_HALF_KEY	    KEY_PRESSED_OFFSET_F	//金矿对位
+ chassis_rc_to_control_vector() 中添加矿石自动对位
+ 障碍块、救援、升降、抓手 添加斜坡函数 控制角度