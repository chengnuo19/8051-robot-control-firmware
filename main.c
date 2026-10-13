/**
 ******************************************************************************
 * @file    main.c
 * @author  Antigravity
 * @brief   电赛控制系统总调度程序 (集成循迹、避障、串口通信、软PWM与多测试模式)
 * @note    支持一键切换测试模式，极为方便硬件调试和程序排错。
 ******************************************************************************
 */

#include "config.h"
#include "timer.h"
#include "motor.h"
#include "servo.h"
#include "oled.h"
#include "uart.h"
#include "tracking.h"
#include "ultrasonic.h"
#include "pid.h"

/* ========================================================================== */
/*                      1. 测试模式配置 (赛场保命神器)                         */
/* ========================================================================== */
#define TEST_MODE_NONE   0   // [推荐] 正常综合运行模式 (循迹 + 超声波避障 + 串口视觉)
#define TEST_MODE_MOTOR  1   // 电机模块测试模式 (前进、后退、左右旋转)
#define TEST_MODE_SERVO  2   // 舵机云台/机械臂测试模式 (在0~180度之间来回扫描)
#define TEST_MODE_TRACK  3   // 循迹红外探头测试模式 (在 OLED 上实时显示探头状态和计算出的 Error)
#define TEST_MODE_SR04   4   // 超声波单模块测距测试模式 (在 OLED 上实时显示 cm 距离)
#define TEST_MODE_UART   5   // OpenMV 串口通信测试模式 (在 OLED 上显示接收到的命令和坐标数值)

// ！！！ 修改下面的值切换不同的测试模式 ！！！
#define RUN_MODE  TEST_MODE_NONE

/* ========================================================================== */
/*                      2. 全局控制结构体与变量                                */
/* ========================================================================== */
PID_t track_pid;            // 循迹 PID 控制器结构体
s16 base_speed = 35;        // 小车直行的基础速度 (百分比 0 ~ 100)

// 简单的非阻塞软件延时计时器
u32 oled_timer = 0;       // OLED 刷新计时
u32 sr04_timer = 0;       // 超声波采样计时 (阱止每循环都阻塞等待Echo)
u16 g_distance = 200;    // 超声波距离全局缓存 (cm), 初始化为安全远距离

/* ========================================================================== */
/*                      3. 软件毫秒延时函数 (阻塞)                             */
/* ========================================================================== */
void Delay_Ms(u16 ms)
{
    u32 start = Timer_GetMs();
    while (Timer_GetMs() - start < ms);
}

/* ========================================================================== */
/*                      4. 主程序入口 main                                    */
/* ========================================================================== */
void main(void)
{
    // --- A. 系统各硬件分块模块初始化 ---
    Timer0_Init();      // 开启定时器0 (100us基准，启动软PWM计数器)
    Motor_Init();       // 电机引脚初始化
    Servo_Init();       // 舵机引脚初始化，默认中位90度
    OLED_Init();        // OLED 初始化，并清屏
    Uart_Init();        // 串口初始化
    Tracking_Init();    // 循迹引脚初始化
    Ultrasonic_Init();  // 超声波引脚初始化
    
    // --- B. 循迹 PID 算法初始化 ---
    // 【定点山数调试说明】 Kp、Ki、Kd 均为实际系数×10：
    // Kp=150 → 实际Kp=15.0: 控制对偏差的响应灵敏度。
    // Ki=0   → 实际Ki=0  : 没有编码器的循迹不加积分项。
    // Kd=40  → 实际Kd=4.0 : 微分项抑制直道摇摇摆路现象。
    // 积分上限=100, 输出限幅=30 (差速不超过基础速度30%)
    PID_Init(&track_pid, 150, 0, 40, 100, 30);
    
    // OLED 显示欢迎字样
    OLED_ShowString(1, 1, "System Ready!   ");
    OLED_ShowString(2, 1, "Mode:           ");
    
#if (RUN_MODE == TEST_MODE_NONE)
    OLED_ShowString(2, 7, "Normal Run");
#elif (RUN_MODE == TEST_MODE_MOTOR)
    OLED_ShowString(2, 7, "Test Motor");
#elif (RUN_MODE == TEST_MODE_SERVO)
    OLED_ShowString(2, 7, "Test Servo");
#elif (RUN_MODE == TEST_MODE_TRACK)
    OLED_ShowString(2, 7, "Test Track");
#elif (RUN_MODE == TEST_MODE_SR04)
    OLED_ShowString(2, 7, "Test SR04 ");
#elif (RUN_MODE == TEST_MODE_UART)
    OLED_ShowString(2, 7, "Test UART ");
#endif

    Delay_Ms(1000);     // 延时1秒，让开机画面显示一会儿
    OLED_Clear();
    
    // --- C. 主循环调度 ---
    while (1)
    {
        /* ================================================================== */
        /* 【模式 0】：综合正常运行模式 (红外循迹 + 避障 + 串口接收)            */
        /* ================================================================== */
        #if (RUN_MODE == TEST_MODE_NONE)
        {
            u16 distance = 0;
            s16 track_err = 0;
            s16 pid_output = 0;
            s16 left_w = 0;
            s16 right_w = 0;
            
            // 1. 获取传感器数据
            // 超声波每100ms采样一次，避免每次循环却阻塞等待Echo (HC-SR04最长阻塞约30ms+)
            if (Timer_GetMs() - sr04_timer >= 100)
            {
                sr04_timer = Timer_GetMs();
                g_distance = Ultrasonic_GetDistance();
            }
            track_err = Tracking_GetError();     // 红外循迹偏差 (-4 到 4)
            distance = g_distance;
            
            // 2. 安全策略：前方有障碍物 (小于 15cm) 紧急刹车
            if (distance < 15)
            {
                Motor_Brake(); // 紧急刹车
                
                // 显示警报
                OLED_ShowString(1, 1, "!! OBSTACLE !!  ");
                OLED_ShowString(2, 1, "Distance:       ");
                OLED_ShowNum(2, 11, distance, 3);
                OLED_ShowString(2, 14, "cm");
                OLED_ShowString(3, 1, "Motor: BRAKE    ");
                OLED_ShowString(4, 1, "Status: PAUSED  ");
                
                Delay_Ms(100);
                continue; // 重新测距，直到障碍物被移开
            }
            
            // 3. 视觉协议处理：判定 OpenMV 是否发来强力控制命令 (例如停车标志 0x02)
            if (uart_rx_flag == 1)
            {
                uart_rx_flag = 0; // 清除标志位
                
                if (uart_cmd == 0x02) // OpenMV 发送了定点停车指令
                {
                    Motor_Brake();
                    OLED_ShowString(1, 1, "-- OPENMV STOP --");
                    OLED_ShowString(3, 1, "Motor: BRAKE    ");
                    while(1); // 死循环在此处，完成定点停车任务，直到重新上电
                }
            }
            
            // 4. PID 循迹计算 (目标偏差 target=0 即完美居中)
            pid_output = PID_Compute(&track_pid, 0, track_err);
            
            // 5. 差速驱动电机
            // pid_output > 0 代表小车偏右，此时差速修正：左轮加速、右轮减速，引导小车向左打方向
            // pid_output < 0 代表小车偏左，此时差速修正：左轮减速、右轮加速，引导小车向右打方向
            left_w  = base_speed + pid_output;
            right_w = base_speed - pid_output;
            
            Motor_SetSpeed(left_w, right_w);
            
            // 6. OLED 定时刷新调试信息 (每150毫秒刷新一次，防止刷屏过快导致闪烁)
            if (Timer_GetMs() - oled_timer >= 150)
            {
                oled_timer = Timer_GetMs();
                
                OLED_ShowString(1, 1, "SYS: NORMAL RUN ");
                
                OLED_ShowString(2, 1, "Dist:");
                OLED_ShowNum(2, 6, distance, 3);
                OLED_ShowString(2, 9, "cm");
                
                OLED_ShowString(2, 11, "Err:");
                OLED_ShowSignedNum(2, 15, track_err, 1);
                
                OLED_ShowString(3, 1, "PWM L:");
                OLED_ShowNum(3, 7, left_w > 0 ? left_w : 0, 3);
                OLED_ShowString(3, 10, " R:");
                OLED_ShowNum(3, 13, right_w > 0 ? right_w : 0, 3);
                
                OLED_ShowString(4, 1, "OMV C:");
                OLED_ShowNum(4, 7, uart_cmd, 2);
                OLED_ShowString(4, 10, " D:");
                OLED_ShowSignedNum(4, 13, uart_data, 3);
            }
        }
        
        /* ================================================================== */
        /* 【模式 1】：电机单元测试模式                                        */
        /* ================================================================== */
        #elif (RUN_MODE == TEST_MODE_MOTOR)
        {
            OLED_ShowString(1, 1, "TEST MOTOR MODE ");
            
            // 1. 前进 1.5秒
            OLED_ShowString(2, 1, "Dir: FORWARD    ");
            Motor_SetSpeed(40, 40);
            Delay_Ms(1500);
            
            // 2. 刹车 1秒
            OLED_ShowString(2, 1, "Dir: BRAKE      ");
            Motor_Brake();
            Delay_Ms(1000);
            
            // 3. 后退 1.5秒
            OLED_ShowString(2, 1, "Dir: BACKWARD   ");
            Motor_SetSpeed(-40, -40);
            Delay_Ms(1500);
            
            // 4. 刹车 1秒
            OLED_ShowString(2, 1, "Dir: BRAKE      ");
            Motor_Brake();
            Delay_Ms(1000);
            
            // 5. 原地原地左转 1秒
            OLED_ShowString(2, 1, "Dir: TURN LEFT  ");
            Motor_SetSpeed(-35, 35);
            Delay_Ms(1000);
            
            // 6. 刹车 1秒
            OLED_ShowString(2, 1, "Dir: BRAKE      ");
            Motor_Brake();
            Delay_Ms(1000);
        }
        
        /* ================================================================== */
        /* 【模式 2】：舵机测试模式 (机械臂/云台)                             */
        /* ================================================================== */
        #elif (RUN_MODE == TEST_MODE_SERVO)
        {
            u16 angle;
            OLED_ShowString(1, 1, "TEST SERVO MODE ");
            
            // 0度 -> 180度
            for (angle = 0; angle <= 180; angle += 10)
            {
                Servo_SetAngle(1, angle);
                Servo_SetAngle(2, 180 - angle); // 互补运动
                
                OLED_ShowString(2, 1, "Servo1 Ang:     ");
                OLED_ShowNum(2, 12, angle, 3);
                OLED_ShowString(3, 1, "Servo2 Ang:     ");
                OLED_ShowNum(3, 12, 180 - angle, 3);
                
                Delay_Ms(150); // 给足舵机转动物理响应时间
            }
            
            Delay_Ms(500);
            
            // 180度 -> 0度
            for (angle = 180; angle >= 0; angle -= 10)
            {
                Servo_SetAngle(1, angle);
                Servo_SetAngle(2, 180 - angle);
                
                OLED_ShowString(2, 1, "Servo1 Ang:     ");
                OLED_ShowNum(2, 12, angle, 3);
                OLED_ShowString(3, 1, "Servo2 Ang:     ");
                OLED_ShowNum(3, 12, 180 - angle, 3);
                
                Delay_Ms(150);
            }
            
            Delay_Ms(500);
        }
        
        /* ================================================================== */
        /* 【模式 3】：循迹测试模式                                           */
        /* ================================================================== */
        #elif (RUN_MODE == TEST_MODE_TRACK)
        {
            u8 raw_val = 0;
            s16 err = 0;
            
            raw_val = Tracking_ReadRaw();
            err = Tracking_GetError();
            
            OLED_ShowString(1, 1, "TEST TRACK MODE ");
            
            // 显示引脚原始电平(二进制掩码)
            OLED_ShowString(2, 1, "Raw Snsr:       ");
            OLED_ShowNum(2, 11, (raw_val & 0x08) ? 1 : 0, 1);
            OLED_ShowNum(2, 12, (raw_val & 0x04) ? 1 : 0, 1);
            OLED_ShowNum(2, 13, (raw_val & 0x02) ? 1 : 0, 1);
            OLED_ShowNum(2, 14, (raw_val & 0x01) ? 1 : 0, 1);
            
            // 显示计算出的相对黑线误差 Error
            OLED_ShowString(3, 1, "Calc Err:       ");
            OLED_ShowSignedNum(3, 11, err, 1);
            
            // 底行中文文字提示
            if (err == 0)       OLED_ShowString(4, 1, "Status: Center  ");
            else if (err < 0)  OLED_ShowString(4, 1, "Status: Left -> ");
            else               OLED_ShowString(4, 1, "Status: <- Right");
            
            Delay_Ms(100);
        }
        
        /* ================================================================== */
        /* 【模式 4】：超声波测距避障测试                                      */
        /* ================================================================== */
        #elif (RUN_MODE == TEST_MODE_SR04)
        {
            u16 dist = 0;
            dist = Ultrasonic_GetDistance();
            
            OLED_ShowString(1, 1, "TEST SR04 MODE  ");
            
            OLED_ShowString(2, 1, "Distance:       ");
            OLED_ShowNum(2, 11, dist, 3);
            OLED_ShowString(2, 15, "cm");
            
            if (dist == 999)
            {
                OLED_ShowString(4, 1, "Sensor Timeout! ");
            }
            else if (dist < 15)
            {
                OLED_ShowString(4, 1, "Warning: Close! ");
            }
            else
            {
                OLED_ShowString(4, 1, "Status: Safe    ");
            }
            
            Delay_Ms(100);
        }
        
        /* ================================================================== */
        /* 【模式 5】：串口与 OpenMV 通信测试                                   */
        /* ================================================================== */
        #elif (RUN_MODE == TEST_MODE_UART)
        {
            OLED_ShowString(1, 1, "TEST UART MODE  ");
            OLED_ShowString(2, 1, "Rx Flag:        ");
            OLED_ShowNum(2, 10, uart_rx_flag, 1);
            
            if (uart_rx_flag == 1)
            {
                uart_rx_flag = 0; // 手动清空标志位
                
                OLED_ShowString(3, 1, "CMD :           ");
                OLED_ShowNum(3, 7, uart_cmd, 2);
                OLED_ShowString(4, 1, "DATA:           ");
                OLED_ShowSignedNum(4, 7, uart_data, 3);
                
                // 收到消息后，可以给 OpenMV 发送一个确认消息
                Uart_SendString("ACK\r\n");
            }
            
            Delay_Ms(20);
        }
        #endif
    }
}
