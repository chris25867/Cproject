#include "main.h"
#include "task.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "tim.h"
#include "GM6020.hpp"
#include <cmath>
#include "Pid.hpp"
#include "iwdg.h"

uint32_t tick = 0;
uint8_t can1_tx_data[8] = {0};
uint8_t can1_rx_data[8] = {0};
uint8_t can2_tx_data[8] = {0};
uint8_t can2_rx_data[8] = {0};
float target_vel = 0;

static CAN_RxHeaderTypeDef rx_header1;

GM6020 Motor1;
Pid Vel_Pid;

Pid Ang_Pid;
float target_angle = 0.0f;
float last_target_angle = 0.0f;
uint8_t control_mode = 0;//切换速度环（0）或角度环（1）

CAN_HandleTypeDef * const motor_can = &hcan1;

static volatile uint32_t motor_rx_tick = 0;
static volatile bool motor_received = false;
static bool motor_fault = false;//掉线不恢复

static void DecodeMotor(CAN_RxHeaderTypeDef *header,
                        uint8_t *data);

void MainInit(void)
{
    tick = 0;
    target_vel = 0.0f;
    target_angle = 0.0f;

    motor_received = false;
    motor_fault = false;
    motor_rx_tick = HAL_GetTick();//初始化

    Motor1.Init(1);

    Vel_Pid.Init(60.0f,300.0f, 0.0f, 0.001f,
                 16384.0f, -16384.0f);//速度pid
    Vel_Pid.SetFeedforward(16.5f);
    Vel_Pid.SetFriction(250.0f, 2.5f);  // 动摩擦前馈: 250, ±2.5 rad/s 内线性过渡
    Vel_Pid.SetStiction(450.0f, 1.2f);  // 静摩擦附加: |v|<1.2 rad/s 时叠加, 到 1.2 线性淡出
    Vel_Pid.SetDerivativeFilter(8.0f);
    Vel_Pid.SetIntegralLimit(8000.0f);

    Ang_Pid.Init(8.0f, 0.0f, 0.0f, 0.001f,
                 10.0f, -10.0f);//角度pid
    Ang_Pid.SetIntegralLimit(10.0f);

    CanFilter_Init(&hcan1);

    Can_EnableAutoRecovery(&hcan1);

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(
            &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

void MainTask(void)
{
    uint32_t primask = __get_PRIMASK();//获取中断状态
    __disable_irq();//暂时禁止中断（防止MainTask被CAN打断）

    float feedback_vel = Motor1.vel();
    float feedback_angle = Motor1.angle();
    uint32_t feedback_tick = motor_rx_tick;
    bool feedback_received = motor_received;

    __set_PRIMASK(primask);//恢复中断

    if (static_cast<uint32_t>(
            HAL_GetTick() - feedback_tick) > 100U) {
        motor_fault = true;
    }//掉线检测（100帧）

    if (!feedback_received || motor_fault) {
        Motor1.setInput(0.0f);
        Vel_Pid.Reset();
        Ang_Pid.Reset();//故障处理：目标电流降为0
    } else {
        if (control_mode == 0U) {
            //target_vel = 15.0f;
            target_vel = sin((float)tick / 1000.0f) * 15;//速度环曲线
            Vel_Pid.Control(target_vel, feedback_vel);
            Motor1.setInput(Vel_Pid.Result());

        } else if (control_mode == 1U) {
            target_angle = sin((float)tick / 1000.0f) * 5.0f;//角度环曲线
            
            float target_vel_ff = (target_angle - last_target_angle)/ 0.001f;
            last_target_angle = target_angle;

            if (target_vel_ff > 8.0f){
                target_vel_ff = 8.0f;
            } else if (target_vel_ff < -8.0){
                target_vel_ff = -8.0f;
            }

            Ang_Pid.Control(target_angle, feedback_angle);
            target_vel = Ang_Pid.Result() + target_vel_ff;

            Vel_Pid.Control(target_vel, feedback_vel);
            Motor1.setInput(Vel_Pid.Result());

        } else {
            Motor1.setInput(0.0f);//异常模式停机
        }

        tick++;
    }

    if (motor_can == &hcan1) {
        if (Motor1.encode(can1_tx_data)) {
            CAN_Send_Msg(&hcan1, can1_tx_data,
                         Motor1.txId(), 8);//有本电机数据就发送到总线
        }
    }

    HAL_IWDG_Refresh(&hiwdg);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调
{
    if (htim == &htim6) {
        MainTask();
    }
}

static void DecodeMotor(CAN_RxHeaderTypeDef *header,
                        uint8_t *data)//can接收
{
    if (header->IDE != CAN_ID_STD ||
        header->RTR != CAN_RTR_DATA ||
        header->DLC != 8U) {
        return;
    }//帧判定

    if (header->StdId == Motor1.rxId()) {
        if (Motor1.decode(data)) {
            motor_rx_tick = HAL_GetTick();//tick更新
            motor_received = true;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) {//传入句柄判定
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {//取出所有帧
        if (HAL_CAN_GetRxMessage(
                hcan, CAN_RX_FIFO0,
                &rx_header1, can1_rx_data) != HAL_OK) {
            break;
        }

        if (motor_can == &hcan1) {
            DecodeMotor(&rx_header1, can1_rx_data);//写入帧头和数据
        }
    }
}
