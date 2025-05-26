/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

#define PD7_Pin GPIO_PIN_7
#define PD6_Pin GPIO_PIN_6
#define PD5_Pin GPIO_PIN_5
#define PD4_Pin GPIO_PIN_4

#define PE2_Pin GPIO_PIN_2
#define PE3_Pin GPIO_PIN_3
#define PE4_Pin GPIO_PIN_4
#define PE6_Pin GPIO_PIN_6

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDGains;

typedef struct {
    PIDGains x_pid;
    PIDGains y_pid;
    PIDGains phi_pid;
    PIDGains u_pid[4];  // Per-wheel PID
} PIDConfig;

typedef struct
{
	double x_desired;
	double y_desired;
	double phi_end;
	double d;
	double r;
//	uint8_t state;
	float u1_desired;
	float u2_desired;
	float u3_desired;
	float u4_desired;

} InputData;

typedef struct {
    double yaw;
    double roll;
    double pitch;
} IMUData;

typedef struct {
    double err_x;
    double err_y;
    double err_phi;
    double u_errs[4];
} Errors;

typedef struct {
    uint32_t current;
    uint32_t previous;
    uint32_t delta;
    uint32_t print_prev;
} TimeState;

typedef struct {
	uint16_t cnt_vals[4];
	uint16_t prevcnt_vals[4];
	float angleVals[4];
	double omegaVals[4];
} EncoderData;

typedef struct {
	double x_pos;
	double phi;
	double y_pos;
	double q_dot[3]; // <- Inertial x_dot, y_dot, z_dot
} OdomData;

typedef struct {
  double x_dot;   // desired x velocity
  double y_dot;   // desired y velocity
  double phi_dot;   // desired rotational velocity
  double PWM_vals[4]; // Motor PIDs outputs
  uint16_t dutyCycles[4]; // Motor control limited duty cycles
  uint8_t M_dirs[4]; // Motor directions,  0 = forward, 1 = back <- arbitrary
  double u[4]; // ik computed required wheel speeds (wheel velocity control input)
} CtrlOutData;


typedef struct {
    IMUData imu;
    EncoderData encoders;
    Errors error;
    TimeState time;
    OdomData odom;
    CtrlOutData ctrl;
} CtrlTsk_Data;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
