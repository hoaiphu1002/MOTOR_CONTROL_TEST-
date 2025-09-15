/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include <stdlib.h>
//#include "BNO055_STM32.h"
#include "string.h"
#include "math.h"
#include <stdbool.h>
#include "can_topic.h"
extern volatile uint16_t OD_MBDV_Statusword; // Khai báo extern
extern volatile int32_t OD_MBDV_PositionActual; // Khai báo extern
#define __PRINTF_FLOAT_SUPPORTED__ 1
#define MODE_POSITION 1
#define MODE_VELOCITY 3
#define MODE_TORQUE   4

// BẢNG ÁNH XẠ NÚT CỦA PS2 (16 bit)
#define BTN_SELECT    0x0001
#define BTN_L3        0x0002
#define BTN_R3        0x0004
#define BTN_START     0x0008
#define BTN_UP        0x0010
#define BTN_RIGHT     0x0020
#define BTN_DOWN      0x0040
#define BTN_LEFT      0x0080
#define BTN_L2        0x0100
#define BTN_R2        0x0200
#define BTN_L1        0x0400
#define BTN_R1        0x0800
#define BTN_TRIANGLE  0x1000
#define BTN_CIRCLE    0x2000
#define BTN_CROSS     0x4000
#define BTN_SQUARE    0x8000

//Guitar Hero (Guitar mode) mapping
#define GREEN_FRET   0x0200  // giống L2
#define RED_FRET     0x2000  // giống Circle
#define BLUE_FRET    0x4000
#define ORANGE_FRET  0x8000
#define WHAMMY_BAR   8       // Index analog

//Joystick chỉ số analog
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8


#define PS2_CMD_PORT GPIOA
#define PS2_CMD_PIN GPIO_PIN_7
#define PS2_DAT_PORT GPIOA
#define PS2_DAT_PIN GPIO_PIN_6
#define PS2_CLK_PORT GPIOA
#define PS2_CLK_PIN GPIO_PIN_5
#define PS2_ATT_PORT GPIOE
#define PS2_ATT_PIN GPIO_PIN_3

#define NODE_ID(node) ((node)->desiredN
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_rx_flag = 0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum {
    MODE_PS2 = 0,
    MODE_AUTO = 1
} ControlMode_t;

volatile ControlMode_t current_mode = MODE_PS2;

// Biến trạng thái để điều khiển quá trình khởi động
typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_CANOPEN_INIT,
    APP_STATE_WAIT_FOR_BOOTUP,
    APP_STATE_SEND_NMT_OPERATIONAL,
    APP_STATE_READ_MODE,
    APP_STATE_SET_MODE,
    APP_STATE_ENABLE_OPERATION,
    APP_STATE_SET_TARGET_POSITION,
    APP_STATE_TRIGGER_MOTION,
    APP_STATE_OPERATIONAL
} App_State_t;

App_State_t appState = APP_STATE_INIT; // Trạng thái ứng dụng hiện tại
uint32_t SDO_timeout_ms = 500; // Thời gian chờ SDO
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C3_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 volatile uint32_t count_sync = 0;
   volatile uint32_t count_rpdo1 = 0;
   volatile uint32_t count_rpdo2 = 0;
   volatile uint32_t count_request_vel = 0;
   volatile uint32_t count_send_vel_can = 0;
uint16_t cw;
CAN_RxHeaderTypeDef RxHeader;
	  uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
uint32_t TxMailbox;

void print_uart(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
#define UART_RX_BUF_SIZE 64
char uart_rx_buf[UART_RX_BUF_SIZE];
volatile uint8_t uart_rx_index = 0;
volatile uint8_t uart_rx_ready = 0;
void handle_uart_command(const char* cmd) {
    if (strncmp(cmd, "1", 1) == 0) {
        current_mode = MODE_PS2;
        print_uart("✅PS2 Mode\r\n");
    } else if (strncmp(cmd, "2",1) == 0) {
        current_mode = MODE_AUTO;
        print_uart("✅AUTO Mode\r\n");
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        char received = uart_rx_buf[uart_rx_index];

        if (received == '\n' || received == '\r') {
            uart_rx_buf[uart_rx_index] = '\0';
            uart_rx_ready = 1;
            uart_rx_index = 0;
        } else {
            uart_rx_index++;
            if (uart_rx_index >= UART_RX_BUF_SIZE)
                uart_rx_index = 0;
        }

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_buf[uart_rx_index], 1);
    }
}



void set_drive_mode(uint8_t mode, uint8_t nodeId) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t txMailbox;
    uint8_t data[8];
    char msg[64];

    // === Gửi NMT Start Node ===
    TxHeader.StdId = 0x000;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;
    data[0] = 0x01;       // Command: Start remote node
    data[1] = nodeId;     // Node ID
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
    HAL_Delay(100);
    snprintf(msg, sizeof(msg), "🔌 NMT Start sent to Node %d\r\n", nodeId);
    print_uart(msg);

    // === Set Mode to Velocity (0x6060 = 3) ===
    TxHeader.StdId = 0x600 + nodeId;  // COB-ID for SDO Tx
    TxHeader.DLC = 8;
    uint8_t mode_vel[] = {0x2F, 0x60, 0x60, 0x00, mode, 0x00, 0x00, 0x00};
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, mode_vel, &txMailbox);
    HAL_Delay(50);
    print_uart("⚙️ Set mode to Velocity (0x6060 = 3)\r\n");

    // === Gửi chuỗi enable: 0x06 → 0x07 → 0x0F ===

    // 1. Shutdown (0x06)
    uint8_t cw_shutdown[] = {0x2F, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, cw_shutdown, &txMailbox);
    HAL_Delay(50);
    print_uart("🔄 CW = 0x06 (Shutdown)\r\n");

    // 2. Switch ON (0x07)
    uint8_t cw_switchon[] = {0x2F, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, cw_switchon, &txMailbox);
    HAL_Delay(50);
    print_uart("🔄 CW = 0x07 (Switch ON)\r\n");

    // 3. Enable operation (0x0F)
    uint8_t cw_enable[] = {0x2F, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, cw_enable, &txMailbox);
    HAL_Delay(100);
    print_uart("✅ CW = 0x0F (Enable Operation)\r\n");

    print_uart("✅ Servo is now enabled in Velocity Mode!\r\n");
}



void delay_us(uint16_t us) {
   htim1.Instance->CNT=0;
    HAL_TIM_Base_Start(&htim1);            // Start timer
    while (htim1.Instance->CNT < us);
    HAL_TIM_Base_Stop(&htim1);             // Optional: Stop to save power
}

//LAP TRINH DIEU KHIEN TAY CAM
void PS2_ATT_LOW() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

void PS2_ATT_HIGH() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
// === PS2 Controller Interface for STM32 ===
// === PS2 Initialization (3 command sequences) ===
void PS2_SendCommand(const uint8_t *tx, uint8_t *rx, uint8_t len) {
    PS2_ATT_LOW(); delay_us(15);
    for (uint8_t i = 0; i < len; i++) {
        HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx[i], (uint8_t*)&rx[i], 1, 100);
        delay_us(10);
    }
    PS2_ATT_HIGH(); delay_us(30);
}
void PS2_Init(void) {
    const uint8_t enter_cfg[]   = {0x01, 0x43, 0x00, 0x01, 0x00};
    const uint8_t set_analog[]  = {0x01, 0x44, 0x00, 0x01, 0x03};
    const uint8_t enable_rumble[]= {0x01, 0x4D, 0x00, 0x00, 0x01};
    const uint8_t exit_cfg[]    = {0x01, 0x43, 0x00, 0x00, 0x5A};
    uint8_t rx[9];

    PS2_SendCommand(enter_cfg, rx, sizeof(enter_cfg));
    PS2_SendCommand(set_analog, rx, sizeof(set_analog));
    PS2_SendCommand(enable_rumble, rx, sizeof(enable_rumble));
    PS2_SendCommand(exit_cfg, rx, sizeof(exit_cfg));

    HAL_Delay(500);
    print_uart("✅ PS2 Init Done\r\n");
}

typedef struct {
    uint16_t buttons;
    uint8_t rx, ry, lx, ly;
} PS2_Data;

bool isAnalog = false;

PS2_Data PS2_ReadButtons(void) {
    PS2_Data result = {0xFFFF, 128, 128, 128, 128};
    uint8_t tx[9] = {0x01, 0x42, 0x00, 0, 0, 0, 0, 0, 0};
    uint8_t rx[9] = {0};
    PS2_SendCommand(tx, rx, 9);
    isAnalog = (rx[1] == 0x73);

    result.buttons = (rx[4] << 8) | rx[3];
    result.rx = rx[5];
    result.ry = rx[6];
    result.lx = rx[7];
    result.ly = rx[8];

    return result;
}

void Test_SPI_1Byte(void) {
    uint8_t tx = 0x01;       // Gửi byte 0x01 (giống trong PS2 poll)
    uint8_t rx = 0x00;

    PS2_ATT_LOW();           // Kéo ATT xuống (CS = LOW)
    delay_us(15);

    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100);

    PS2_ATT_HIGH();          // Thả ATT lên (CS = HIGH)

    char msg[64];
    snprintf(msg, sizeof(msg), "SPI test RX: 0x%02X\r\n", rx);
    print_uart(msg);
}
#define DEADZONE 20
void test_joystick(const char *label, uint8_t raw_x, uint8_t raw_y) {
    int8_t x = raw_x - 128;
    int8_t y = raw_y - 128;

    const char *direction = "🟡 Trung tâm";

    if (x > DEADZONE) {
        direction = "➡️ Phải";
    } else if (x < -DEADZONE) {
        direction = "⬅️ Trái";
    }

    if (y > DEADZONE) {
        direction = "⬇️ Xuống";
    } else if (y < -DEADZONE) {
        direction = "⬆️ Lên";
    }

    // Nếu nằm chéo, gộp hướng
    if (abs(x) > DEADZONE && abs(y) > DEADZONE) {
        if (x > 0 && y < 0) {
        	direction = "↗️ Lên-Phải";
        }
        else if (x < 0 && y < 0) {
        	direction = "↖️ Lên-Trái";
        }
        else if (x > 0 && y > 0) {
        	direction = "↘️ Xuống-Phải";
        }
        else if (x < 0 && y > 0) {
        	direction = "↙️ Xuống-Trái";
        }
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "🎮 %s X=%3d Y=%3d → %s\r\n", label, raw_x, raw_y, direction);
    print_uart(msg);
}

                            //CAN THUẦN LẬP TRÌNH THEO TIÊU CHUẨN CIA402,301 -> ĐIỀU KHIỂN ĐỘNG CƠ//

void send_sdo_write_u8(uint8_t nodeId, uint16_t index, uint8_t subidx, uint8_t value) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t txMailbox;
    uint8_t data[8] = {
        0x2F,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subidx,
        value,
        0x00,
        0x00,
        0x00
    };

    TxHeader.StdId = 0x600 + nodeId;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
    HAL_Delay(10);
}
void send_sdo_write_u16(uint8_t nodeId, uint16_t index, uint8_t subidx, uint16_t value) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t txMailbox;
    uint8_t data[8] = {
        0x2B,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subidx,
        value & 0xFF,
        (value >> 8) & 0xFF,
        0x00,
        0x00
    };

    TxHeader.StdId = 0x600 + nodeId;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
    HAL_Delay(10);
}
void send_sdo_write_u32(uint8_t nodeId, uint16_t index, uint8_t subidx, uint32_t value) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t txMailbox;
    uint8_t data[8] = {
        0x23,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subidx,
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF
    };

    TxHeader.StdId = 0x600 + nodeId;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
    HAL_Delay(10);
}
void remap_rpdo1_for_velocity(uint8_t nodeId) {
    // 1. Disable RPDO1
    send_sdo_write_u32(nodeId, 0x1400, 0x01, 0x80000200);

    // 2. Clear existing mapping
    send_sdo_write_u8(nodeId, 0x1600, 0x00, 0);

    // 3. Map Controlword (0x6040, 16-bit)
    send_sdo_write_u32(nodeId, 0x1600, 0x01, 0x60400010);

    // 4. Map Target Velocity (0x60FF, 32-bit)
    send_sdo_write_u32(nodeId, 0x1600, 0x02, 0x60FF0020);

    // 5. Set number of mapped entries = 2
    send_sdo_write_u8(nodeId, 0x1600, 0x00, 2);

    // 6. Enable RPDO1 (SYNC: 0x00000200)
    send_sdo_write_u32(nodeId, 0x1400, 0x01, 0x00000200);

    print_uart("✅ RPDO1 re-mapped for CW + Target Velocity!\r\n");
}
void remap_tpdo1_velocity(uint8_t nodeId) {
    send_sdo_write_u32(nodeId, 0x1800, 0x01, 0x80000200); // disable TPDO1
    send_sdo_write_u8(nodeId,  0x1A00, 0x00, 0);           // clear mapping
    send_sdo_write_u32(nodeId, 0x1A00, 0x01, 0x606C0020);  // map 606C: 32-bit
    send_sdo_write_u8(nodeId,  0x1A00, 0x00, 1);           // 1 entry
    send_sdo_write_u32(nodeId, 0x1800, 0x01, 0x00000200);  // enable TPDO1
}

void send_enable_sequence(uint8_t nodeId) {
	// 1. Fault Reset trước
	    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x0080);
	    HAL_Delay(10);
    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x06); HAL_Delay(10); // Shutdown
    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x07); HAL_Delay(10); // Switch on
    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x0F); HAL_Delay(10); // Enable operation
}

void send_sync_frame() {
    CAN_TxHeaderTypeDef tx;
    uint8_t dummy = 0;
    uint32_t mbox;
    tx.StdId = 0x080;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 0;
    HAL_CAN_AddTxMessage(&hcan2, &tx, &dummy, &mbox);
    count_sync++;
}

uint32_t txMailbox;

void send_velocity_rpdo(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel) {
    static bool toggle = false;
    uint8_t data[8];

    // 1. Set acceleration & deceleration trước khi gửi velocity
    send_sdo_write_u32(node, 0x6083, 0x00, accel);  // acceleration
    HAL_Delay(2);
    send_sdo_write_u32(node, 0x6084, 0x00, decel);  // deceleration
    HAL_Delay(2);

    // 2. Toggle CW
    uint16_t cw = toggle_cw ? (toggle ? 0x1F : 0x0F) : 0x0F;
    toggle = !toggle;

    // 3. Build RPDO data (CW + velocity)
    data[0] = cw & 0xFF;
    data[1] = (cw >> 8) & 0xFF;

    data[2] = (velocity >> 0) & 0xFF;
    data[3] = (velocity >> 8) & 0xFF;
    data[4] = (velocity >> 16) & 0xFF;
    data[5] = (velocity >> 24) & 0xFF;

    data[6] = 0;
    data[7] = 0;

    // 4. Gửi RPDO
    TxHeader.StdId = 0x200 + node;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
    HAL_Delay(5);
}
int32_t read_actual_velocity(uint8_t nodeId) {
    CAN_TxHeaderTypeDef tx;
    CAN_RxHeaderTypeDef rx;
    uint8_t txData[8] = {0};
    uint8_t rxData[8] = {0};
    uint32_t txMailbox;

    // ==== Gửi yêu cầu đọc SDO (0x606C:00) ====
    tx.StdId = 0x600 + nodeId;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    txData[0] = 0x40;      // SDO Upload request
    txData[1] = 0x6C;      // Index LSB (0x606C)
    txData[2] = 0x60;      // Index MSB
    txData[3] = 0x00;      // Subindex = 0
    // txData[4..7] = 0
    HAL_CAN_AddTxMessage(&hcan2, &tx, txData, &txMailbox);
HAL_Delay(100);
if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx, rxData) != HAL_OK) {
       print_uart("❌ Failed to get RX message\r\n");
       return 0;
   }
HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx, rxData);
if (rx.StdId != (0x580 + nodeId) || rxData[0] != 0x43) {
    print_uart("❌ Invalid SDO response!\r\n");
    return 0;
}

    // ==== Ghép vận tốc từ Little Endian 4 byte ====
    int32_t velocity = (int32_t)(
        ((uint32_t)rxData[4]) |
        ((uint32_t)rxData[5] << 8) |
        ((uint32_t)rxData[6] << 16) |
        ((uint32_t)rxData[7] << 24)
    );

    return velocity;
}

void print_velocity_both_nodes() {
    int32_t v1 = read_actual_velocity(1);
    int32_t v2 = read_actual_velocity(2);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "🔄 Velocities | Node1: %ld  | Node2: %ld\r\n", v1, v2);
    print_uart(msg);

    // In ra toàn bộ các frame còn trong FIFO
    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rx;
        uint8_t rxData[8];

        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx, rxData);

        char log[128];
        snprintf(log, sizeof(log),
            "📥 CAN RX: ID=0x%03lX | DLC=%ld | Data= %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            rx.StdId,
            rx.DLC,
            rxData[0], rxData[1], rxData[2], rxData[3],
            rxData[4], rxData[5], rxData[6], rxData[7]);
        print_uart(log);
   }
}

uint32_t up_vel1, up_vel2, last_vel1, last_vel2;
int32_t currentVel1 = 0, currentVel2 = 0; // bien de gui qua can
//static uint8_t lostCounter1 = 0, lostCounter2 = 0;
//extern volatile bool need_reenable_node1, need_reenable_node2;
//extern int32_t prevVel1=0, prevVel2=0;  // vận tốc yêu cầu gần nhất
volatile uint8_t ready1=0;
volatile uint8_t ready2=0;
void send_temp_to_usbcan(int32_t temperature) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t txMailbox;
    uint8_t data[8] = {0};

    TxHeader.StdId = 0x018;
    TxHeader.DLC = 4;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    data[0] = (uint8_t)(temperature & 0xFF);
    data[1] = (uint8_t)((temperature >> 8) & 0xFF);
    data[2] = (uint8_t)((temperature >> 16) & 0xFF);
    data[3] = (uint8_t)((temperature >> 24) & 0xFF);

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
}
// ⬆️ Global biến bổ sung
volatile uint8_t node_booted[3] = {0};
volatile uint8_t  drive_enabled1 = 0;
volatile uint8_t  drive_enabled2 = 0;
volatile uint8_t ps2_blocked = 0;  // Nếu bị block bởi vật cản
volatile uint8_t mode_changed_flag = 0;  // Dùng để reset vận tốc trong vòng lặp
uint32_t last_stop_update = 0;  // thời gian cuối nhận gói stop = 1/2/3
//Ham ngat xu ly:
//Xử lý Statusword (enable/disable).
//
//Xử lý Actual Velocity.
//
//Xử lý Boot-up message (0x701, 0x702).
//
//Xử lý Mode change (0x020).
//
//Xử lý Temp driver (0x581 object 0x5012).
//
//Xử lý LED command (0x021).
//
//Xử lý PS2 block command (0x022).
//
//Xử lý Auto velocity update (0x013).
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
  static bool toggle = false;
  uint8_t rxData[8];
  uint32_t now = HAL_GetTick();
  (void)toggle; (void)now; /* Silence unused-variable warnings when not used */
    can_rx_count++;
    can_rx_flag ++;
   // static uint32_t lastCheck = 0;  // ✅ dùng static, không reset mỗi lần
    char buf[64];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        print_uart("❌ Failed to get RX message in callback\r\n");
        return;
    }

    if ((rxHeader.StdId == 0x701 || rxHeader.StdId == 0x702) &&
        rxHeader.DLC == 1 && rxData[0] == 0x00) {

        uint8_t nodeId = rxHeader.StdId - 0x700;
        node_booted[nodeId] = 1;

    }


    // Kiểm tra nếu đây là phản hồi từ node1 hoặc node2 cho SDO Actual Velocity
    if (rxHeader.StdId == 0x581 || rxHeader.StdId == 0x582) {
        if (rxData[0] == 0x43 && rxData[1] == 0x6C && rxData[2] == 0x60) {
            int nodeId = rxHeader.StdId - 0x580;
// 0x591, 0x582 phản hồi SDO có header SD0 4 byte đầu tiên, và dữ liệu velocity ở 4 byte cuối (4->7)
//Ví dụ:      Byte 0: Command specifier (0x43)
//            Byte 1: Index low  (0x6C)
//            Byte 2: Index high (0x60)   → Object 0x606C: Actual Velocity
//            Byte 3: Subindex (usually 0)
//            Byte 4–7: Dữ liệu 32-bit (int32_t velocity)

            int32_t velocity = (int32_t)(
                ((uint32_t)rxData[4]) |
                ((uint32_t)rxData[5] << 8) |
                ((uint32_t)rxData[6] << 16) |
                ((uint32_t)rxData[7] << 24)
            );
            if(nodeId==1) currentVel1=velocity;
            if(nodeId==2) currentVel2=velocity;
            snprintf(buf, sizeof(buf),
                     "🔄 Node %d Actual Velocity: %ld\r\n", nodeId, velocity);

            print_uart(buf);

                  }
    }
    if ((rxHeader.StdId == 0x013) && (rxHeader.DLC == 8)) { // frame chứa 2 vận tốc
    	   if (current_mode != MODE_AUTO) {
    	        // Không xử lý gói 0x013 nếu không ở AUTO mode
    	        return;
    	    }
        print_uart("oke\r\n");
        memcpy(&up_vel1, &rxData[0], 4);
        memcpy(&up_vel2, &rxData[4], 4);
        ready1 = 1; ready2 = 1;
//        if( up_vel1 >= 80000 ) up_vel1=80000;
//        else if( up_vel1 <= -80000) up_vel1=-80000;
//        if( up_vel2 >= 80000 ) up_vel2=80000;
//        else if( up_vel2 <= -80000) up_vel2=-80000;
        snprintf(buf, sizeof(buf), "✅ Velocity 1 (Motor1) updated: %ld\r\n", up_vel1);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

        snprintf(buf, sizeof(buf),
            "📦 Data 1: %02X %02X %02X %02X\r\n",
            rxData[0], rxData[1], rxData[2], rxData[3]);
        print_uart(buf);

        snprintf(buf, sizeof(buf), "✅ Velocity 2 (Motor2) updated: %ld\r\n", up_vel2);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

        snprintf(buf, sizeof(buf),
            "📦 Data 2: %02X %02X %02X %02X\r\n",
            rxData[4], rxData[5], rxData[6], rxData[7]);
        print_uart(buf);

    }

    if ((rxHeader.StdId == 0x020) && (rxHeader.DLC == 8)) {
        uint32_t mode;
        memcpy(&mode, &rxData[0], 4);

        if (mode == 1) {  // MODE_PS2
            current_mode = MODE_PS2;
            mode_changed_flag = 1;   // Để vòng while reset tốc độ
            print_uart("✅ PS2 Mode\r\n");
        }
        else if (mode == 2) { // MODE_AUTO
            current_mode = MODE_AUTO;
            mode_changed_flag = 1;   // Để vòng while reset tốc độ
            print_uart("✅ Auto Mode\r\n");
        }
        else {
            char buf[64];
            snprintf(buf, sizeof(buf), "⚠️ Unknown Mode: %lu\r\n", mode);
            print_uart(buf);
        }
    }

    if ((rxHeader.StdId == 0x581) &&
        rxData[0] == 0x43 &&
        rxData[1]==0x12 && rxData[2]==0x50 && rxData[3]==0x00) {

        int32_t driver_temp = (int32_t)(
            ((uint32_t)rxData[4]) |
            ((uint32_t)rxData[5] << 8) |
            ((uint32_t)rxData[6] << 16) |
            ((uint32_t)rxData[7] << 24)
        );

        char buf[64];
        snprintf(buf, sizeof(buf), "🌡️ Driver Temp = %ld°C\r\n", driver_temp);
        print_uart(buf);

        // Gửi lên CAN ID 0x018 để debug trên USB-CAN
        send_temp_to_usbcan(driver_temp);
    }
    //nhan lenh bat den sang 0x021
    if ((rxHeader.StdId == 0x021) && (rxHeader.DLC == 8)){
       	  uint32_t led;
       	memcpy(&led, &rxData[0], 4);
   //    	 mode = (int32_t)((rxData[0]) |
   //    	                           (rxData[1] << 8) |
   //    	                           (rxData[2] << 16) |
   //    	                           (rxData[3] << 24));
        static uint32_t last_led_cmd = 0xFFFFFFFF;  // Lưu lệnh trước để tránh xử lý lại

           // Nếu lệnh mới khác lệnh cũ thì xử lý (tránh lặp lại nhiều lần)
           if (led != last_led_cmd) {
               last_led_cmd = led;
       	 if (led==1){  //01 00 00 00
       		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

       	 }
       	 else if (led==2){// 02 00 00 00
       		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

       	 }

       }
    }
    //dem goi tin nhan duoc trong 1 s
//    can_rx_count++;
    //vo hiẹu hoa ps2 khi co vat can

    if ((rxHeader.StdId == 0x024) && (rxHeader.DLC == 8)){
          	  uint32_t stop;
          	memcpy(&stop, &rxData[0], 4);
      //    	 mode = (int32_t)((rxData[0]) |
      //    	                           (rxData[1] << 8) |
      //    	                           (rxData[2] << 16) |
      //    	                           (rxData[3] << 24));
      if (stop==1||stop==2||stop==3){

    	  ps2_blocked=1;
    	  //last_stop_update = HAL_GetTick();
      }
     // else if(stop==0) ps2_blocked=0;
      else ps2_blocked=0;
    }
//    else ps2_blocked=0;

}
void send_vel_can(int32_t vel1, int32_t vel2) {
    static uint32_t last_tick = 0;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t txMailbox;
    uint8_t data[8];

    // ===== Giới hạn tần suất gửi: mỗi 20ms một lần (50Hz) =====
    if (HAL_GetTick() - last_tick < 50) return;
    last_tick = HAL_GetTick();

    // ===== Header CAN =====
    TxHeader.StdId = 0x030;   // ID cố định
    TxHeader.DLC   = 8;       // 8 byte (vel1 + vel2)
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;

    // ===== Đóng gói vel1 =====
    data[0] = (uint8_t)(vel1 & 0xFF);
    data[1] = (uint8_t)((vel1 >> 8) & 0xFF);
    data[2] = (uint8_t)((vel1 >> 16) & 0xFF);
    data[3] = (uint8_t)((vel1 >> 24) & 0xFF);

    // ===== Đóng gói vel2 =====
    data[4] = (uint8_t)(vel2 & 0xFF);
    data[5] = (uint8_t)((vel2 >> 8) & 0xFF);
    data[6] = (uint8_t)((vel2 >> 16) & 0xFF);
    data[7] = (uint8_t)((vel2 >> 24) & 0xFF);

    // ===== Gửi nếu Mailbox rảnh =====
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox) == HAL_OK) {
        count_send_vel_can++;
    } else {
        // Nếu mailbox đầy, bỏ qua frame này để tránh nghẽn
        // (có thể thêm debug ở đây nếu muốn)
    }
}


void update_vel(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel){
	uint8_t data[8];
	static bool toggle = false;
    // 1. Set acceleration & deceleration trước khi gửi velocity
    send_sdo_write_u32(node, 0x6083, 0x00, accel);  // acceleration
    HAL_Delay(2);
    send_sdo_write_u32(node, 0x6084, 0x00, decel);  // deceleration
    HAL_Delay(2);

    // 2. Toggle CW
    uint16_t cw = toggle_cw ? (toggle ? 0x1F : 0x0F) : 0x0F;
    toggle = !toggle;
	// phân tích vận tốc cần thay đổi -> hệ hex để gửi đến driver
	    data[0] = cw & 0xFF;
	    data[1] = (cw >> 8) & 0xFF;

	    data[2] = (velocity >> 0) & 0xFF;
	    data[3] = (velocity >> 8) & 0xFF;
	    data[4] = (velocity >> 16) & 0xFF;
	    data[5] = (velocity >> 24) & 0xFF;

	    data[6] = 0;
	    data[7] = 0;

	    // 4. Gửi RPDO
	    TxHeader.StdId = 0x200 + node;
	    TxHeader.IDE = CAN_ID_STD;
	    TxHeader.RTR = CAN_RTR_DATA;
	    TxHeader.DLC = 8;
	    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
	    // ===== Gửi nếu Mailbox rảnh =====
	    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox) == HAL_OK) {
	        count_send_vel_can++;
	    } else {
	        // Nếu mailbox đầy, bỏ qua frame này để tránh nghẽn
	        // (có thể thêm debug ở đây nếu muốn)
	    }
}
// => sau đó gọi update_vel(1,vel1,true,50000, 150000) -> để cập nhật vận tốc đến driver và motor 1
// => sau đó gọi update_vel(2,vel2,true,50000, 150000) -> để cập nhật vận tốc đến driver và motor 2
void request_actual_velocity(uint8_t nodeId) {
    CAN_TxHeaderTypeDef tx;
    uint8_t txData[8] = {0x40, 0x6C, 0x60, 0x00, 0, 0, 0, 0};
    uint32_t txMailbox;

    tx.StdId = 0x600 + nodeId;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan2, &tx, txData, &txMailbox);
}
//doc statusword tu driver
void request_statusword(uint8_t nodeId) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};

    txHeader.StdId = 0x600 + nodeId;  // SDO Tx COB-ID
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 8;

    // ==== SDO Upload Request ====
    txData[0] = 0x40;       // Command byte: upload request
    txData[1] = 0x41;       // Low byte of index (0x6041)
    txData[2] = 0x60;       // High byte of index
    txData[3] = 0x00;       // Subindex
    txData[4] = 0x00;       // Reserved
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    uint32_t txMailbox;
    HAL_CAN_AddTxMessage(&hcan2, &txHeader, txData, &txMailbox);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->ErrorCode & HAL_CAN_ERROR_BOF) {
        HAL_CAN_Stop(hcan);
        HAL_CAN_DeInit(hcan);
        HAL_CAN_Init(hcan);
        HAL_CAN_Start(hcan);

        // Bật lại interrupt
        HAL_CAN_ActivateNotification(hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR);
//
//        char msg[] = "⚡ CAN bus-off recovered\r\n";
//        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	    HAL_TIM_Base_Start(&htim1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
    // === Cấu hình CAN filter ===
    HAL_CAN_Start(&hcan2);
    set_drive_mode(3,1);  set_drive_mode(3,2);
    remap_rpdo1_for_velocity(1); remap_rpdo1_for_velocity(2);
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterBank = 14;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &filter);

//    CAN_FilterTypeDef filter;
//    filter.FilterActivation = CAN_FILTER_ENABLE;
//    filter.FilterBank = 14;
//    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    filter.FilterMode = CAN_FILTERMODE_IDLIST;
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;
//
//    // ID 1: 0x030
//    filter.FilterIdHigh     = (0x030 << 5);
//    filter.FilterIdLow      = 0x0000;
//
//    // ID 2: 0x024
//    filter.FilterMaskIdHigh = (0x024 << 5);
//    filter.FilterMaskIdLow  = 0x0000;
//
//    filter.SlaveStartFilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &filter);

//    HAL_CAN_Start(&hcan2);
//HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
HAL_CAN_ActivateNotification(&hcan2,
	CAN_IT_RX_FIFO0_MSG_PENDING|
    CAN_IT_ERROR_WARNING |
    CAN_IT_ERROR_PASSIVE |
    CAN_IT_BUSOFF |
    CAN_IT_LAST_ERROR_CODE |
    CAN_IT_ERROR);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   // print_uart("🔧 Bắt đầu khởi tạo Velocity Mode...\r\n");
// CANopenNodeSTM32 axis;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);//on led
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    //print_uart("Nhấn MODE nếu cần cho Analog\r\n");
   // HAL_Delay(1000);
    PS2_Init();
   // print_uart("Đang test SPI đơn giản...\r\n");
    Test_SPI_1Byte();

       set_drive_mode(3,1);  set_drive_mode(3,2);
       remap_rpdo1_for_velocity(1); remap_rpdo1_for_velocity(2);
       remap_tpdo1_velocity(1);  // Remap TPDO1 để chứa Actual Velocity
       remap_tpdo1_velocity(2);
    // === Gửi lệnh quay bằng RPDO ===
  //  int32_t target_velocity1 = 50000;  // hoặc -100000 để quay ngược
    uint32_t lastSend = HAL_GetTick();
  //  int32_t target_velocity2 = -50000;

//    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // === Biến toàn cục trong main ===
  int32_t prevVel1 = 0, prevVel2 = 0;
  uint32_t lastPrint = 0;
  static uint8_t last_block_state = 0;
  uint32_t last_upvel1_time = 0;
  uint32_t last_upvel2_time = 0;
  (void)last_upvel1_time; (void)last_upvel2_time; /* Silence unused-variable warnings */


    while (1) {

    	  /* USER CODE END WHILE */

    	   /* USER CODE BEGIN 3 */
    	// Nếu cả 2 node booted cùng lúc
    	if (node_booted[1] && node_booted[2]) {
    	    node_booted[1] = 0;
    	    node_booted[2] = 0;
    	    HAL_Delay(100);

    	    // --- NMT Start cả 2 ---
//    	    send_nmt_start(1);
//    	    send_nmt_start(2);
//    	    HAL_Delay(10);

    	    // --- Cấu hình Mode + PDO ---
    	    set_drive_mode(3,1);
    	    set_drive_mode(3,2);

    	    remap_rpdo1_for_velocity(1);
    	    remap_rpdo1_for_velocity(2);

    	    remap_tpdo1_velocity(1);
    	    remap_tpdo1_velocity(2);

    	    // --- Reset vận tốc về 0 ---
    	    send_velocity_rpdo(1, 0, true, 50000, 200000);
    	    send_velocity_rpdo(2, 0, true, 50000, 200000);
    	    HAL_Delay(20);

    	    // --- Enable đồng thời ---
    	    send_enable_sequence(1);
    	    send_enable_sequence(2);

    	    print_uart("✅ Cả 2 node đã re-enable đồng bộ\r\n");
    	}


    	 int32_t vel1 = 0, vel2 = 0;
    	if (can_rx_flag){
    	    	char buf [64];
    	        if (HAL_GetTick() - lastPrint >= 1000) { //1s
    	            snprintf(buf, sizeof(buf), "📥 Gói CAN nhận/giây: %lu\r\n", can_rx_count);
    	         print_uart(buf);
    	   can_rx_count=0;
    	         lastPrint=HAL_GetTick();
    	        }
    	}
    	//gửi gói CAN
        uint32_t now = HAL_GetTick();
        // Nếu vừa đổi mode → reset vận tốc ngay
          if (mode_changed_flag) {
              vel1 = 0;
              vel2 = 0;
              prevVel1 = -1;
              prevVel2 = -1;
              send_velocity_rpdo(1, 0, true, 50000, 250000);
              send_velocity_rpdo(2, 0, true, 50000, 250000);
              mode_changed_flag = 0;
          }
        static uint32_t lastPS2 = 0;
        static uint32_t last_ps2_update = 0;
        uint32_t last_stop_time = 0;
        (void)last_stop_time; /* Silence unused-variable warning */
        const int32_t jogVel = 100000; //max =180000;
        if(uart_rx_ready ){
        	uart_rx_ready=0;
        	handle_uart_command(uart_rx_buf);
        }
        // === Gửi SYNC + cập nhật vận tốc định kỳ ===
        if (now - lastSend >= 50) { //ban dau la 50, tần so hoat dong spi tam 20

        //	set_drive_mode(3,1);  set_drive_mode(3,2);
            lastSend = now;
            send_sync_frame();  // Nếu RPDO cần SYNC

        }
        // --- Kiểm tra block/unblock ---
        if (ps2_blocked != last_block_state) {
            if (ps2_blocked == 0) {
                // Vừa bỏ block → reset hết cờ
                vel1 = 0;
                vel2 = 0;
                prevVel1 = -1;
                prevVel2 = -1;
                last_ps2_update = HAL_GetTick();
                print_uart("✅ Unblocked → PS2 ready\n");
            } else {
                print_uart("⛔ Blocked by sensor\n");
            }
            last_block_state = ps2_blocked;
        }

  static uint8_t ps2_init_ok = 0; // 0 = chưa sẵn sàng, 1 = đã thấy trạng thái không nhấn
  (void)ps2_init_ok; /* Silence unused-variable warning until used */
        if ((current_mode==MODE_PS2) && (now - lastPS2 >= 20)) {
            lastPS2 = now;
        // === Đọc tay cầm PS2 ===
        PS2_Data ps2 = PS2_ReadButtons();

        //loc nhieu

        bool up_now     = !(ps2.buttons & BTN_UP);
        bool down_now   = !(ps2.buttons & BTN_DOWN);
        bool tri_now    = !(ps2.buttons & BTN_TRIANGLE);
        bool cross_now  = !(ps2.buttons & BTN_CROSS);
        bool right_now  = !(ps2.buttons & BTN_RIGHT);
        bool square_now = !(ps2.buttons & BTN_SQUARE);
        // Debug nút nhấn
        char msg[64];
       // char buf[64];
        snprintf(msg, sizeof(msg),
            "BTN: UP=%d DW=%d TR=%d CR=%d RI=%d SQ=%d\r\n",
            up_now, down_now, tri_now, cross_now, right_now, square_now);
        print_uart(msg);

        // === Gán vận tốc theo nút nhấn ===

        if (!ps2_blocked) {
        	if (up_now )         vel1 = -jogVel;
        	 if (tri_now)        vel2 = +jogVel;
        }

        if (down_now)  vel1 = +jogVel;
        if (cross_now) vel2 = -jogVel;
        if (right_now) {
            vel1 = -0.5 * jogVel;
            vel2 = +jogVel;
        }

        if (square_now) {
            vel1 = -jogVel;
            vel2 = +0.5 * jogVel;
        }


        // Nếu dữ liệu thay đổi → cập nhật thời gian cuối nhận PS2
        if (ps2.buttons != 0xFF) {
            last_ps2_update = now;
        }
       // }
        if (now - last_ps2_update > 500) {
            if (vel1 != 0 || vel2 != 0) {
                print_uart("❌ PS2 timeout, reset động cơ\r\n");
                vel1 = 0;
                vel2 = 0;
                // ✅ Gán giá trị khác để buộc gửi RPDO lại
                        prevVel1 = -1;
                        prevVel2 = -1;

             	 }
        	}
        }
        if (current_mode == MODE_AUTO) {
            static uint32_t lastSendAuto = 0;
            uint32_t now = HAL_GetTick();

            // Cập nhật giá trị mới cho từng bánh nếu có gói mới
            if (ready1) {
                vel1 = up_vel1;
                if (vel1 > jogVel) vel1 = jogVel;
                else if (vel1 < -jogVel) vel1 = -jogVel;
               // last_upvel1_time = now;
                ready1 = 0;
            }
            if (ready2) {
                vel2 = up_vel2;
                if (vel2 > jogVel) vel2 = jogVel;
                else if (vel2 < -jogVel) vel2 = -jogVel;
              //  last_upvel2_time = now;
                ready2 = 0;
            }

            // Gửi RPDO định kỳ (50 ms)
            if (now - lastSendAuto >= 10) {
             //   lastSendAuto = now;
            	lastSendAuto += 50;  // giữ nhịp đều
                send_velocity_rpdo(1, vel1, false, 50000, 250000);
                send_velocity_rpdo(2, vel2, false, 50000, 250000);
                send_sync_frame();
            }
        }
        // === Gửi RPDO nếu vận tốc thay đổi ===
        if (vel1 != prevVel1) {
            send_velocity_rpdo(1, vel1, true, 50000, 250000); // node 1
            count_rpdo1++;
            prevVel1 = vel1;
        }

        if (vel2 != prevVel2) {
            send_velocity_rpdo(2, vel2, true, 50000, 250000); // node 2
            count_rpdo2++;
            prevVel2 = vel2;
        }
        // === Đọc tốc độ thực tế mỗi 200ms ===
        if (now - lastPrint >= 200) { // cứ 100ms gửi vận tốc lên 1 làn
            lastPrint = now;
          request_actual_velocity(1);
          request_actual_velocity(2);
          count_request_vel++;
          send_vel_can(currentVel1, currentVel2);
         // send_vel_can(1, currentVel2);

        }
    }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 83;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xffff-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 83;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart_rx_buf[uart_rx_index], 1);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Bật ngắt EXTI cho PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // hoặc RISING, tùy vào mạch
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
