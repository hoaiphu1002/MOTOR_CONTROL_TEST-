///*
// * can_tranceive.c
// *
// *  Created on: Aug 30, 2025
// *      Author: TRƯƠNG VŨ HOÀI PHÚ
// */
//
//#include "can_tranceive.h"
//#include <string.h>
//#include <stdio.h>
//
//extern CAN_HandleTypeDef hcan2;
//extern UART_HandleTypeDef huart2;
//
//volatile uint32_t can_rx_count = 0;
//volatile uint32_t can_rx_flag = 0;
//volatile int32_t currentVel1 = 0, currentVel2 = 0;
//volatile uint8_t node_booted[3] = {0};
//volatile uint8_t ready1=0, ready2=0;
//uint32_t up_vel1, up_vel2;
//
//static CAN_TxHeaderTypeDef TxHeader;
//static uint32_t txMailbox;
//
//static void print_uart(const char *msg) {
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}
//
//// ==== Các hàm SDO ====
//void send_sdo_write_u8(uint8_t nodeId, uint16_t index, uint8_t subidx, uint8_t value) {
//    uint8_t data[8] = {0x2F, index & 0xFF, (index >> 8), subidx, value, 0,0,0};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8; TxHeader.IDE = CAN_ID_STD; TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//    HAL_Delay(10);
//}
//
//void send_sdo_write_u16(uint8_t nodeId, uint16_t index, uint8_t subidx, uint16_t value) {
//    uint8_t data[8] = {0x2B, index & 0xFF, (index >> 8), subidx, value & 0xFF, (value>>8)&0xFF, 0,0};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8; TxHeader.IDE = CAN_ID_STD; TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//    HAL_Delay(10);
//}
//
//void send_sdo_write_u32(uint8_t nodeId, uint16_t index, uint8_t subidx, uint32_t value) {
//    uint8_t data[8] = {0x23, index & 0xFF, (index >> 8), subidx,
//                       (value)&0xFF, (value>>8)&0xFF, (value>>16)&0xFF, (value>>24)&0xFF};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8; TxHeader.IDE = CAN_ID_STD; TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//    HAL_Delay(10);
//}
//
//// ==== PDO Remap ====
//void remap_rpdo1_for_velocity(uint8_t nodeId) {
//    send_sdo_write_u32(nodeId, 0x1400, 0x01, 0x80000200); // disable RPDO1
//    send_sdo_write_u8 (nodeId, 0x1600, 0x00, 0);
//    send_sdo_write_u32(nodeId, 0x1600, 0x01, 0x60400010); // Controlword
//    send_sdo_write_u32(nodeId, 0x1600, 0x02, 0x60FF0020); // Target velocity
//    send_sdo_write_u8 (nodeId, 0x1600, 0x00, 2);
//    send_sdo_write_u32(nodeId, 0x1400, 0x01, 0x00000200); // enable RPDO1
//    print_uart("✅ RPDO1 re-mapped\n");
//}
//
//void remap_tpdo1_velocity(uint8_t nodeId) {
//    send_sdo_write_u32(nodeId, 0x1800, 0x01, 0x80000200);
//    send_sdo_write_u8 (nodeId,  0x1A00, 0x00, 0);
//    send_sdo_write_u32(nodeId, 0x1A00, 0x01, 0x606C0020);
//    send_sdo_write_u8 (nodeId,  0x1A00, 0x00, 1);
//    send_sdo_write_u32(nodeId, 0x1800, 0x01, 0x00000200);
//}
//
//// ==== Enable sequence ====
//void send_enable_sequence(uint8_t nodeId) {
//    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x0080); HAL_Delay(10);
//    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x06);   HAL_Delay(10);
//    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x07);   HAL_Delay(10);
//    send_sdo_write_u16(nodeId, 0x6040, 0x00, 0x0F);   HAL_Delay(10);
//}
//
//// ==== SYNC frame ====
//void send_sync_frame(void) {
//    CAN_TxHeaderTypeDef tx; uint8_t dummy = 0; uint32_t mbox;
//    tx.StdId = 0x080; tx.IDE = CAN_ID_STD; tx.RTR = CAN_RTR_DATA; tx.DLC = 0;
//    HAL_CAN_AddTxMessage(&hcan2, &tx, &dummy, &mbox);
//}
//
//// ==== Gửi vận tốc RPDO ====
//void send_velocity_rpdo(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel) {
//    static bool toggle = false;
//    uint8_t data[8];
//    send_sdo_write_u32(node, 0x6083, 0x00, accel);
//    send_sdo_write_u32(node, 0x6084, 0x00, decel);
//    uint16_t cw = toggle_cw ? (toggle ? 0x1F : 0x0F) : 0x0F;
//    toggle = !toggle;
//    data[0]=cw&0xFF; data[1]=cw>>8;
//    data[2]=velocity; data[3]=velocity>>8; data[4]=velocity>>16; data[5]=velocity>>24;
//    data[6]=0; data[7]=0;
//    TxHeader.StdId=0x200+node; TxHeader.DLC=8; TxHeader.IDE=CAN_ID_STD; TxHeader.RTR=CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//}
//
//// ==== Callback nhận CAN ====
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//    CAN_RxHeaderTypeDef rxHeader; uint8_t rxData[8]; char buf[64];
//    can_rx_count++; can_rx_flag++;
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) return;
//
//    // Boot-up
//    if ((rxHeader.StdId == 0x701 || rxHeader.StdId == 0x702) && rxData[0] == 0x00) {
//        uint8_t nodeId = rxHeader.StdId - 0x700; node_booted[nodeId] = 1;
//    }
//
//    // Actual Velocity
//    if ((rxHeader.StdId == 0x581 || rxHeader.StdId == 0x582) && rxData[0] == 0x43 && rxData[1]==0x6C) {
//        int nodeId = rxHeader.StdId - 0x580;
//        int32_t velocity = (int32_t)(rxData[4] | (rxData[5]<<8) | (rxData[6]<<16) | (rxData[7]<<24));
//        if(nodeId==1) currentVel1=velocity;
//        if(nodeId==2) currentVel2=velocity;
//        snprintf(buf,sizeof(buf),"🔄 Node%d Vel=%ld\r\n",nodeId,velocity); print_uart(buf);
//    }
//
//    // AUTO mode update (0x013)
//    if (rxHeader.StdId == 0x013 && rxHeader.DLC == 8) {
//        memcpy(&up_vel1, &rxData[0], 4); memcpy(&up_vel2, &rxData[4], 4);
//        ready1=1; ready2=1;
//    }
//}
//// ==== Set drive mode (Velocity, Position, Torque) ====
//void set_drive_mode(uint8_t mode, uint8_t node_id) {
//    // mode: 1=position, 3=velocity, 4=torque
//    send_sdo_write_u8(node_id, 0x6060, 0x00, mode);
//    HAL_Delay(50);
//    send_enable_sequence(node_id);
//    char buf[64];
//    snprintf(buf, sizeof(buf), "⚙️ Node%d set mode=%d\r\n", node_id, mode);
//    print_uart(buf);
//}
//
//// ==== Request actual velocity ====
//void request_actual_velocity(uint8_t node_id) {
//    uint8_t data[8] = {0x40, 0x6C, 0x60, 0x00, 0,0,0,0};
//    TxHeader.StdId = 0x600 + node_id;
//    TxHeader.DLC   = 8;
//    TxHeader.IDE   = CAN_ID_STD;
//    TxHeader.RTR   = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//}
//
//// ==== Send 2 velocities at once ====
//void send_vel_can(int32_t vel1, int32_t vel2) {
//    send_velocity_rpdo(1, vel1, false, 5000, 5000);
//    send_velocity_rpdo(2, vel2, false, 5000, 5000);
//}
//
