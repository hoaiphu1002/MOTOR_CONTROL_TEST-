/////*
//// * can_tranceive.c
//// *
//// *  Created on: Aug 30, 2025
//// *      Author: TRƯƠNG VŨ HOÀI PHÚ
//// */
////
//#include "can_tranceive.h"
//#include <string.h>
//
//static CAN_TxHeaderTypeDef TxHeader;
//static uint32_t txMailbox;
//
///* === Khởi tạo CAN === */
//void CAN_Init(void) {
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//}
//
///* === Gửi SDO === */
//void CAN_SendSDO_U8(uint8_t nodeId, uint16_t index, uint8_t subidx, uint8_t value) {
//    uint8_t data[8] = {0x2F, index & 0xFF, (index >> 8) & 0xFF, subidx, value, 0, 0, 0};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//}
//
//void CAN_SendSDO_U16(uint8_t nodeId, uint16_t index, uint8_t subidx, uint16_t value) {
//    uint8_t data[8] = {0x2B, index & 0xFF, (index >> 8) & 0xFF, subidx,
//                       value & 0xFF, (value >> 8) & 0xFF, 0, 0};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//}
//
//void CAN_SendSDO_U32(uint8_t nodeId, uint16_t index, uint8_t subidx, uint32_t value) {
//    uint8_t data[8] = {0x23, index & 0xFF, (index >> 8) & 0xFF, subidx,
//                       value & 0xFF, (value >> 8) & 0xFF,
//                       (value >> 16) & 0xFF, (value >> 24) & 0xFF};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//}
//
///* === SYNC === */
//void CAN_SendSync(void) {
//    CAN_TxHeaderTypeDef tx;
//    tx.StdId = 0x080;
//    tx.IDE = CAN_ID_STD;
//    tx.RTR = CAN_RTR_DATA;
//    tx.DLC = 0;
//    HAL_CAN_AddTxMessage(&hcan2, &tx, NULL, &txMailbox);
//}
//
///* === Velocity RPDO === */
//void CAN_SendVelocityRPDO(uint8_t node, int32_t velocity, bool toggle_cw,
//                          uint32_t accel, uint32_t decel) {
//    static bool toggle = false;
//    uint8_t data[8];
//
//    // Accel & Decel trước
//    CAN_SendSDO_U32(node, 0x6083, 0x00, accel);
//    CAN_SendSDO_U32(node, 0x6084, 0x00, decel);
//
//    // Toggle Controlword
//    uint16_t cw = toggle_cw ? (toggle ? 0x1F : 0x0F) : 0x0F;
//    toggle = !toggle;
//
//    // Build data
//    data[0] = cw & 0xFF;
//    data[1] = (cw >> 8) & 0xFF;
//    data[2] = velocity & 0xFF;
//    data[3] = (velocity >> 8) & 0xFF;
//    data[4] = (velocity >> 16) & 0xFF;
//    data[5] = (velocity >> 24) & 0xFF;
//    data[6] = 0;
//    data[7] = 0;
//
//    TxHeader.StdId = 0x200 + node;
//    TxHeader.DLC = 8;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txMailbox);
//}
//
///* === RPDO Remap === */
//void CAN_RemapRPDO1_Velocity(uint8_t nodeId) {
//    CAN_SendSDO_U32(nodeId, 0x1400, 0x01, 0x80000200);
//    CAN_SendSDO_U8(nodeId,  0x1600, 0x00, 0);
//    CAN_SendSDO_U32(nodeId, 0x1600, 0x01, 0x60400010); // Controlword
//    CAN_SendSDO_U32(nodeId, 0x1600, 0x02, 0x60FF0020); // Target velocity
//    CAN_SendSDO_U8(nodeId,  0x1600, 0x00, 2);
//    CAN_SendSDO_U32(nodeId, 0x1400, 0x01, 0x00000200);
//}
//
//void CAN_RemapTPDO1_Velocity(uint8_t nodeId) {
//    CAN_SendSDO_U32(nodeId, 0x1800, 0x01, 0x80000200);
//    CAN_SendSDO_U8(nodeId,  0x1A00, 0x00, 0);
//    CAN_SendSDO_U32(nodeId, 0x1A00, 0x01, 0x606C0020); // Actual velocity
//    CAN_SendSDO_U8(nodeId,  0x1A00, 0x00, 1);
//    CAN_SendSDO_U32(nodeId, 0x1800, 0x01, 0x00000200);
//}
//
///* === Enable sequence === */
//void CAN_SendEnableSequence(uint8_t nodeId) {
//    CAN_SendSDO_U16(nodeId, 0x6040, 0x00, 0x0080); // Fault reset
//    HAL_Delay(10);
//    CAN_SendSDO_U16(nodeId, 0x6040, 0x00, 0x06);   // Shutdown
//    HAL_Delay(10);
//    CAN_SendSDO_U16(nodeId, 0x6040, 0x00, 0x07);   // Switch on
//    HAL_Delay(10);
//    CAN_SendSDO_U16(nodeId, 0x6040, 0x00, 0x0F);   // Enable operation
//    HAL_Delay(10);
//}
//
///* === Đặt mode drive === */
//void CAN_SetDriveMode(uint8_t mode, uint8_t nodeId) {
//    // Gửi NMT Start
//    CAN_TxHeaderTypeDef tx;
//    uint8_t data[8];
//    tx.StdId = 0x000;
//    tx.IDE = CAN_ID_STD;
//    tx.RTR = CAN_RTR_DATA;
//    tx.DLC = 2;
//    data[0] = 0x01;
//    data[1] = nodeId;
//    HAL_CAN_AddTxMessage(&hcan2, &tx, data, &txMailbox);
//
//    HAL_Delay(100);
//    // Gửi SDO set mode
//    uint8_t mode_vel[] = {0x2F, 0x60, 0x60, 0x00, mode, 0, 0, 0};
//    TxHeader.StdId = 0x600 + nodeId;
//    TxHeader.DLC = 8;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, mode_vel, &txMailbox);
//
//    HAL_Delay(50);
//    CAN_SendEnableSequence(nodeId);
//}
