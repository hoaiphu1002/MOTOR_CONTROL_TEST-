/////*
//// * can_tranceive.h
//// *
//// *  Created on: Aug 30, 2025
//// *      Author: TRƯƠNG VŨ HOÀI PHÚ
//// */
////
//#ifndef CAN_TRANSCEIVE_H
//#define CAN_TRANSCEIVE_H
//
//#include "stm32f4xx_hal.h"   // hoặc HAL tương ứng
//#include <stdbool.h>
//#include <stdint.h>
//
//extern CAN_HandleTypeDef hcan2;
//
///* === Hàm khởi tạo CAN === */
//void CAN_Init(void);
//
///* === Hàm gửi SDO === */
//void CAN_SendSDO_U8(uint8_t nodeId, uint16_t index, uint8_t subidx, uint8_t value);
//void CAN_SendSDO_U16(uint8_t nodeId, uint16_t index, uint8_t subidx, uint16_t value);
//void CAN_SendSDO_U32(uint8_t nodeId, uint16_t index, uint8_t subidx, uint32_t value);
//
///* === Hàm gửi PDO & SYNC === */
//void CAN_SendSync(void);
//void CAN_SendVelocityRPDO(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel);
//
///* === Hàm remap PDO === */
//void CAN_RemapRPDO1_Velocity(uint8_t nodeId);
//void CAN_RemapTPDO1_Velocity(uint8_t nodeId);
//
///* === Chuỗi enable servo === */
//void CAN_SendEnableSequence(uint8_t nodeId);
//
///* === Đặt mode vận hành === */
//void CAN_SetDriveMode(uint8_t mode, uint8_t nodeId);
//
//#endif
//
////
////
////#endif
////
////
