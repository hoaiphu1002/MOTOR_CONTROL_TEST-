///*
// * can_tranceive.h
// *
// *  Created on: Aug 30, 2025
// *      Author: TRƯƠNG VŨ HOÀI PHÚ
// */
//
//#ifndef __CAN_TRANCEIVE_H
//#define __CAN_TRANCEIVE_H
//
//#include "main.h"
//#include <stdbool.h>
//
//// === Biến trạng thái CAN ===
//extern volatile uint32_t can_rx_count;
//extern volatile uint32_t can_rx_flag;
//extern volatile int32_t currentVel1, currentVel2;
//extern volatile uint8_t node_booted[3];
//extern volatile uint8_t ready1, ready2;
//extern uint32_t up_vel1, up_vel2;
//
//// === API khởi tạo & enable ===
//void set_drive_mode(uint8_t mode, uint8_t nodeId);
//void send_enable_sequence(uint8_t nodeId);
//
//// === API PDO / SDO ===
//void send_sdo_write_u8(uint8_t nodeId, uint16_t index, uint8_t subidx, uint8_t value);
//void send_sdo_write_u16(uint8_t nodeId, uint16_t index, uint8_t subidx, uint16_t value);
//void send_sdo_write_u32(uint8_t nodeId, uint16_t index, uint8_t subidx, uint32_t value);
//void remap_rpdo1_for_velocity(uint8_t nodeId);
//void remap_tpdo1_velocity(uint8_t nodeId);
//void send_velocity_rpdo(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel);
//void update_vel(uint8_t node, int32_t velocity, bool toggle_cw, uint32_t accel, uint32_t decel);
//void request_actual_velocity(uint8_t nodeId);
//void request_statusword(uint8_t nodeId);
//
//// === API CAN utilities ===
//void send_sync_frame(void);
//void send_temp_to_usbcan(int32_t temperature);
//void send_vel_can(int32_t vel1, int32_t vel2);
//
//// === Debug ===
//void print_velocity_both_nodes(void);
//
//// === Callback ===
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void set_drive_mode(uint8_t mode, uint8_t node_id);
//void request_actual_velocity(uint8_t node_id);
//void send_vel_can(int32_t vel1, int32_t vel2);
//
//
//#endif
//
//
