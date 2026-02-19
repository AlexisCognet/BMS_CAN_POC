/*
 *  can.c
 *
 *  Created on: 17 févr. 2026
 *  Author: alexis.cognet
 */

#include "can.h"
#include "main.h"
#include "stm32u5xx_hal_fdcan.h"
#include "stm32u5xx_hal_uart.h"
#include <stdint.h>

bool SendMessage(FDCAN_HandleTypeDef hfdcan1, uint8_t TxData[1],
                 enum MESSAGE_TYPE type) {
  FDCAN_TxHeaderTypeDef TxHeader;
  // Critical alert ID is lower because it has a higher priority.
  if (type == ALERT_WARNING)
    TxHeader.Identifier = 0x50;
  else if (type == ALERT_CRITICAL)
    TxHeader.Identifier = 0x25;

  // Use Standard ID not Extended.
  TxHeader.IdType = FDCAN_STANDARD_ID;
  // Send a Data Frame.
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  // Indicate we are sending 1 byte.
  TxHeader.DataLength = FDCAN_DLC_BYTES_1;
  // Allow us to be notified if there is any error in transmission.
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // Using the same bitrate for both Arbitration and Data Fields
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  // Using the CAN standard
  TxHeader.FDFormat = FDCAN_FD_CAN;

  // Not using TxEvent and MessageMarker.
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef RxHeader;
  volatile uint8_t RxData[6];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) !=
        HAL_OK) {
      /* Reception Error */
      Error_Handler();
    }
  }
}