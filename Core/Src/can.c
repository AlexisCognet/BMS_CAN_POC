/*
 *  can.c
 *
 *  Created on: 17 févr. 2026
 *  Author: alexis.cognet
 */

#include "can.h"
#include "logger.h"
#include "main.h"
#include "stm32u5xx.h"
#include "stm32u5xx_hal_def.h"
#include "utils.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

bool sendCANMessage(uint8_t TxData[1], enum MESSAGE_TYPE type) {
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
  // Using the FD CAN standard
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;

  // Not using TxEvent and MessageMarker.
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  HAL_StatusTypeDef status;
  if ((status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)) !=
      HAL_OK) {
    LOG(WARN, "Can't send FDCAN message : HAL_status=%d.\r\n", status);
    return false;
  } else {
    if (type == ALERT_WARNING)
      LOG(INFO, "Sent FD CAN message, type=ALERT_WARNING, code = %d.",
          TxData[0]);
    else if (type == ALERT_CRITICAL)
      LOG(INFO, "Sent FD CAN message, type=ALERT_CRITICAL, code = %d.",
          TxData[0]);
  }
  return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {

  LOG(INFO, "FDCAN Callback triggered !");
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

    /* Retreive Rx messages from RX FIFO0 */
    HAL_StatusTypeDef status;
    if ((status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader,
                                         RxData)) != HAL_OK) {
      /* Reception Error */
      LOG(WARN,
          "Failure when trying to retrieve CAN message from FIFO : "
          "HAL_Status=%d.",
          status);
    } else {

      if (RxHeader.Identifier == 0x100) {
        float voltage, intensity;
        memcpy(&voltage, &RxData[4], sizeof(float));
        memcpy(&intensity, &RxData[0], sizeof(float));
        LOG(INFO,
            "Received CAN message : ID=%x | Voltage=%.2fV | Intensity=%.2fA.",
            RxHeader.Identifier, voltage, intensity);
      }

      else if (RxHeader.Identifier == 0x200) {
        float temperature;
        memcpy(&temperature, &RxData[0], sizeof(float));
        LOG(INFO, "Received CAN message : ID=%x | Temperature=%.2f°C.",
            RxHeader.Identifier, temperature);
      }
    }
  }
}