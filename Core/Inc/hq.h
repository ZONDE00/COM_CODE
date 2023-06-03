/*
 * hq.h
 *
 *  Created on: Dec 17, 2022
 *      Author: Kristers
 */

#ifndef INC_HQ_H_
#define INC_HQ_H_

#include "main.h"
#include "si4463.h"

typedef enum {
    GS_COM_NAN = 0,         // not valid
    GS_COM_REG,
    GS_COM_VAR,
    GS_COM_VAR_RES,
    GS_COM_CMD,
    GS_COM_ACK = 0xAA,
    GS_COM_NCK,
    GS_COM_UNK,
    GS_COM_CRCE,
} GS_COM_CMDS;

typedef enum {
    // variables from this system
    HQ_VARS_RX_LONGITUDE,
    HQ_VARS_RX_LATITUDE,
    HQ_VARS_RX_ALTITUDE,
    HQ_VARS_RX_SPEED,
    HQ_VARS_RX_TIME,
    HQ_VARS_RX_MCU_TMP,
    HQ_VARS_RX_CMD_OK,
    HQ_VARS_RX_CMD_NCK,
    HQ_VARS_RX_CMD_CRC,
    HQ_VARS_RX_CMD_TMO,

    HQ_VARS_LOCAL_LIMIT,
    // must be requested from MCU
    HQ_VARS_TX_BAT_VOLT,
    HQ_VARS_TX_4V_CUR,
    HQ_VARS_TX_3V3_CUR,
    HQ_VARS_TX_3V3,
    HQ_VARS_TX_TMP_MCU,
    HQ_VARS_TX_TMP_IN ,
    HQ_VARS_TX_TMP_OUT,
    HQ_VARS_TX_TMP_BME,
    HQ_VARS_TX_PRESSURE,
    HQ_VARS_TX_HUMID,
    HQ_VARS_TX_AIRQ,
    HQ_VARS_TX_AIR_RES,
    HQ_VARS_TX_SEN_STATUS,

    HQ_VARS_TOTAL_SIZE  // cant be larger than 32

} HQ_VARS_SI;

typedef enum {
    HQ_CMD_NCK,
    HQ_CMD_ACK,
} HQ_Commands;

typedef enum {
    HQ_CMD_RX_TIME,
    HQ_CMD_RX_LONGITUDE,
    HQ_CMD_RX_LATITUDE,
    HQ_CMD_RX_ALTITUDE,
    HQ_CMD_RX_SPEED,
    HQ_CMD_RX_MCU_TMP,
    HQ_CMD_RX_CMD_OK,
    HQ_CMD_RX_CMD_NCK,
    HQ_CMD_RX_CMD_CRC,
    HQ_CMD_RX_CMD_TMO,
    HQ_CMD_RX_END
} HQ_Commands_RX;

typedef enum {
    HQ_CMD_TX_BAT_VOLT,
    HQ_CMD_TX_4V_CUR,
    HQ_CMD_TX_3V3_CUR,
    HQ_CMD_TX_3V3,
    HQ_CMD_TX_TMP_MCU,
    HQ_CMD_TX_TMP_IN ,
    HQ_CMD_TX_TMP_OUT,
    HQ_CMD_TX_TMP_BME,
    HQ_CMD_TX_PRESSURE,
    HQ_CMD_TX_HUMID,
    HQ_CMD_TX_AIRQ,
    HQ_CMD_TX_AIR_RES,
    HQ_CMD_TX_SEN_STATUS,
    HQ_CMD_TX_END
} HQ_Commands_TX;

typedef enum {
    HQ_OK = 0x00U,
    HQ_ERROR = 0x01U,
    HQ_BUSY = 0x02U,
    HQ_TIMEOUT = 0x03U,
    HQ_ERROR_UART = 0x04U,
    HQ_ERROR_SI = 0x08U,
    HQ_ERROR_CRC = 0x10U,
    HQ_ERROR_CMD = 0x20U,
} HQ_StatusTypeDef;

typedef enum {
    HQ_CMD_STATUS_NAN = 0,
    HQ_CMD_STATUS_OK,
    HQ_CMD_STATUS_REP,
    // below are errors
    HQ_CMD_STATUS_TIMEOUT,
    HQ_CMD_STATUS_CRC,
    HQ_CMD_STATUS_SIZE,
    HQ_CMD_STATUS_NR,
    // unknown error
    HQ_CMD_STATUS_ERROR
} HQ_CmdStatusTypeDef;

typedef struct {
    SI4463_Handle *siHandle; // for communication with WASSUP
    UART_HandleTypeDef *uart; // for communication with MCU
} HQ_Handle;


HQ_StatusTypeDef HQ_Init(HQ_Handle *handle);

HQ_StatusTypeDef HQ_Loop();
HQ_StatusTypeDef HQ_SI_Process_Data();
HQ_StatusTypeDef HQ_SI_Send(uint8_t *data, uint8_t size);

void HQ_VAR_Process();
void HQ_CMD_Process();
uint8_t HQ_VAR_Send(uint8_t ind);
uint8_t HQ_CMD_Send(HQ_CmdStatusTypeDef status);

uint16_t HQ_CalcCRC(uint8_t *data, uint8_t size);

void HQ_SI_RX_CallBack();

#endif /* INC_HQ_H_ */
