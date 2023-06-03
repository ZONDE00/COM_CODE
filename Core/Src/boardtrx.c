/*
 * boardtrx.c
 *
 *  Created on: 7 Mar 2023
 *      Author: Kristers
 *
 *      The point of this library is to easily and reliably, asynchronously
 *      share data/variables between two devices.
 *
 *      Library handles all the things necessary for communication, and
 *      tries to handle any errors if any happens to be.
 *
 *      UART CIRCULAR DMA IS USED
 *
 *      TODO: multi device support
 *      TODO: cmd packet nck response on crc errors
 *
 */

#include <string.h>
#include "boardtrx.h"

#define BOARDTRX_START_BYTE         0x45
#define BOARDTRX_REQUEST_BYTE       0x80
#define BOARDTRX_CMD_BYTE           0x40
#define BOARDTRX_HARD_MAX           0x40
#define BOARDTRX_RES_ACK            0xAA
#define BOARDTRX_RES_NCK            0x55
#define BOARDTRX_RES_REP            0x5A
#define BOARDTRX_CMD_HEADER_SIZE    0x04
#define BOARDTRX_DATA_HEADER_SIZE   0x03

BOARDTRX_Status brdState = { 0 };
BOARDTRX_Handle *brdHandle;

uint8_t brdInitialised = 0; // initialisation check

uint8_t brdUartReceived = 0; // set when data packet is received
uint8_t brdUartRx = 0;      // tmp buf for received uart data
uint8_t brdUartRxBuf[256];  // not so tmp buf for received uart data
uint8_t brdUartTxBuf[256];  // buf for sending
uint32_t brdUartRxInd = 257;
uint32_t brdUartRxSize = 0;
uint16_t brdUartCrc = 0;

uint8_t brdRequestQueDataTx[BOARDTRX_DATA_COUNT];
uint8_t brdRequestQueDataRx[BOARDTRX_DATA_COUNT];
uint8_t brdRequestQueCmdTx[BOARDTRX_CMD_COUNT];
uint8_t brdRequestQueCmdRx[BOARDTRX_CMD_COUNT];

// Does sanity checks and makes sure all is somewhat ok
BOARDTRX_Status BOARDTRX_Init(BOARDTRX_Handle *handle) {
    if (handle == 0x00) {
        brdState.errorTRX = BOARDTRX_INT;
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    brdHandle = handle;

    if (brdHandle->uart == 0x00) {
        brdState.errorTRX = BOARDTRX_INT;
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    if (brdHandle->countDataRx > 0) {
        uint8_t tmpLen = brdHandle->countDataRx;
        if (brdHandle->dataRx == 0x00 || tmpLen > BOARDTRX_DATA_COUNT || tmpLen > BOARDTRX_HARD_MAX) {
            brdState.status = BOARDTRX_ERROR;
            brdState.errorTRX = BOARDTRX_INT;
            brdState.isNewError = 1;
            return brdState;
        }
    }

    if (brdHandle->countDataTx > 0) {
        uint8_t tmpLen = brdHandle->countDataTx;
        if (brdHandle->dataTx == 0x00 || tmpLen > BOARDTRX_DATA_COUNT || tmpLen > BOARDTRX_HARD_MAX) {
            brdState.status = BOARDTRX_ERROR;
            brdState.errorTRX = BOARDTRX_INT;
            brdState.isNewError = 1;
            return brdState;
        }
    }

    if (brdHandle->countCmdRx > 0) {
        uint8_t tmpLen = brdHandle->countCmdRx;
        if (brdHandle->cmdRx == 0x00 || tmpLen > BOARDTRX_CMD_COUNT || tmpLen > BOARDTRX_HARD_MAX) {
            brdState.status = BOARDTRX_ERROR;
            brdState.errorTRX = BOARDTRX_INT;
            brdState.isNewError = 1;
            return brdState;
        }
    }

    if (brdHandle->countCmdTx > 0) {
        uint8_t tmpLen = brdHandle->countCmdTx;
        if (brdHandle->cmdTx == 0x00 || tmpLen > BOARDTRX_CMD_COUNT || tmpLen > BOARDTRX_HARD_MAX) {
            brdState.status = BOARDTRX_ERROR;
            brdState.errorTRX = BOARDTRX_INT;
            brdState.isNewError = 1;
            return brdState;
        }
    }

    if (brdHandle->countDataTx == 0 && brdHandle->countDataRx == 0 && brdHandle->countCmdRx == 0 && brdHandle->countCmdTx == 0) {
        brdState.status = BOARDTRX_ERROR;
        brdState.errorTRX = BOARDTRX_INT;
        brdState.isNewError = 1;
        return brdState;
    }

    HAL_StatusTypeDef tmp = HAL_UART_Receive_DMA(brdHandle->uart, &brdUartRx, 1);

    if (tmp != HAL_OK) {
        brdState.status = BOARDTRX_ERROR;
        brdState.errorTRX = BOARDTRX_INT;
        brdState.isNewError = 1;
        return brdState;
    }

    memset(brdRequestQueDataTx, 0, BOARDTRX_DATA_COUNT);
    memset(brdRequestQueDataRx, 0, BOARDTRX_DATA_COUNT);
    memset(brdRequestQueCmdTx, 0, BOARDTRX_CMD_COUNT);
    memset(brdRequestQueCmdRx, 0, BOARDTRX_CMD_COUNT);

    brdState.status = BOARDTRX_OK;
    brdInitialised = 1;
    return brdState;
}

/*
 * @brief adds cmd to que for processing
 * @param nr of cmd to add
 * @param id of the cmd
 * @param setId whether or not to set id manually, 1 - manually 0 - automatically
 */
BOARDTRX_Status BOARDTRX_CmdSend(uint8_t nr, uint8_t id, uint8_t setId) {
    if (!brdInitialised) {
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    if (nr >= brdHandle->countCmdTx) {
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    if(brdHandle->cmdTx[nr]->wasSent){
        brdHandle->cmdTx[nr]->wasSent = 0;
        brdHandle->cmdTx[nr]->id++;
    }

    if(brdHandle->cmdTx[nr]->id == 0){
        brdHandle->cmdTx[nr]->id++;
    }

    if(setId){
        brdHandle->cmdTx[nr]->id = id;
    }

    return BOARDTRX_AddToQue(BOARDTRX_CMD_TX, nr);
}

/*
 * @Brief returns current TX CMD status
 * @param nr Nr of the CMD to return status of
 * @param clearStatus if set then on BOARDTRX_CMD_OK will clear it to BOARDTRX_CMD_DEF
 * @retval status of CMD
 */
BOARDTRX_CMD_Status BOARDTRX_CmdGetStatus(uint8_t nr) {
    if (nr < brdHandle->countCmdTx) {
        return brdHandle->cmdTx[nr]->status;
    }
    return BOARDTRX_CMD_DEF;
}

/*
 * @Brief returns current TX CMD status
 * @param nr Nr of the CMD to return status of
 * @retval status of CMD
 */
void BOARDTRX_CmdClearStatus(uint8_t nr) {
    if (nr < brdHandle->countCmdTx) {
        brdHandle->cmdTx[nr]->status = BOARDTRX_CMD_DEF;
    }
}

BOARDTRX_Status BOARDTRX_DataRequestAll() {
    if (!brdInitialised) {
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    for (uint8_t i = 0; i < brdHandle->countDataTx; i++) {
        BOARDTRX_AddToQue(BOARDTRX_DATA_TX, i);
    }

    return brdState;
}

BOARDTRX_Status BOARDTRX_DataRequest(uint8_t nr) {
    if (!brdInitialised) {
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    if (nr >= brdHandle->countDataTx) {
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    return BOARDTRX_AddToQue(BOARDTRX_DATA_TX, nr);
}

BOARDTRX_Status BOARDTRX_Loop() {
    if (!brdInitialised) {
        brdState.status = BOARDTRX_ERROR;
        brdState.isNewError = 1;
        return brdState;
    }

    // process received data
    if (brdUartReceived) {
        return BOARDTRX_ProcessRx();
    }

    // send data if any requested
    uint8_t tmp = BOARDTRX_GetFromQue(BOARDTRX_DATA_RX, 0);
    if (tmp != 255) {
        BOARDTRX_ClearFromQue(BOARDTRX_DATA_RX, tmp);
        return BOARDTRX_UART_Send(tmp, brdHandle->dataRx[tmp]->data, brdHandle->dataRx[tmp]->size);
    }

    // request data if any needed
    tmp = BOARDTRX_GetFromQue(BOARDTRX_DATA_TX, 0);
    while (1) {
        // dont return if did not do anything TODO
        if (tmp != 255) {
            return BOARDTRX_ProcessTx(BOARDTRX_DATA_TX, tmp);
        } else {
            break;
        }

        // get new request if already waiting current one
        tmp = BOARDTRX_GetFromQue(BOARDTRX_DATA_TX, tmp + 1);
    }

    // send cmd response if any requested
    tmp = BOARDTRX_GetFromQue(BOARDTRX_CMD_RX, 0);
    if (tmp != 255) {

        uint8_t response = BOARDTRX_RES_ACK;

        if (brdHandle->cmdRx[tmp]->status != BOARDTRX_CMD_NEW && brdHandle->cmdRx[tmp]->status != BOARDTRX_CMD_REP) {
            response = BOARDTRX_RES_NCK;
        }

        if(brdHandle->cmdRx[tmp]->status == BOARDTRX_CMD_REP){
            response = BOARDTRX_RES_REP;
        }

        // check wheter or not cmd is new, if is then callback
        if (brdHandle->cmdRx[tmp]->status == BOARDTRX_CMD_NEW) {
            if (brdHandle->cmdRx[tmp]->CMD_CB != 0x00) {
                brdHandle->cmdRx[tmp]->CMD_CB();
            }
        }

        BOARDTRX_ClearFromQue(BOARDTRX_CMD_RX, tmp);
        tmp |= BOARDTRX_CMD_BYTE;
        return BOARDTRX_UART_Send(tmp, &response, 1);
    }

    // send CMDs
    tmp = BOARDTRX_GetFromQue(BOARDTRX_CMD_TX, 0);
    while (1) {

        if (tmp != 255) {
            return BOARDTRX_ProcessTx(BOARDTRX_CMD_TX, tmp);
        } else {
            break;
        }

        // get new request if already waiting current one
        tmp = BOARDTRX_GetFromQue(BOARDTRX_CMD_TX, tmp + 1);
    }

    return brdState;
}

BOARDTRX_Status BOARDTRX_ProcessTx(BOARDTRX_TRX_Target target, uint8_t nr) {
    uint32_t timeout;
    uint8_t retries;
    uint8_t tmp = nr;

    // is new transfer ?
    if (target == BOARDTRX_DATA_TX) {
        timeout = brdHandle->dataTx[nr]->timeout;
    } else {
        timeout = brdHandle->cmdTx[nr]->timeout;
    }

    if (timeout == 0) {
        if (target == BOARDTRX_DATA_TX) {
            timeout = brdHandle->dataTx[nr]->retries = 0;
        } else {
            timeout = brdHandle->cmdTx[nr]->retries = 0;
        }

        timeout = HAL_GetTick() + brdHandle->txTimeout;
        if (target == BOARDTRX_DATA_TX) {
            timeout = brdHandle->dataTx[nr]->timeout = timeout;
        } else {
            timeout = brdHandle->cmdTx[nr]->timeout = timeout;
        }

        tmp += BOARDTRX_REQUEST_BYTE;
        if (target == BOARDTRX_CMD_TX) {
            tmp += BOARDTRX_CMD_BYTE;
            return BOARDTRX_UART_Send(tmp, brdHandle->cmdTx[nr]->data, brdHandle->cmdTx[nr]->size);
        }
        return BOARDTRX_UART_Send(tmp, &tmp, 0);

        // or was requested already ?
    } else if (brdHandle->dataTx[tmp]->timeout < HAL_GetTick()) {
        if (target == BOARDTRX_DATA_TX) {
            timeout = brdHandle->dataTx[nr]->retries++;
            retries = brdHandle->dataTx[nr]->retries;
        } else {
            timeout = brdHandle->cmdTx[nr]->retries++;
            retries = brdHandle->cmdTx[nr]->retries;
        }
        if (retries >= brdHandle->txRetries) {
            brdState.isNewError = 1;
            brdState.errorsTimeout += 1;
            brdState.errorDataNr = tmp;
            brdState.status = BOARDTRX_TIMEOUT;
            brdState.errorTRX = BOARDTRX_RX;
            // timed out clear request
            brdHandle->dataTx[tmp]->timeout = 0;
            if (target == BOARDTRX_DATA_TX) {
                BOARDTRX_ClearFromQue(BOARDTRX_DATA_TX, tmp);
                brdHandle->dataTx[tmp]->status = BOARDTRX_CMD_TIM;
            } else {
                BOARDTRX_ClearFromQue(BOARDTRX_CMD_TX, tmp);
                brdHandle->cmdTx[tmp]->status = BOARDTRX_CMD_TIM;
            }

            return brdState;
        } else {
            // send new request to target
            timeout = HAL_GetTick() + brdHandle->txTimeout;
            if (target == BOARDTRX_DATA_TX) {
                timeout = brdHandle->dataTx[nr]->timeout = timeout;
            } else {
                timeout = brdHandle->cmdTx[nr]->timeout = timeout;
            }

            tmp += BOARDTRX_REQUEST_BYTE;
            if (target == BOARDTRX_CMD_TX) {
                tmp += BOARDTRX_CMD_BYTE;
                return BOARDTRX_UART_Send(tmp, brdHandle->cmdTx[nr]->data, brdHandle->cmdTx[nr]->size);
            }
            return BOARDTRX_UART_Send(tmp, &tmp, 0);
        }
    }

    brdState.status = BOARDTRX_OK;
    return brdState;
}

BOARDTRX_Status BOARDTRX_ProcessRx() {
    brdUartReceived = 0;

    // sanity check
    if (brdUartRxSize == 0) {
        brdState.isNewError = 1;
        brdState.errorsRX += 1;
        brdState.errorDataNr = 255;
        brdState.status = BOARDTRX_ERROR_CMD;
        brdState.errorTRX = BOARDTRX_RX;
        return brdState;
    }

    // check crc
    if (BOARDTRX_CheckCRC(brdUartRxBuf, brdUartRxSize) == 0) {
        brdState.isNewError = 1;
        brdState.errorsCRC += 1;
        brdState.errorDataNr = 255;
        brdState.status = BOARDTRX_ERROR_CRC;
        brdState.errorTRX = BOARDTRX_RX;
        return brdState;
    }

    uint8_t rqCmd = brdUartRxBuf[1] & (~BOARDTRX_REQUEST_BYTE);
    rqCmd = rqCmd & (~BOARDTRX_CMD_BYTE);

    // check if requesting data or sending to us
    if (brdUartRxBuf[1] & BOARDTRX_REQUEST_BYTE) {
        // CMD request
        if (brdUartRxBuf[1] & BOARDTRX_CMD_BYTE) {
            uint8_t id = brdUartRxBuf[2];
            // is in bounds ?
            if (rqCmd >= brdHandle->countCmdRx) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = brdUartRxBuf[1];
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                brdHandle->cmdRx[rqCmd]->status = BOARDTRX_CMD_NCK;
                return brdState;
            }

            // is size correct ?
            if (brdHandle->cmdRx[rqCmd]->size != (brdUartRxSize - BOARDTRX_CMD_HEADER_SIZE)) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = rqCmd;
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                brdHandle->cmdRx[rqCmd]->status = BOARDTRX_CMD_NCK;
                return brdState;
            }

            if (id != brdHandle->cmdRx[rqCmd]->id) {
                brdHandle->cmdRx[rqCmd]->status = BOARDTRX_CMD_NEW;
                brdHandle->cmdRx[rqCmd]->id = id;
                memcpy(brdHandle->cmdRx[rqCmd]->data, &brdUartRxBuf[3], brdHandle->cmdRx[rqCmd]->size);
            } else {
                brdHandle->cmdRx[rqCmd]->status = BOARDTRX_CMD_REP;
            }
            return BOARDTRX_AddToQue(BOARDTRX_CMD_RX, rqCmd);
        } else {
            uint8_t rqCmd = brdUartRxBuf[1] & (~BOARDTRX_REQUEST_BYTE);
            // is in bounds ?
            if (rqCmd >= brdHandle->countDataRx) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = brdUartRxBuf[1];
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            }

            brdHandle->dataRx[rqCmd]->wasRequested = 1;
            return BOARDTRX_AddToQue(BOARDTRX_DATA_RX, rqCmd);
        }
    } else {
        if (brdUartRxBuf[1] & BOARDTRX_CMD_BYTE) {
            // processing CMD response, should have ACK or NCK in response
            uint8_t id = brdUartRxBuf[2];
            uint8_t response = brdUartRxBuf[3];

            BOARDTRX_ClearFromQue(BOARDTRX_CMD_TX, rqCmd);

            // id check, should match
            if (id != brdHandle->cmdTx[rqCmd]->id) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = brdUartRxBuf[1];
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            }

            // response check
            if (response == BOARDTRX_RES_ACK) {
                brdHandle->cmdTx[rqCmd]->status = BOARDTRX_CMD_OK;
                brdHandle->cmdTx[rqCmd]->wasSent = 1;
                brdState.status = BOARDTRX_OK;
                return brdState;
            } else if (response == BOARDTRX_RES_NCK) {
                brdHandle->cmdTx[rqCmd]->status = BOARDTRX_CMD_NOK;
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = brdUartRxBuf[1];
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            } else if (response == BOARDTRX_RES_REP) {
                brdHandle->cmdTx[rqCmd]->status = BOARDTRX_CMD_REP;
                brdHandle->cmdTx[rqCmd]->wasSent = 1;
                brdState.status = BOARDTRX_OK;
                return brdState;
            } else {
                brdHandle->cmdTx[rqCmd]->status = BOARDTRX_CMD_NOK;
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = brdUartRxBuf[1];
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            }

//            // is in bounds ?
//            if (rqCmd >= brdHandle->countCmdRx) {
//                brdState.isNewError = 1;
//                brdState.errorsRX += 1;
//                brdState.errorDataNr = rqCmd;
//                brdState.status = BOARDTRX_ERROR_CMD;
//                brdState.errorTRX = BOARDTRX_RX;
//                return brdState;
//            }
//
//            // is size correct ?
//            if (brdHandle->cmdRx[rqCmd]->size != (brdUartRxSize - 3)) {
//                brdState.isNewError = 1;
//                brdState.errorsRX += 1;
//                brdState.errorDataNr = rqCmd;
//                brdState.status = BOARDTRX_ERROR_CMD;
//                brdState.errorTRX = BOARDTRX_RX;
//                return brdState;
//            }
//
//            // copy received data to the memory pointed by struct
//            memcpy(brdHandle->cmdRx[rqCmd]->data, &brdUartRxBuf[2], brdHandle->cmdRx[rqCmd]->size);
//            brdHandle->cmdRx[rqCmd]->isNew = 1;
//            brdHandle->cmdRx[rqCmd]->timeout = 0;
//            brdHandle->cmdRx[rqCmd]->retries = 0;
//
//            // is request even active ?
//            if (brdRequestQueDataRx[rqCmd] == 0) {
//                brdState.isNewError = 1;
//                brdState.errorsRX += 1;
//                brdState.errorDataNr = rqCmd;
//                brdState.status = BOARDTRX_MISS;
//                brdState.errorTRX = BOARDTRX_RX;
//                return brdState;
//            } else {
//                BOARDTRX_ClearFromQue(BOARDTRX_CMD_RX, rqCmd);
//            }
//
//            brdState.status = BOARDTRX_OK;
//            return brdState;
        } else {
            uint8_t rqCmd = brdUartRxBuf[1];

            // is in bounds ?
            if (rqCmd >= brdHandle->countDataTx) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = rqCmd;
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            }

            // is size correct ?
            if (brdHandle->dataTx[rqCmd]->size != (brdUartRxSize - BOARDTRX_DATA_HEADER_SIZE)) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = rqCmd;
                brdState.status = BOARDTRX_ERROR_CMD;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            }

            // copy received data to the memory pointed by struct
            memcpy(brdHandle->dataTx[rqCmd]->data, &brdUartRxBuf[2], brdHandle->dataTx[rqCmd]->size);
            brdHandle->dataTx[rqCmd]->status = BOARDTRX_CMD_NEW;
            brdHandle->dataTx[rqCmd]->timeout = 0;
            brdHandle->dataTx[rqCmd]->retries = 0;

            // is request even active ?
            if (brdRequestQueDataTx[rqCmd] == 0) {
                brdState.isNewError = 1;
                brdState.errorsRX += 1;
                brdState.errorDataNr = rqCmd;
                brdState.status = BOARDTRX_MISS;
                brdState.errorTRX = BOARDTRX_RX;
                return brdState;
            } else {
                BOARDTRX_ClearFromQue(BOARDTRX_DATA_TX, rqCmd);
            }

            brdState.status = BOARDTRX_OK;
            return brdState;
        }
    }
}

uint8_t BOARDTRX_CheckCRC(uint8_t *data, uint8_t size) {
    uint16_t tmpCrc = BOARDTRX_CalcCRC(data + 1, size - 2);
    uint16_t crc = (data[size - 1] << 8) + data[size];
    return crc == tmpCrc;
}

// internal functions
uint16_t BOARDTRX_CalcCRC(uint8_t *data, uint8_t size) {
    uint8_t crc_a = 0;
    uint8_t crc_b = 0;

    for (uint8_t i = 0; i < size; i++) {
        crc_a ^= data[i];
        crc_b ^= crc_a;
    }

    return (crc_a << 8) + crc_b;
}

/*
 * @brief Ads requested dataNr, from target, to queue, if already was requested then ads to error status
 *        but still keeps request active
 * @retVal status of system
 */
BOARDTRX_Status BOARDTRX_AddToQue(BOARDTRX_TRX_Target target, uint8_t dataNr) {
    uint8_t tmp = 0;

    if (target == BOARDTRX_DATA_RX) {
        tmp = brdRequestQueDataRx[dataNr];
        if (!tmp) {
            brdRequestQueDataRx[dataNr] = 1;
        }

        brdHandle->dataRx[dataNr]->wasRequested = 1;

    } else if (target == BOARDTRX_DATA_TX) {
        tmp = brdRequestQueDataTx[dataNr];
        if (!tmp) {
            brdRequestQueDataTx[dataNr] = 1;
        }

        brdHandle->dataTx[dataNr]->retries = 1;
        brdHandle->dataTx[dataNr]->status = BOARDTRX_CMD_SEN;

    } else if (target == BOARDTRX_CMD_RX) {
        tmp = brdRequestQueCmdRx[dataNr];
        if (!tmp) {
            brdRequestQueCmdRx[dataNr] = 1;
        }

    } else if (target == BOARDTRX_CMD_TX) {
        tmp = brdRequestQueCmdTx[dataNr];
        if (!tmp) {
            brdRequestQueCmdTx[dataNr] = 1;
        }

        // reset retry parameter
        brdHandle->cmdTx[dataNr]->retries = 0;
        brdHandle->cmdTx[dataNr]->status = BOARDTRX_CMD_SEN;
    }

    if (tmp) {
        brdState.isNewError = 1;
        brdState.errorsMissed += 1;
        brdState.errorDataNr = dataNr;
        brdState.status = BOARDTRX_MISS;
        brdState.errorTRX = target;
    } else {
        brdState.status = BOARDTRX_OK;
    }

    return brdState;
}

/*
 * @brief Checks if there is any active requests if is then returns its nr
 * @param target target queue to get active request from
 * @param startInd start index, allows to get next active request
 * @retval active request Nr or 255 if none
 */
uint8_t BOARDTRX_GetFromQue(BOARDTRX_TRX_Target target, uint8_t startInd) {
    uint8_t tmp;
    uint8_t max;

    if (target == BOARDTRX_DATA_RX) {
        max = brdHandle->countDataRx;
    } else if (target == BOARDTRX_DATA_TX) {
        max = brdHandle->countDataTx;
    } else if (target == BOARDTRX_CMD_RX) {
        max = brdHandle->countCmdRx;
    } else if (target == BOARDTRX_CMD_TX) {
        max = brdHandle->countCmdTx;
    }

    for (uint8_t i = startInd; i < max; i++) {
        if (target == BOARDTRX_DATA_RX) {
            tmp = brdRequestQueDataRx[i];
        } else if (target == BOARDTRX_DATA_TX) {
            tmp = brdRequestQueDataTx[i];
        } else if (target == BOARDTRX_CMD_RX) {
            tmp = brdRequestQueCmdRx[i];
        } else if (target == BOARDTRX_CMD_TX) {
            tmp = brdRequestQueCmdTx[i];
        }

        if (tmp) {
            return i;
        }
    }

    return 255;
}

/*
 * @brief Clears request from que
 *
 */
void BOARDTRX_ClearFromQue(BOARDTRX_TRX_Target target, uint8_t dataNr) {
    if (target == BOARDTRX_DATA_RX) {
        brdRequestQueDataRx[dataNr] = 0;
    } else if (target == BOARDTRX_DATA_TX) {
        brdRequestQueDataTx[dataNr] = 0;
    } else if (target == BOARDTRX_CMD_RX) {
        brdRequestQueCmdRx[dataNr] = 0;
    } else if (target == BOARDTRX_CMD_TX) {
        brdRequestQueCmdTx[dataNr] = 0;
    }
}

/*
 * @brief sends data to target devices, adds necessary data packet information
 * @param nr data number
 * @param data pointer to data to send
 * @param size size of data to send
 */
BOARDTRX_Status BOARDTRX_UART_Send(uint8_t nr, uint8_t *data, uint8_t size) {
    uint8_t ptr = 0;

    // set start byte
    brdUartTxBuf[ptr] = BOARDTRX_START_BYTE;
    ptr++;

    // set length of data packet, 1 byte for nr + data size + 2 bytes crc
    uint8_t totalSize = 1 + size + 2;
    if (nr & BOARDTRX_CMD_BYTE) {
        totalSize++; // id byte
    }
    brdUartTxBuf[ptr] = totalSize;
    ptr++;

    // set data Nr
    brdUartTxBuf[ptr] = nr;
    ptr++;

    // if cmd byte then add id
    if (nr & BOARDTRX_CMD_BYTE) {
        uint8_t ind = nr & (~BOARDTRX_CMD_BYTE);
        ind = ind & (~BOARDTRX_REQUEST_BYTE);
        if (nr & BOARDTRX_REQUEST_BYTE) {
            brdUartTxBuf[ptr] = brdHandle->cmdTx[ind]->id;
        } else {
            brdUartTxBuf[ptr] = brdHandle->cmdRx[ind]->id;
        }
        ptr++;
    }

    // copy data if any
    if (size > 0) {
        memcpy(&brdUartTxBuf[ptr], data, size);
        ptr += size;
    }

    // calculate crc for Nr + data
    uint16_t crc = BOARDTRX_CalcCRC(&brdUartTxBuf[2], totalSize - 2);

    // copy crc
    brdUartTxBuf[ptr] = crc >> 8;
    ptr++;
    brdUartTxBuf[ptr] = crc & 0xff;
    ptr++;

    HAL_StatusTypeDef ret = HAL_UART_Transmit(brdHandle->uart, brdUartTxBuf, ptr, 1000);

    if (ret != HAL_OK) {
        brdState.isNewError = 1;
        brdState.errorsTX += 1;
        brdState.errorDataNr = nr;
        brdState.status = BOARDTRX_ERROR_TX;
        brdState.errorTRX = BOARDTRX_TX;
        return brdState;
    }

    brdState.status = BOARDTRX_OK;
    return brdState;
}

/*
 * if uart reaceive error then restart dma
 *
 */
void BOARDTRX_UART_RXERROR_CB() {
    HAL_UART_Receive_DMA(brdHandle->uart, &brdUartRx, 1);
}

/*
 * if data packet then store inside bigger buffer
 * when whole packet is received mark as such
 *
 */
void BOARDTRX_UART_RX_CB() {
    if (brdUartRxInd >= 256) {
        if (brdUartRx == BOARDTRX_START_BYTE) {
            brdUartRxInd = 0;
            brdUartRxSize = 0;
        }
    } else {
        brdUartRxBuf[brdUartRxInd] = brdUartRx;

        if (brdUartRxInd == 0) {    // first byte will be size of packet
            brdUartRxSize = brdUartRx;
        }

        if (brdUartRxInd == brdUartRxSize) {    // packet is received
            brdUartReceived = 1;
            brdUartRxInd = 257;
        }

        brdUartRxInd++;
    }
}
