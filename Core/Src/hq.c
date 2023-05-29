/*
 * hq.c
 *
 *  Created on: Dec 17, 2022
 *      Author: Kristers
 */

#include "hq.h"
#include "string.h"
#include "stdio.h"
#include "gps.h"
#include "boardtrx.h"

// TODO checksums might not be needed as SI4463 already sends 16 bit CRC
// TODO check max tick time, multiply by 10 and implement WDT
// TODO get time from GPS

HQ_Handle *hqHandle;

// GENERAL STUFF
uint8_t initialised = 0;

// UART STUFF
#define HQ_UART_START_BYTE      0x45
uint8_t uartRxBuf[64];
uint8_t uartRxLen = 255;
uint8_t uartRxToGet = 0;
uint8_t uartGotFrame = 0;
uint8_t uartMiniBuf;
uint8_t uartRxToReceive = 0;
uint8_t uartWaitResponse = 0;

// FSK STUFF

#define HQ_FSK_MAX_DATA_SIZE    4

SI4463_StatusTypeDef status;
uint8_t isSI4463inRx = 0;

uint8_t rxRawBuf[8];
uint8_t rxReceived = 0;
uint8_t siRxToReceive = 0;

uint8_t fskTxOk[3] = "ACK";
uint8_t fskTxNok[3] = "NCK";
uint8_t fskTxCrcE[4] = "CRCE";

// TIMER STUFF

uint32_t timer = 0;
// well do stuff every second
uint32_t timerDelay = 10000;

// wtf
float bat_v;
float i4;
float i3_3;
float temp;

// some statistics
float STATS_CMD_MCU_TEMP = 0;
uint32_t STATS_CMD_ACK = 0;
uint32_t STATS_CMD_NCK = 0;
uint32_t STATS_CMD_CRC = 0;
uint32_t STATS_CMD_TMO = 0; // timed out

BOARDTRX_TX_Data mcuTxBatVolt = { 0 };
BOARDTRX_TX_Data mcuTx4vCurr = { 0 };
BOARDTRX_TX_Data mcuTx3v3Curr = { 0 };
BOARDTRX_TX_Data mcuTxTemp = { 0 };
BOARDTRX_TX_Data mcuTxTemporary = { 0 };
BOARDTRX_TX_Data *txDataArray[(HQ_VARS_TOTAL_SIZE - HQ_VARS_LOCAL_LIMIT) - 1] =
        { &mcuTxBatVolt, &mcuTx4vCurr, &mcuTx3v3Curr, &mcuTxTemp, &mcuTxTemporary, &mcuTxTemporary, &mcuTxTemporary, &mcuTxTemporary,
                &mcuTxTemporary, &mcuTxTemporary, &mcuTxTemporary, &mcuTxTemporary };

BOARDTRX_RX_Data comRxGpsLong = { 0 };
BOARDTRX_RX_Data comRxGpsLat = { 0 };
BOARDTRX_RX_Data comRxGpsAlt = { 0 };
BOARDTRX_RX_Data comRxGpsSpeed = { 0 };
BOARDTRX_RX_Data comRxGpsTime = { 0 };
BOARDTRX_RX_Data comRxComTemperature = { 0 };
BOARDTRX_RX_Data comRxComCmdOk = { 0 };
BOARDTRX_RX_Data comRxComCmdNck = { 0 };
BOARDTRX_RX_Data comRxComCmdCrc = { 0 };
BOARDTRX_RX_Data comRxComCmdTmo = { 0 };

// data that can be received from COM
BOARDTRX_RX_Data *rxDataArray[10] = { &comRxGpsLong, &comRxGpsLat, &comRxGpsAlt, &comRxGpsSpeed, &comRxGpsTime, &comRxComTemperature, &comRxComCmdOk, &comRxComCmdNck,
        &comRxComCmdCrc, &comRxComCmdTmo};

#define CMD_COUNT   1
BOARDTRX_TX_Cmd mcuTxBurn = { 0 };
BOARDTRX_TX_Cmd *txCmdArray[CMD_COUNT] = { &mcuTxBurn };

BOARDTRX_Handle mcuTrxHandle = { 0 };

// Varaibles

extern uint8_t gpsData[24];    //0 coord latitude //1 coord longitude "05723.06487N", "02132.70887E"
extern uint8_t gpsHeight[8];   //2 height
extern uint8_t gpsSpeed[7];    //3 speed
extern uint8_t gpsTime[6];     // UTC time from GPGGA

//uint8_t gpsAlt[32];
//uint8_t gpsSpe[8];
//uint32_t mcuTemp = 0;

//HQ_VARS_RX_LONGITUDE,
//HQ_VARS_RX_LATITUDE,
//HQ_VARS_RX_ALTITUDE,
//HQ_VARS_RX_SPEED,
//HQ_VARS_RX_TIME
//HQ_VARS_RX_MCU_TEMP,
//HQ_VARS_RX_CMD_OK,
//HQ_VARS_RX_CMD_NCK,
//HQ_VARS_RX_CMD_CRC,

//HQ_VARS_TX_BAT_VOLT,
//HQ_VARS_TX_4V_CUR,
//HQ_VARS_TX_3V3_CUR,
//HQ_VARS_TX_3V3,
//HQ_VARS_TX_4V,
//HQ_VARS_TX_TMP_MCU,
//HQ_VARS_TX_TMP_IN,
//HQ_VARS_TX_TMP_OUT,
//HQ_VARS_TX_PRESSURE,
//HQ_VARS_TX_HUMID,
//HQ_VARS_TX_AIRQ,
//HQ_VARS_TX_SEN_STATUS,

float testVar = 1.234567890;
uint32_t testVar1 = 111222333;

//uint8_t varTypes[HQ_VARS_TOTAL_SIZE - 1] = { 0 };
uint8_t varSize[HQ_VARS_TOTAL_SIZE - 1] = { 12, 12, 8, 7, 6, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };
uint8_t *varPointers[HQ_VARS_TOTAL_SIZE - 1] = { gpsData, &gpsData[12], gpsHeight, gpsSpeed, gpsTime, (uint8_t*) &testVar, (uint8_t*) &STATS_CMD_ACK,
        (uint8_t*) &STATS_CMD_NCK, (uint8_t*) &STATS_CMD_CRC, (uint8_t*) &STATS_CMD_TMO,
        // end of local variables, start of mcu variables
        (uint8_t*) &bat_v, (uint8_t*) &i4, (uint8_t*) &i3_3, (uint8_t*) &temp, (uint8_t*) &testVar1, (uint8_t*) &testVar1, (uint8_t*) &testVar1,
        (uint8_t*) &testVar1, (uint8_t*) &testVar1, (uint8_t*) &testVar1, (uint8_t*) &testVar1, (uint8_t*) &testVar1, };

uint8_t varSentParts[HQ_VARS_TOTAL_SIZE - 1] = { 0 };
uint8_t varRequests[HQ_VARS_TOTAL_SIZE - 1] = { 0 };
uint32_t varRequestedMcu[(HQ_VARS_TOTAL_SIZE - HQ_VARS_LOCAL_LIMIT) - 1] = { 0 };
//uint32_t varRequest = 0;

uint8_t cmdReqData[4] = { 0 };  // max 4 bytes
uint8_t cmdReqSize = 0;
uint8_t cmdReqNr = 0;
uint8_t cmdReqId = 0;
uint8_t cmdReqState = 0;

/*
 * @Brief Initializes HQ_Handle and starts data reiceiving
 * @param HQ_Handle - pointer to configured handle
 * @retval value of initialization status
 */
HQ_StatusTypeDef HQ_Init(HQ_Handle *handle) {
    HQ_StatusTypeDef retVal = HQ_OK;

    hqHandle = handle;

    // first thing we should do is go into RX mode
    if (SI4463_Receive_FSK_IRQ(hqHandle->siHandle) != SI4463_OK) {
        retVal |= HQ_ERROR_SI;
    }

    // initialize shared variables rx

    comRxGpsLong.data = (uint8_t *)&gpsData;
    comRxGpsLong.size = 12;

    comRxGpsLat.data = (uint8_t *)&gpsData[12];
    comRxGpsLat.size = 12;

    comRxGpsAlt.data = (uint8_t *)&gpsHeight;
    comRxGpsAlt.size = 8;

    comRxGpsSpeed.data = (uint8_t *)&gpsSpeed;
    comRxGpsSpeed.size = 7;

    comRxGpsTime.data = (uint8_t *)&gpsTime;
    comRxGpsTime.size = 6;

    comRxComTemperature.data = (uint8_t *)&STATS_CMD_MCU_TEMP;
    comRxComTemperature.size = 4;

    comRxComCmdOk.data = (uint8_t *)&STATS_CMD_ACK;
    comRxComCmdOk.size = 4;

    comRxComCmdNck.data = (uint8_t *)&STATS_CMD_NCK;
    comRxComCmdNck.size = 4;

    comRxComCmdCrc.data = (uint8_t *)&STATS_CMD_CRC;
    comRxComCmdCrc.size = 4;

    comRxComCmdTmo.data = (uint8_t *)&STATS_CMD_TMO;
    comRxComCmdTmo.size = 4;

    // initialize shared variables rx
    mcuTxBatVolt.data = (uint8_t*) &bat_v;
    mcuTxBatVolt.size = 4;

    mcuTx4vCurr.data = (uint8_t*) &i4;
    mcuTx4vCurr.size = 4;

    mcuTx3v3Curr.data = (uint8_t*) &i3_3;
    mcuTx3v3Curr.size = 4;

    mcuTxTemp.data = (uint8_t*) &temp;
    mcuTxTemp.size = 4;

    mcuTxTemporary.data = (uint8_t*) &testVar1;
    mcuTxTemporary.size = 4;

    uint8_t burnTime = 5;
    mcuTxBurn.data = &burnTime;
    mcuTxBurn.size = 1;

    // this is what we can request
    mcuTrxHandle.countDataTx = 12;
    mcuTrxHandle.dataTx = txDataArray;
    // some timings
    mcuTrxHandle.rxRetries = 3;
    mcuTrxHandle.rxTimeout = 20000;
    // this is what can be requested from us
    mcuTrxHandle.countDataRx = 10;
    mcuTrxHandle.dataRx = rxDataArray;
    // this is commands that can be sent to be executed
    mcuTrxHandle.cmdTx = txCmdArray;
    mcuTrxHandle.countCmdTx = 1;
    // other config
    mcuTrxHandle.uart = handle->uart;

    BOARDTRX_Status ret;
    ret = BOARDTRX_Init(&mcuTrxHandle);

    initialised = 1;

    return retVal;
}

#if DEBUG_ENABLE
extern uint8_t debLed[4];
#endif

/*
 * @Brief goes trough things, processes them etc
 */
HQ_StatusTypeDef HQ_Loop() {
    BOARDTRX_Status ret;

    if (!initialised) {
        return HQ_ERROR;
    }

    ret = BOARDTRX_Loop();

    // TODO count errors, manage them or whatever
    // count errors
    if (ret.isNewError) {
        ret.isNewError = 0;
        if (ret.status == BOARDTRX_ERROR_CRC) {
            STATS_CMD_CRC++;
        }
    }

    if (siRxToReceive) {
        siRxToReceive = 0;
        SI4463_StatusTypeDef ret;
        ret = SI4463_Receive_FSK(hqHandle->siHandle, rxRawBuf, 8, 100);
        if (ret == SI4463_OK) {
            rxReceived = 1;
        }

        // data from GS
        HQ_SI_Process_Data();
    }

    HQ_VAR_Process();
    HQ_CMD_Process();

    return SI4463_OK;
}

/*
 * @brief goes trough VAR things and processes them
 */
void HQ_VAR_Process() {
    for (uint8_t i = 0; i < (HQ_VARS_TOTAL_SIZE - 1); i++) {
        if (varRequests[i]) {
            if (i < HQ_VARS_LOCAL_LIMIT) {
                if (HQ_VAR_Send(i)) {
                    varRequests[i] = 0;
                }
            } else {
                // check if received from mcu before sending
                // if has then send
                uint8_t tmpInd = (i - HQ_VARS_LOCAL_LIMIT);
                uint8_t status = txDataArray[tmpInd]->status;
                if (status == BOARDTRX_CMD_NEW || status == BOARDTRX_CMD_REP) {
                    STATS_CMD_ACK++;
                    if (HQ_VAR_Send(i)) {
                        txDataArray[tmpInd]->status = BOARDTRX_CMD_DEF;
                        varRequests[i] = 0;
                    }

                } else if (status == BOARDTRX_CMD_TIM) {
                    STATS_CMD_TMO++;
                    // timed out
                    txDataArray[tmpInd]->status = BOARDTRX_CMD_DEF;
                    varRequests[i] = 0;

                } else if (status == BOARDTRX_CMD_DEF) {
                    BOARDTRX_DataRequest(tmpInd);

                } else if (status == BOARDTRX_CMD_SEN) {
                    return;
                } else {
                    STATS_CMD_NCK++;
                    varRequests[i] = 0;
                    txDataArray[tmpInd]->status = BOARDTRX_CMD_DEF;

                }
            }
        }
    }
}

/*
 * @brief goes trough CMD things and processes them
 */
void HQ_CMD_Process() {

    if (cmdReqState == 1) {
        // send cmd request to mcu
        BOARDTRX_CmdSend(cmdReqNr, cmdReqId, 1);
        cmdReqState = 2;
    } else if (cmdReqState == 2) {
        // wait for reply from mcu
        BOARDTRX_CMD_Status retVal = BOARDTRX_CMD_DEF;
        retVal = BOARDTRX_CmdGetStatus(cmdReqNr);
        if (retVal == BOARDTRX_CMD_NEW || retVal == BOARDTRX_CMD_OK) {
            cmdReqState = 3;
        } else if (retVal == BOARDTRX_CMD_REP) {
            cmdReqState = 4;
        } else if (retVal == BOARDTRX_CMD_NOK) {
            cmdReqState = 255;
        }
    } else if (cmdReqState == 3) {
        // everythings went ok
        cmdReqState = 0;
        HQ_CMD_Send(HQ_CMD_STATUS_OK);

    } else if (cmdReqState == 4) {
        // cmd alreadu executed
        cmdReqState = 0;
        HQ_CMD_Send(HQ_CMD_STATUS_REP);

    } else if (cmdReqState == 5) {
        // invalid cmd Nr
        cmdReqState = 0;
        HQ_CMD_Send(HQ_CMD_STATUS_NR);

    } else if (cmdReqState == 6) {
        // invalid cmd data size
        cmdReqState = 0;
        HQ_CMD_Send(HQ_CMD_STATUS_SIZE);

//    } else if (cmdReqState == 7) {
//        // invalid cmd data size
//        cmdReqState = 0;
//        HQ_CMD_Send(HQ_CMD_STATUS_SIZE);
    } else if (cmdReqState >= HQ_CMD_STATUS_NR) {
        // invalid cmd data size
        cmdReqState = 0;
        HQ_CMD_Send(HQ_CMD_STATUS_ERROR);
    }
}

/*
 * @Brief sends variable trough SI, if sent fully returns 1 if more data needs to be sent returns 0
 * @param ind index of variable to send from varPointers
 * @retval 1 if variable was sent fully, 0 if more data needs to be sent
 */
uint8_t HQ_VAR_Send(uint8_t ind) {
    uint8_t *data = 0x00;
    uint8_t *parts = &varSentParts[ind];
    data = varPointers[ind];

    uint8_t sentParts = *parts;
    uint8_t totalParts = varSize[ind];
    uint8_t toSend = totalParts - sentParts;

    // send as much as we can in one go
    if (toSend > HQ_FSK_MAX_DATA_SIZE) {
        toSend = HQ_FSK_MAX_DATA_SIZE;
    }

    // variable is split into parts in case it cant be sent in a single 8 byte packet
    uint8_t tmp[8] = { 0 };
    tmp[0] = GS_COM_VAR;
    tmp[1] = ind;
    tmp[2] = totalParts;
    tmp[3] = sentParts;
    for (uint8_t i = 0; i < toSend; i++) {
        tmp[4 + i] = data[sentParts + i];
    }

    HQ_SI_Send(tmp, 8);

    sentParts += toSend;

    // save how many we have sent
    if (sentParts == totalParts) {
        *parts = 0;
        return 1;
    } else {
        *parts = sentParts;
    }

    return 0;
}

/*
 * @Brief sends CMD trough SI, if sent returns 1
 * @param status status of cmd that will be sent
 * @retval 1 if sent
 */
uint8_t HQ_CMD_Send(HQ_CmdStatusTypeDef status) {

    uint8_t tmp[4] = { 0 };
    tmp[0] = GS_COM_CMD;
    tmp[1] = cmdReqNr;
    tmp[2] = cmdReqId;
    tmp[3] = status;

    HQ_SI_Send(tmp, 8);

    return 1;
}

/*
 * @Brief Sends data packet
 * @Param data pointer to data to send
 * @Param size size of data to send
 */
HQ_StatusTypeDef HQ_UART_Send_Packet(uint8_t *data, uint8_t size) {
    uint8_t tmp[255];

    // add start byte
    tmp[0] = HQ_UART_START_BYTE;
    tmp[1] = size;

    // copy data to send
    memcpy(&tmp[2], data, size);

    uint16_t tmpCrc = HQ_CalcCRC(data, size);
    // add crc
    tmp[size + 2] = tmpCrc >> 8;
    tmp[size + 3] = tmpCrc & 0xff;

    // transmit
    if (HAL_UART_Transmit(hqHandle->uart, tmp, size + 4, 100) == HAL_OK) {
        return HQ_OK;
    }

    return HQ_ERROR;
}

/*
 * @Brief Requests specified cmd
 * @Param cmd pointer to data to send
 * @Param len length of cmd
 * @Param buf pointer where to store received data
 * @Param rLen pointer where received length of data will be written
 * @Retval status of request
 */
HQ_StatusTypeDef HQ_UART_Request(uint8_t *cmd, uint8_t len, uint8_t *buf, uint8_t *rLen) {
    // send cmd
    HQ_UART_Send_Packet(cmd, len);

    // get response
    if (uartRxToReceive) {
        if (HQ_UART_Receive_Packet() == HQ_OK) {
            // check crc
            //uint8_t tmpCrc = HQ_CalcCRC(uartRxBuf, uartRxToReceive);
            //if (uartRxBuf[uartRxToReceive] != (tmpCrc >> 8) || uartRxBuf[uartRxToReceive + 1] != (tmpCrc & 0xff)) {
            //    return HQ_ERROR_CMD;
            //}
            if (uartRxBuf[1] == HQ_CMD_RX_NCK) {
                return HQ_ERROR_CMD;
            }

        }
    }
    return HQ_OK;
}

/*
 * @Brief Gets whole data packet
 */
HQ_StatusTypeDef HQ_UART_Receive_Packet() {

    if (uartRxLen == 255) {
        if (uartMiniBuf == HQ_UART_START_BYTE) {
            uartRxLen = 0;
        }
    } else {
        uartRxBuf[uartRxLen] = uartMiniBuf;
        uartRxLen++;
        if (uartRxLen == 1) {
            uartRxToGet = uartMiniBuf;
        }
    }

    if ((uartRxLen + 2) == uartRxToGet) {
        uartRxLen = 255;
        return HQ_OK;
    }

    return HQ_BUSY;
}

HQ_StatusTypeDef HQ_SI_Process_Data() {
    /*
     *  simple FSK packet format
     *  y   : x - 6 bytes   - data itself, no more than 6 bytes
     *  CRC : 2 bytes       - XOR checksum of data, HQ_CalcCRC()
     */

    HQ_StatusTypeDef retVal = SI4463_RX_CRC_ERROR;

    // process received data from FSK
    if (rxReceived) {
        rxReceived = 0;
        uint8_t data[9] = { 0 };
        memcpy(data, &rxRawBuf, 8);

#if DEBUG_ENABLE
        printf("processing: ");
        printf((char*) data);
        printf("\n\r");
        debLed[2] = !debLed[2];
#endif

        // check crc
//        uint16_t tmpCrc = HQ_CalcCRC(data, 6);
//        if ((tmpCrc >> 8 == data[6]) && ((uint8_t) tmpCrc) == data[7]) {
        if (data[0] == GS_COM_REG) {

        } else if (data[0] == GS_COM_CMD) {
            cmdReqState = 1;
            cmdReqNr = data[1];
            cmdReqId = data[2];
            cmdReqSize = data[3];

            if (cmdReqNr >= CMD_COUNT) {
                cmdReqState = 5;
                STATS_CMD_NCK++;
                return SI4463_ERROR;
            }

            if (txCmdArray[cmdReqNr]->size != cmdReqSize) {
                cmdReqState = 6;
                STATS_CMD_NCK++;
                return SI4463_ERROR;
            }

            STATS_CMD_ACK++;
            memcpy(txCmdArray[cmdReqNr]->data, &data[4], cmdReqSize);

        } else if (data[0] == GS_COM_VAR) {
            if (data[1] < (HQ_VARS_TOTAL_SIZE - 1)) {
                varRequests[data[1]] = 1;
            } else {
                uint8_t res[2];
                res[0] = GS_COM_VAR_RES;
                res[1] = GS_COM_NCK;
                HQ_SI_Send(res, 2);
                retVal = SI4463_RX_CRC_ERROR;
                STATS_CMD_NCK++;
            }
            STATS_CMD_ACK++;
        } else {
            // command / message unknown or faulty
            STATS_CMD_NCK++;
            uint8_t res = GS_COM_UNK;
            HQ_SI_Send(&res, 1);
            retVal = SI4463_RX_CRC_ERROR;
        }
//        } else {
//            // command / message unknown or faulty
//            HQ_SI_Send(fskTxCrcE, sizeof(fskTxCrcE));
//            retVal = SI4463_RX_UNKNOWN;
//        }

        SI4463_Receive_FSK_IRQ(hqHandle->siHandle);
        return retVal;
    }

    if (timer < HAL_GetTick()) {
        timer += timerDelay;
#if DEBUG_ENABLE
        printf("Tick %ld\n\r", HAL_GetTick() / 1000);
#endif
    }

    return SI4463_OK;
}

/*
 * @Brief sends data trough FSK to WASSUP. before formating data packet and adding checksum
 * @param data - pointer to data to send
 * @param size - size of data to send
 */
HQ_StatusTypeDef HQ_SI_Send(uint8_t *data, uint8_t size) {
    uint8_t txData[8] = { 0 };

    if (size > 8 || size == 0) {
        return SI4463_ERROR;
    }

    // copy data to send to packet
    memcpy(txData, data, size);

//    // get crc
//    uint16_t tmpCrc = HQ_CalcCRC(txData, 6);
//
//    // put CRC in packet
//    txData[6] = tmpCrc >> 8;
//    txData[7] = tmpCrc & 0xff;

    // send data and then go back to RX
    SI4463_StatusTypeDef ret;
    ret = SI4463_Transmit_FSK(hqHandle->siHandle, txData, 8, 100);
    if (ret != SI4463_OK) {
        SI4463_Receive_FSK_IRQ(hqHandle->siHandle);
        return HQ_ERROR;
    }

    SI4463_Receive_FSK_IRQ(hqHandle->siHandle);

    return HQ_OK;
}

// gets received data and puts it in rxBufQue
void HQ_UART_RX_CallBack() {
    //uartRxToReceive = 1;
}

// gets received data and puts it in rxBufQue
void HQ_SI_RX_CallBack() {
    siRxToReceive = 1;
//    SI4463_Receive_FSK(hqHandle->siHandle, rxBufQue, 8, 100);
//    rxReceived = 1;
}

// some simple CRC
uint16_t HQ_CalcCRC(uint8_t *data, uint8_t size) {
    uint8_t crc_a = 0;
    uint8_t crc_b = 0;

    for (uint8_t i = 0; i < size; i++) {
        crc_a ^= data[i];
        crc_b ^= crc_a;
    }

    return (crc_a << 8) + crc_b;
}

// runs when got response from UART
void HQ_UART_RequestCallBack() {
    //MAIN_HQ_Response();
}

