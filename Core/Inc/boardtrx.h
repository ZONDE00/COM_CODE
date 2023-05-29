/*
 * boardtrx.h
 *
 *  Created on: 7 Mar 2023
 *      Author: Kristers
 */

#ifndef SRC_BOARDTRX_H_
#define SRC_BOARDTRX_H_

#include "main.h"

#define BOARDTRX_DATA_COUNT       32
#define BOARDTRX_CMD_COUNT        8

typedef enum {
    BOARDTRX_OK,
    BOARDTRX_ERROR,
    BOARDTRX_BUSY,
    BOARDTRX_TIMEOUT,
    BOARDTRX_MISS,
    BOARDTRX_ERROR_CRC,
    BOARDTRX_ERROR_CMD,
    BOARDTRX_ERROR_TX,
} BOARDTRX_StatusTypeDef;

typedef enum {
    BOARDTRX_INT, // other error internal error
    BOARDTRX_TX,
    BOARDTRX_RX
//    BOARDTRX_ERROR_UART,
} BOARDTRX_TRX_Status;

typedef enum {
    BOARDTRX_CMD_DEF,   // default state, nothing is happening

    BOARDTRX_CMD_NEW,   // new id respond with ack
    BOARDTRX_CMD_REP,   // same id, already executed
    BOARDTRX_CMD_NCK,

    BOARDTRX_CMD_SEN,   // trying to send CMD
    BOARDTRX_CMD_OK,    // CMD was sent successfully
    BOARDTRX_CMD_NOK,   // CMD was not sent
    BOARDTRX_CMD_TIM,    // timeout
} BOARDTRX_CMD_Status;

typedef enum {
    BOARDTRX_DATA_TX,
    BOARDTRX_DATA_RX,
    BOARDTRX_CMD_TX,
    BOARDTRX_CMD_RX,
} BOARDTRX_TRX_Target;

/*
 * Data VS CMD
 *
 * Data can be requested from target devices, we can't send it
 * when data is requested or sent we don't care if receiving device actually got anything thus we don't get ACK or NCK
 * we only get timeout if we don't get data requested
 *
 * CMD is data like, but we send it to target device, we don't request it
 * When CMD is sent we do care if it was received or not, that is why we will wait for ACK or NCK
 * CMD uses ID system to make sure CMD is received only once, even though target device have gotten it correctly but we did not get response
 * CMD sent with same ID are dismissed, ID changes only when ACK is received from target device.
 *
 */

// data that can be requested from target
typedef struct {
    // external
    uint8_t *data; // data that will be received
    uint8_t size;  // size of data
    uint8_t status; // set if is new, clear manually
    // internal
    uint8_t retries;
    uint32_t timeout;

} BOARDTRX_TX_Data;

// data that target device can request
typedef struct {
    uint8_t *data; // communication with target device
    uint8_t size;  // size of variable
    uint8_t wasRequested; // set if was requested, clear manually'
} BOARDTRX_RX_Data;

// cmd that can be received from target device
typedef struct {
    // external
    uint8_t *data; // data that will be received
    uint8_t size;  // size of data
    void (*CMD_CB)(); // callback function to call when CMD is received successfully
    // internal
    BOARDTRX_CMD_Status status;
    uint8_t id;
} BOARDTRX_RX_Cmd;

// data that we can send to target device
typedef struct {
    // external
    uint8_t *data; // data to send to target
    uint8_t size;  // size of data to send
    BOARDTRX_CMD_Status status; // BOARDTRX_CMD_DEF when idle, BOARDTRX_CMD_OK when sent successfully, BPARDTRX_CMD_NOK, when was not sent
                                // BOARDTRX_CMD_SEN when sending
    // internal
    uint8_t retries;
    uint32_t timeout;
    uint8_t id;    // id of last request
    uint8_t wasSent;
} BOARDTRX_TX_Cmd;

// not mandatory but can be used to determine TRX quality
typedef struct {
    // general status indicators
    BOARDTRX_StatusTypeDef status; // status of last function
    uint8_t isNewError; // set if new error happened, clear manually
    uint8_t errorDataNr; // index of data when last error happened, valid if isNewError was set
    BOARDTRX_TRX_Status errorTRX;    // valid if isNewError was set, marks whether error was on RX or TX

    // error counters
    uint32_t errorsCRC; // amount of CRC errors that happened
    uint32_t errorsRX; // amount of unrecognised packets received
    uint32_t errorsTX; // amount of packets that was not recognised by target device
    uint32_t errorsTimeout;  // total timeouts
    uint32_t errorsMissed;   // counts how many times new request was received before fulfilling last
    uint32_t errorsInt; // other internal errors
} BOARDTRX_Status;

/*
 * Required struct for initialisation
 * Can be used for just TX or RX, or both
 *
 * Pass pointer of variablesXX array to struct and size of it.
 *
 * BOARDTRX_XX_Data struct holds data and its size.
 * As data type is uint8_t*, any variable can be cast to uint8_t* and be used,
 *      as long as proper size is set.
 *
 *  Max data size is 250.
 *  Max data count is BOARDTRX_DATA_COUNT, hard max is 64
 *  Max CMD  count is BOARDTRX_CMD_COUNT,  hard max is 64
 */
typedef struct {
    // data that can be received from target device
    BOARDTRX_TX_Data **dataTx; // pointer to array of pointers
    uint8_t countDataTx;

    // data that target device can request
    BOARDTRX_RX_Data **dataRx; // pointer to array of pointers
    uint8_t countDataRx;

    // CMDs that can be sent to target device
    BOARDTRX_TX_Cmd **cmdTx; // pointer to array of pointers
    uint8_t countCmdTx;

    // CMDs that can be received from target device
    BOARDTRX_RX_Cmd **cmdRx; // pointer to array of pointers
    uint8_t countCmdRx;

    UART_HandleTypeDef *uart; // communication handle with target device

    uint8_t rxRetries; // how many times will retry to receive data before giving up, default 3
    uint32_t rxTimeout; // time in which will rx timeout in ms, default 20'000

    void (*TX_COMPLETE_CB)(); // callback when target requested data was sent, not mandatory
    void (*RX_COMPLETE_CB)(); // callback when requested data was received, not mandatory
    void (*ERROR_CB)();       // callback on error, not mandatory
} BOARDTRX_Handle;

// ---------------------------------------------------- external functions
BOARDTRX_Status BOARDTRX_Init(BOARDTRX_Handle *handle);
BOARDTRX_Status BOARDTRX_DataRequestAll();
BOARDTRX_Status BOARDTRX_DataRequest(uint8_t nr);
BOARDTRX_Status BOARDTRX_CmdSend(uint8_t nr, uint8_t id, uint8_t setId);
BOARDTRX_CMD_Status BOARDTRX_CmdGetStatus(uint8_t nr);
void BOARDTRX_CmdClearStatus(uint8_t nr);
// run this in main loop
BOARDTRX_Status BOARDTRX_Loop();

// add this function to UART receive complete callbacks
void BOARDTRX_UART_RX_CB();
// add this function to UART error complete callbacks
void BOARDTRX_UART_RXERROR_CB();

//  ---------------------------------------------------- internal functions
BOARDTRX_Status BOARDTRX_ProcessTx(BOARDTRX_TRX_Target target, uint8_t nr);
BOARDTRX_Status BOARDTRX_ProcessRx();
BOARDTRX_Status BOARDTRX_UART_Send(uint8_t nr, uint8_t *data, uint8_t size);

uint16_t BOARDTRX_CalcCRC(uint8_t *data, uint8_t size);
uint8_t BOARDTRX_CheckCRC(uint8_t *data, uint8_t size);

BOARDTRX_Status BOARDTRX_AddToQue(BOARDTRX_TRX_Target target, uint8_t dataNr);
void BOARDTRX_ClearFromQue(BOARDTRX_TRX_Target target, uint8_t dataNr);
uint8_t BOARDTRX_GetFromQue(BOARDTRX_TRX_Target target, uint8_t startInd);

#endif /* SRC_BOARDTRX_H_ */
