/*
 *	GSM library made for A6 thinkerer (might work for other modules)
 * 	Made for sending sms only
 */

#include "gsm.h"

GSM_Handle *handle;

#define GSM_MSG_END											13				// carriage return
#define GSM_MODE_TEXT										"AT+CMGF=1"
#define GSM_SET_NUMBER										"AT+CMGS=\""
#define GSM_MSG_SEND										26				//CTRL + Z
//#define GSM_MSG_SPACE										32				//SPACE

/*
 * @Brief Initializes GSM
 * @retVal GSM_OK if initializes, GSM_ERROR if not
 */
uint8_t GSM_Init(GSM_Handle *GSM_Handle) {
    if (GSM_Handle == 0x00) {
        return GSM_ERROR;
    }

    handle = GSM_Handle;

    if (handle->uart == 0x00) {
        return GSM_ERROR;
    }

    // TODO check for GSM device

    return GSM_OK;
}

/*
 * @Brief Checks if GSM has good signal
 * @retVal 1 if has good signal, 0 if does not
 */
uint8_t GSM_Check_Signal() {
    uint8_t buf[32] = { 0 };
    buf[0] = 'A';
    buf[1] = 'T';

    GSM_Send(buf, 2);
    GSM_Receive(buf, 9);

    return GSM_IsOk(buf, 9);
}

/*
 * @Brief Checks if GSM has good signal
 * @retVal 1 if has good signal, 0 if does not
 */
uint8_t GSM_Check_Signal_Quality() {
    uint8_t buf[32] = { 0 };
    buf[0] = 'A';
    buf[1] = 'T';
    buf[1] = '+';
    buf[1] = 'C';
    buf[1] = 'S';
    buf[1] = 'Q';

    GSM_Send(buf, 6);
    GSM_Receive(buf, 13);

    return GSM_IsOk(buf, 9);
}

/*
 * @Brief Checks if string has "OK" in it
 * @param dataBuf pointer to text where to search for "OK"
 * @param size size of text
 * @retVal 1 if has "OK", 0 if does not
 */
uint8_t GSM_IsOk(uint8_t *dataBuf, uint8_t size) {
    uint8_t i = 0;

    for (; i < size; i++) {
        if (*dataBuf == 'O') {
            dataBuf++;
            if (*dataBuf == 'K') {
                return GSM_OK;
            }
        }
        dataBuf++;
    }
    return GSM_ERROR;
}

/*
 * @Brief Searches for specific things inside data
 * @param dataBuf pointer to text where to search
 * @param size size of text
 * @param toFind pointer to characters to find
 * @param tSize size of characters to find
 * @retVal 1 if has "OK", 0 if does not
 */
uint8_t GSM_Find(uint8_t *dataBuf, uint8_t size, uint8_t *toFind, uint8_t tSize) {
    uint8_t i = 0;
    uint8_t f = 0;
    uint8_t temp[16];

    memcpy(temp, toFind, tSize);

    for (; i < size; i++) {

        if (*dataBuf == temp[f]) {
            f++;
        } else {
            f = 0;
        }

        if (f == tSize) {
            return GSM_OK;
        }
        dataBuf++;
    }

    return GSM_ERROR;
}

/*
 * @Brief Resets GSM
 */
void GSM_Reset() {
    HAL_GPIO_WritePin(handle->PWRPort, handle->PWRPin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(handle->RSTPort, handle->RSTPin, GPIO_PIN_SET);
    HAL_Delay(1000);
}

/*
 * @Brief Turns on GSM
 */
void GSM_On() {
    HAL_GPIO_WritePin(handle->PWRPort, handle->PWRPin, GPIO_PIN_SET);
    HAL_Delay(3000);
    //HAL_GPIO_WritePin(handle->PWRPort, handle->PWRPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(handle->RSTPort, handle->RSTPin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GSM_RTS_GPIO_Port, GSM_RTS_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GSM_CTS_GPIO_Port, GSM_CTS_Pin, GPIO_PIN_RESET);
}

/*
 * @Brief Turns off GSM
 */
void GSM_Off() {
    HAL_GPIO_WritePin(handle->PWRPort, handle->PWRPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(handle->RSTPort, handle->RSTPin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GSM_RTS_GPIO_Port, GSM_RTS_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GSM_CTS_GPIO_Port, GSM_CTS_Pin, GPIO_PIN_SET);
}

/*
 * @Brief Sends SMS with specified number and message
 * @param dataBuf pointer to data to send
 * @param size size of data to send
 * @param number telephone number as uint32_t where to send SMS to
 * @retVal 1 if SMS was sent, 0 if was not
 */
uint8_t GSM_Message_Send(uint8_t *dataBuf, uint8_t size, uint32_t number) {
    uint8_t temp[256];

    if (GSM_Check_Signal() == GSM_ERROR) {
        return GSM_ERROR;
    }

    memcpy(temp, GSM_MODE_TEXT, 9);
    GSM_Send(temp, 9);
    GSM_Receive(temp, 20);
    if (GSM_IsOk(temp, 20) == GSM_ERROR) {
        return GSM_ERROR;
    }

    memcpy(temp, GSM_SET_NUMBER, 9);
    sprintf((char*) (temp + 9), "%lu", number);
    memcpy(temp + 17, "\"", 1);
    GSM_Send(temp, 17);
    GSM_Receive(temp, 40);
    if (GSM_Find(temp, 40, (uint8_t*) ">", 1) == GSM_ERROR) {
        return GSM_ERROR;
    }

    temp[0] = GSM_MSG_SEND;
    GSM_Send(dataBuf, size);
    GSM_Send(temp, 1);

    HAL_UART_Receive(handle->uart, temp, 255, 100);
    HAL_UART_Receive(handle->uart, temp, 17, 5000);
    if (GSM_IsOk(temp, size + 40) == GSM_ERROR) {
        return GSM_ERROR;
    }

    return GSM_OK;
}

/*
 * @Brief Sends raw data to GSM
 * @param dataBuf pointer to data to send
 * @param size size of data to send
 */
void GSM_Send(uint8_t *dataBuf, uint8_t size) {
    uint8_t temp = GSM_MSG_END;
    HAL_UART_Transmit(handle->uart, dataBuf, size, 200);
    HAL_UART_Transmit(handle->uart, &temp, 1, 200);
}

/*
 * @Brief Receivs raw data from GSM
 * @param dataBuf pointer where data will be stored
 * @param size size of data to receive
 */
void GSM_Receive(uint8_t *dataBuf, uint8_t size) {
    HAL_UART_Abort(handle->uart);   // this should not be necessary
    HAL_UART_Receive(handle->uart, dataBuf, size, 1000);
}
