#ifndef INC_GSM_H_
#define INC_GSM_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    /* USER MUST SET THESE */
    UART_HandleTypeDef *uart;

    GPIO_TypeDef *PWRPort;  // power pin
    uint16_t PWRPin;

    GPIO_TypeDef *RSTPort; // reset pin
    uint16_t RSTPin;

} GSM_Handle;


typedef enum {
    GSM_OK = 0x00U,
    GSM_ERROR = 0x01U
} GSM_StatusTypeDef;

void GSM_Reset();
void GSM_On();
void GSM_Off();

GSM_StatusTypeDef GSM_Init(GSM_Handle *GSM_Handle);
uint8_t GSM_Check_Signal();
uint8_t GSM_Check_Signal_Quality();
uint8_t GSM_Message_Send(uint8_t *dataBuf, uint8_t size, uint32_t number);
uint8_t GSM_IsOk(uint8_t *dataBuf, uint8_t size);
uint8_t GSM_Find(uint8_t *dataBuf, uint8_t size, uint8_t *toFind, uint8_t tSize);
void GSM_Send(uint8_t *dataBuf, uint8_t size);
void GSM_Receive(uint8_t *dataBuf, uint8_t size);

#endif /* INC_GSM_H_ */
