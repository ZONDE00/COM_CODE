#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"

#define FIX_MODE_2D 0
#define FIX_MODE_3D 1
#define FIX_MODE_AUTO 2

typedef enum {
    GPS_OK = 0x00U,
    GPS_ERROR = 0x01U,
    GPS_NOK = 0x02U
} GPS_StatusTypeDef;

typedef enum {
    GPS_MODEL_PORTABLE = 0x00U,
    GPS_MODEL_STATIONARY = 0x02U,
    GPS_MODEL_PEDESTRIAN = 0x03U,
    GPS_MODEL_AUTOMOTIVE = 0x04U,
    GPS_MODEL_SEA = 0x05U,
    GPS_MODEL_AIRBORNE_1 = 0x06U,
    GPS_MODEL_AIRBORNE_2 = 0x07U,
    GPS_MODEL_AIRBORNE_3 = 0x08U
} GPS_MODEL_StatusTypeDef;

void GPS_Receive(uint8_t data);
void GPS_GetCoord();

GPS_StatusTypeDef GPS_IsData();
GPS_StatusTypeDef GPS_IsNewData();

void GPS_GetTime(uint8_t *buf);
void GPS_GetLat(uint8_t *buf);
void GPS_GetLon(uint8_t *buf);
void GPS_GetSpeed(uint8_t *buf);
void GPS_GetHei(uint8_t *buf);
void GPS_GetYear(uint8_t *buf);
void GPS_GetMonth(uint8_t *buf);
void GPS_GetDate(uint8_t *buf);
void GPS_GetTime(uint8_t *buf);
GPS_StatusTypeDef GPS_SetBaudrate(uint32_t baudRate);
GPS_StatusTypeDef GPS_SetDynamicModel(GPS_MODEL_StatusTypeDef model);
GPS_StatusTypeDef GPS_GetDynamicModel(uint8_t *model);
GPS_StatusTypeDef GPS_GetBaudRate(uint32_t *baudRate);


GPS_StatusTypeDef GPS_Parse(uint8_t *buf, uint8_t len);
uint8_t GPS_CheckSum(uint8_t *buf, uint8_t len);
uint8_t GPS_message_checksum(char *message, uint16_t size);
GPS_StatusTypeDef GPS_HexToByte(uint8_t *hex, uint8_t *value);
uint16_t UBX_Checksum(uint8_t *data, uint8_t size);
void GPS_ChecksumUBLOX(uint8_t *data);
size_t write_to_buffer(uint8_t *buffer, size_t size, uint8_t data);
uint8_t GPS_init_Uart(UART_HandleTypeDef *huart);


#endif /* INC_GPS_H_ */
