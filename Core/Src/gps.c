#include "gps.h"
#include "string.h"

uint8_t gpsData[24] = "05723.10817N02132.78037E";    //0 coord latitude //1 coord longitude "05723.06487N", "02132.70887E"
uint8_t gpsHeight[8] = "000000.0";   //2 height
uint8_t gpsSpeed[7] = "000.000";    //3 speed
uint8_t gpsTime[6] = "000000";     // UTC time from GPGGA
uint8_t gpsTime_UTC[9] = { 0 }; // UTC time from GPZDA
uint8_t gpsDate[3] = { 0 };     // date from GPZDA
uint8_t gpsMonth[3] = { 0 };    // month from GPZDA
uint8_t gpsYear[5] = { 0 };     // year from GPZDA
uint8_t local_zone_desc[3] = { 0 }; // DON"T KNOW
uint8_t local_zone_min_desc[3] = { 0 }; // DON"T KNOW

uint8_t gpsTemp[80];
uint8_t gpsTempLen = 255;

uint8_t isData = 0;
uint8_t isNewData = 0;

UART_HandleTypeDef *GPS_uart;

/* pass UART handle that will communicate with GPS module*/
uint8_t GPS_init_Uart(UART_HandleTypeDef *huart) {
    if (huart == 0x00) {
        return HAL_ERROR;
    }

    GPS_uart = huart;

    // try to set baud rate
    //if (GPS_SetBaudrate(115200) != GPS_OK) {
    //    return GPS_ERROR;
    //}

    // try to set mode
    return GPS_SetDynamicModel(GPS_MODEL_AIRBORNE_1);
}

GPS_StatusTypeDef GPS_SetBaudrate(uint32_t baudRate) {
    uint32_t timeout = 1000;

    uint8_t data[28] = {
            0xB5, 0x62, // header
            0x06, 0x00, // ID CFG_PRT
            0x14, // length 24
            0x00, // length
            // payload part
            0x01,// port id
            0x00, // reserved0
            0x00, // txReady
            0x00, // txReady
            0xD0, // mode
            0x08, // mode
            0x00, // mode
            0x00, // mode

            (uint8_t)baudRate,         // baud rate
            (uint8_t)(baudRate >> 8),  // baud rate
            (uint8_t)(baudRate >> 16), // baud rate
            (uint8_t)(baudRate >> 24), // baud rate

            0x07, // inProtoMask
            0x00, // inProtoMask
            0x03, // outProtoMask
            0x00, // outProtoMask
            0x00, // reserved 4
            0x00, // reserved 4
            0x00, // reserved 5
            0x00, // reserved 5
            // checksum
            0x00,// chk_a
            0x00 // chk_b
            };

    // generate checkSum
    GPS_ChecksumUBLOX(data);

    // send Data
    if (HAL_UART_Transmit(GPS_uart, data, 28, 1000) != HAL_OK) {
        return GPS_ERROR;
    }

    // reconfigure UART to new baud rate
    GPS_uart->Init.BaudRate = baudRate;
    if (HAL_UART_Init(GPS_uart) != HAL_OK) {
        Error_Handler();
    }

    return GPS_OK;

    // get response
    uint32_t endTime = HAL_GetTick() + timeout;
    uint8_t counter = 0;
    uint8_t ackHeader[4] = {0xB5, 0x62, 0x05, 0x01};
    while(1){
        // get first four bytes of ack frame, we are skipping rest of ack frame
        if (HAL_UART_Receive(GPS_uart, data + counter, 1, timeout) == HAL_OK) {
            if(data[counter] == ackHeader[counter]){
                counter++;
                if(counter == 4){
                    break;
                }
            }else{
                counter = 0;
            }
        }

        if(endTime < HAL_GetTick()){
            return GPS_ERROR;
        }
    }

    return GPS_OK;
}

//// get response
//if (HAL_UART_Receive(GPS_uart, data, 10, 1000) != HAL_OK) {
//    return GPS_ERROR;
//}
//
//// check header
//if (data[0] != 0xB5 || data[1] != 0x62) {
//    return GPS_ERROR;
//}
//
//// check if was id     check if was ACK
//if (data[2] != 0x05 || data[3] != 0x01) {
//    return GPS_ERROR;
//}

GPS_StatusTypeDef GPS_SetDynamicModel(GPS_MODEL_StatusTypeDef model) {
    uint32_t timeout = 1000;

    uint8_t data[44] = {
            0xB5, 0x62, // header
            0x06, 0x24, // id CFG_NAV5
            0x24, // length 36
            0x00, // length
            // payload
            0x01,// mask
            0x00, // mask

            model, // dynModel

            0x00, // fixmode        maybe should be 3 not 0
            0x00, // fixedAlt
            0x00, // fixedAlt
            0x00, // fixedAlt
            0x00, // fixedAlt
            0x00, // fixedAltVar
            0x00, // fixedAltVar
            0x00, // fixedAltVar
            0x00, // fixedAltVar
            0x00, // minElv
            0x00, // drLimit
            0x00, // pDop
            0x00, // pDop
            0x00, // tDop
            0x00, // tDop
            0x00, // pAcc
            0x00, // pAcc
            0x00, // tAcc
            0x00, // tAcc
            0x00, // staticHoldThresh
            0x00, // dgpsTimeOut
            0x00, // reseved 2
            0x00, // reseved 2
            0x00, // reseved 2
            0x00, // reseved 2
            0x00, // reseved 3
            0x00, // reseved 3
            0x00, // reseved 3
            0x00, // reseved 3
            0x00, // reseved 4
            0x00, // reseved 4
            0x00, // reseved 4
            0x00, // reseved 4
            // checksum
            0x00, // chk_a
            0x00  // chk_d
            };

    // calcualte checksum
    GPS_ChecksumUBLOX(data);

    // send data
    if (HAL_UART_Transmit(GPS_uart, data, 44, 100) != HAL_OK) {
        return GPS_ERROR;
    }

    // get response
    uint32_t endTime = HAL_GetTick() + timeout;
    uint8_t ackHeader[4] = { 0xB5, 0x62, 0x05, 0x01 };
    while (1) {
        // get first four bytes of ack frame, we are skipping rest of ack frame as its not important
        if (HAL_UART_Receive(GPS_uart, data, 4, timeout) == HAL_OK) {
            if (memcmp(data, ackHeader, 4) == 0) {
                break;
            }
        }

        if (endTime < HAL_GetTick()) {
            return GPS_ERROR;
        }
    }

    return GPS_OK;
}

GPS_StatusTypeDef GPS_GetDynamicModel(uint8_t *model) {
    uint8_t data[44] = {
            0xB5, 0x62, // header
            0x06, 0x24, // id CFG_NAV5
            0x00, // length
            0x00, // length
            0x00, // chk_a
            0x00, // chk_b
            };

    // calc checksum
    GPS_ChecksumUBLOX(data);

    // send data
    if (HAL_UART_Transmit(GPS_uart, data, 8, 100) != HAL_OK) {
        return GPS_ERROR;
    }

    // get response
    if (HAL_UART_Receive(GPS_uart, data, 44, 200) != HAL_OK) {
        return GPS_ERROR;
    }

    return GPS_OK;
}

GPS_StatusTypeDef GPS_GetBaudRate(uint32_t *baudRate) {
    uint8_t data[44] = {
            0xB5, 0x62, // header
            0x06, 0x00, // id CFG-PRT
            0x00, // length
            0x00, // length
            0x00, // chk_a
            0x00, // chk_b
            };

    // calc checksum
    GPS_ChecksumUBLOX(data);

    // send data
    if (HAL_UART_Transmit(GPS_uart, data, 8, 100) != HAL_OK) {
        return GPS_ERROR;
    }

    // get response
    if (HAL_UART_Receive(GPS_uart, data, 28, 200) != HAL_OK) {
        return GPS_ERROR;
    }

    return GPS_OK;
}

void GPS_ChecksumUBLOX(uint8_t *data) {
    unsigned i, j;
    uint8_t a = 0, b = 0;

    j = ((unsigned) data[4] + ((unsigned) data[5] << 8) + 6);

    for (i = 2; i < j; i++) {
        a += data[i];
        b += a;
    }
    data[i] = a;
    data[++i] = b;
}

/* Pass uint8_t of received data */
void GPS_Receive(uint8_t data) {
    if (data == '$') {
        gpsTempLen = 0;
    } else if ((data == 13 || data == 10) && gpsTempLen != 255) { // looks for new_line or vertical tab
        GPS_Parse(gpsTemp, gpsTempLen);
        gpsTempLen = 255;
    } else if (gpsTempLen != 255) {
        gpsTemp[gpsTempLen] = data;
        gpsTempLen++;
    }
}

uint8_t GPS_parse_comma_delimited_str(uint8_t *string, uint8_t **fields, uint8_t max_fields) {
    uint8_t i = 0;
    fields[i++] = string;
    while ((i < max_fields) && NULL != (string = strchr((char*) string, ','))) {
        *string = '\0';
        fields[i++] = ++string;
    }
    return --i;
}

/* parses received frame */
GPS_StatusTypeDef GPS_Parse(uint8_t *buf, uint8_t len) {
    if (strncmp("GPGGA", (char*) buf, 5) == 0) { // get coordinates and height
        if (GPS_CheckSum(buf, len) == GPS_OK) {
            uint8_t step = 0;
            uint8_t i = 0;
            while (step < 8) {
                if (buf[i] == ',') {
                    i++;
                    step++;
                    if (step == 1) {
                        uint8_t tempData[6] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsTime, '0', 6);
                        memcpy(gpsTime + (6 - leng), tempData, leng);
                    } else if (step == 2 || step == 3) {
                        uint8_t tempData[12] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        i++;
                        if (buf[i] == 'N' || buf[i] == 'S' || buf[i] == 'E' || buf[i] == 'W') {
                            memset((gpsData + (step - 2) * 12), '0', 12);
                            gpsData[(step - 2) * 12 + 11] = buf[i];
                        } else {
                            return GPS_NOK;
                        }
                        memcpy(gpsData + (step - 2) * 12 + (11 - leng), tempData, leng);
                    } else if (step == 7) {
                        i++;
                        uint8_t tempData[12] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsHeight, '0', 8);
                        memcpy(gpsHeight + (8 - leng), tempData, leng);
                    }
                }
                i++;
            }
            isNewData = 1;
            isData = 1;
            return GPS_OK;
        } else {
            return GPS_NOK;
        }
    } else if (strncmp("GPVTG", (char*) buf, 5) == 0) { // get speed in km/h
        if (GPS_CheckSum(buf, len) == GPS_OK) {
            uint8_t step = 0;
            uint8_t i = 0;
            while (step < 8) {
                if (buf[i] == ',') {
                    step++;
                    if (step == 7) {
                        i++;
                        uint8_t tempData[12] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsSpeed, '0', 7);
                        memcpy(gpsSpeed + (7 - leng), tempData, leng);
                    }
                }
                i++;
            }
            return GPS_OK;
        } else {
            return GPS_NOK;
        }
    } else if (strncmp("GPZDA", (char*) buf, 5) == 0) { // get precise time
        if (GPS_CheckSum(buf, len) == GPS_OK) {
            uint8_t step = 0;
            uint8_t i = 0;
            while (step < 6) {
                if (buf[i] == ',') {
                    step++;
                    if (step == 1) {
                        uint8_t tempData[8] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsTime_UTC, '0', 9);
                        memcpy(gpsTime_UTC + (8 - leng), tempData, leng);
                    } else if (step == 2) {
                        uint8_t tempData[2] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsDate, '0', 3);
                        memcpy(gpsDate + (2 - leng), tempData, leng);
                    } else if (step == 3) {
                        uint8_t tempData[2] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsMonth, '0', 3);
                        memcpy(gpsMonth + (2 - leng), tempData, leng);
                    } else if (step == 4) {
                        uint8_t tempData[4] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(gpsYear, '0', 5);
                        memcpy(gpsYear + (4 - leng), tempData, leng);
                    } else if (step == 5) {
                        uint8_t tempData[2] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(local_zone_desc, '0', 3);
                        memcpy(local_zone_desc + (2 - leng), tempData, leng);
                    } else if (step == 5) {
                        uint8_t tempData[2] = { 0 };
                        uint8_t leng = 0;
                        while (buf[i] != ',') {
                            tempData[leng] = buf[i];
                            leng++;
                            i++;
                        }
                        if (leng == 0) {
                            return GPS_NOK;
                        }
                        memset(local_zone_min_desc, '0', 3);
                        memcpy(local_zone_min_desc + (2 - leng), tempData, leng);
                    }
                }
                i++;
            }
            return GPS_OK;
        } else {
            return GPS_NOK;
        }
    } else {
        return GPS_NOK;
    }
    return GPS_NOK;
}

/* checks checksum returns GPS_ok if data is valid*/
GPS_StatusTypeDef GPS_CheckSum(uint8_t *buf, uint8_t len) {
    uint8_t sum;
    uint8_t check = 0;

    if (GPS_HexToByte((buf + len - 2), &sum) != GPS_OK) {
        return GPS_NOK;
    }

    for (uint8_t i = 0; i < (len - 3); i++) {
        check ^= buf[i];
    }

    if (check == sum) {
        return GPS_OK;
    }

    return GPS_NOK;

}

uint8_t GPS_message_checksum(char *message, uint16_t size) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < size; i++) {
        checksum ^= message[i];
    }
    return checksum;
}

/* converts hex string e.g. AB (0xAB) to byte value writes to value, returns GPS_OK if valid hex */
GPS_StatusTypeDef GPS_HexToByte(uint8_t *hex, uint8_t *value) {
    uint8_t temp[2];

    for (uint8_t i = 0; i < 2; i++) {
        temp[i] = hex[i];
        if (temp[i] > 47 && temp[i] < 58) {
            temp[i] -= 48;
        } else if (temp[i] > 64 && temp[i] < 71) {
            temp[i] -= 55;
        } else {
            return GPS_NOK;
        }
    }

    *value = (temp[0] << 4) + temp[1];

    return GPS_OK;
}

/* Returns latitude of length 12...9*/
void GPS_GetLat(uint8_t *buf) {
    isNewData = 0;
    for (uint8_t i = 1; i < 10; i++) {
        buf[i - 1] = gpsData[i];
    }
}

/* Returns longitude of length 12...9*/
void GPS_GetLon(uint8_t *buf) {
    isNewData = 0;
    for (uint8_t i = 1; i < 10; i++) {
        buf[i - 1] = gpsData[i + 12];
    }
}

/* Returns speed of length 6*/
void GPS_GetSpeed(uint8_t *buf) {
    isNewData = 0;
    for (uint8_t i = 0; i < 6; i++) {
        buf[i] = gpsSpeed[i];
    }
}

/* Returns height of length 8*/
void GPS_GetHei(uint8_t *buf) {
    isNewData = 0;
    uint8_t i = 0;
    for (; i < 8; i++) {
        buf[i] = gpsHeight[i];
    }
}

/* Returns time of length 9 in format "hh:mm:ss.msms" */
void GPS_GetTime(uint8_t *buf) {
    isNewData = 0;
    buf[2] = ':';
    buf[5] = ':';
    for (uint8_t i = 0; i < 2; i++) {
        buf[i] = gpsTime[i];
        buf[i + 3] = gpsTime[i + 2];
        buf[i + 6] = gpsTime[i + 4];
    }
}
void GPS_GetTimeFull(uint8_t *buf) {
    isNewData = 0;
    buf[2] = ':';
    buf[5] = ':';
    for (uint8_t i = 0; i < 2; i++) {
        buf[i] = gpsTime_UTC[i];
        buf[i + 3] = gpsTime_UTC[i + 2];
        buf[i + 6] = gpsTime_UTC[i + 4];
    }
    buf[8] = gpsTime_UTC[6];
    buf[9] = gpsTime_UTC[7];
    buf[10] = gpsTime_UTC[8];
}

/* Returns year of length 4 in format "yyyy" */
void GPS_GetYear(uint8_t *buf) {
    isNewData = 0;
    for (uint8_t i = 0; i < 5; i++) {
        buf[i] = gpsYear[i];
    }
}

/* Returns month of length 2 in format "xx" */
void GPS_GetMonth(uint8_t *buf) {
    isNewData = 0;
    for (uint8_t i = 0; i < 2; i++) {
        buf[i] = gpsMonth[i];
    }
}

/* Returns date of length 2 in format "xx" */
void GPS_GetDate(uint8_t *buf) {
    isNewData = 0;
    for (uint8_t i = 0; i < 2; i++) {
        buf[i] = gpsDate[i];
    }
}

/* Returns GPS_OK if got any data */
GPS_StatusTypeDef GPS_IsData() {
    if (isData) {
        isData = 0;
        return GPS_OK;
    } else {
        return GPS_NOK;
    }
}

/* Returns GPS_OK if got new data, clears on any get function */
GPS_StatusTypeDef GPS_IsNewData() {
    if (isNewData) {
        return GPS_OK;
    } else {
        return GPS_NOK;
    }
}
