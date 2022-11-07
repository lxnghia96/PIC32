#include <stdint.h>
#include "ipc.h"
#include <stddef.h>                     // Defines NULL
#include <string.h>                     

uint8_t Ipc_CalculateCrc(uint8_t *data, uint8_t len);
uint8_t Ipc_CalculateCrc(uint8_t *data, uint8_t len)
{
    uint8_t crcValue;

    uint8_t i;

    uint8_t bitNum;

    const uint8_t POLYNOMIAL = 0x31U;
    
    const uint8_t CRC_MASK = 0x80U;

    crcValue = 0U;

    /* Calculate CRC value */
    for (i = 0U; i < len; i++)
    {
        crcValue = data[i] ^ crcValue;

        for (bitNum = 8U; bitNum > 0U; bitNum--)
        {
            if (crcValue & CRC_MASK)
            {
                crcValue = (crcValue << 1U) ^ POLYNOMIAL;
            }
            else
            {
                crcValue = (crcValue << 1U);
            }
        }
    }

    crcValue = crcValue & 0xFF;

    /* Return CRC value */
    return crcValue ;
}


uint8_t Ipc_PackMessage(uint8_t SerId, uint8_t *rawData, uint32_t lengthOfRawData, uint8_t *ipcMes, uint32_t* lengthOfIpcMes)
{
    uint8_t retval;

    uint8_t crc;
    
    /*Parameters is OK*/
    if((rawData != NULL) && (ipcMes != NULL) && (lengthOfIpcMes != NULL) && (lengthOfRawData > 0))
    {   
        /*1 byte sid + 2 byte length + data + 1 byte crc*/
        *lengthOfIpcMes = lengthOfRawData + 4;

        /* Calculate the CRC of the response*/
        memset(ipcMes, 0U, *lengthOfIpcMes);

        ipcMes[0] = SERVICE_FW_UPDATE;

        ipcMes[1] = (uint8_t)(lengthOfRawData & 0xFFU);

        ipcMes[2] = (uint8_t)((lengthOfRawData >> 8U)  & 0xFFU);

        memcpy(&ipcMes[3], rawData, lengthOfRawData);

        /* Calculate  crc */
        crc = Ipc_CalculateCrc(ipcMes, *lengthOfIpcMes - 1 );

        ipcMes[*lengthOfIpcMes-1] = crc;

        retval = SUCCESS;
    }
    else
    {
        retval = ERROR;
    }
    return retval;
}

uint8_t retval(void);



