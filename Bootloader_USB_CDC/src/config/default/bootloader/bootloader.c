/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    bootloader.c

  Summary:
    Interface for the Bootloader library.

  Description:
    This file contains the interface definition for the Bootloader library.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <string.h>
#include "device.h"
#include "bootloader/bootloader.h"
#include "bootloader/datastream/datastream.h"
#include "bootloader/bootloader_nvm_interface.h"
#include "ipc.h"
#include <stdio.h>
#include <stdlib.h>

#define BOOTLOADER_BUFFER_SIZE  512

#define MAJOR_VERSION           3  // Bootloader Major Version Shown From a Read Version on PC
#define MINOR_VERSION           4  // Bootloader Minor Version Shown From a Read Version on PC


extern uint32_t BldNvm_appEndAddress;
extern uint32_t prevCrc; 

typedef enum
{
    READ_BOOT_INFO = 1,
    ERASE_FLASH,
    PROGRAM_FLASH,
    READ_CRC,
    JMP_TO_APP
} BOOTLOADER_COMMANDS;

typedef enum
{
    /* If we need to program, then open the datastream. */
    BOOTLOADER_OPEN_DATASTREAM = 0,

    /* The Bootloader gets a command from the host application. */
    BOOTLOADER_GET_COMMAND,

    /* The Bootloader processes the command from the host application. */
    BOOTLOADER_PROCESS_COMMAND,

    /* The Bootloader sends data back to the host application. */
    BOOTLOADER_SEND_RESPONSE,

    /* The Bootloader waits in this state for the driver to finish
       sending/receiving the message. */
    BOOTLOADER_WAIT_FOR_DONE,

    /* Close the datastream */
    BOOTLOADER_CLOSE_DATASTREAM,

    /* The Bootloader enters the user application. */
    BOOTLOADER_ENTER_APPLICATION,

    /* This state indicates an error has occurred. */
    BOOTLOADER_ERROR,

    BOOTLOADER_RESPONSE_ENTER_BOOTMODE,

} BOOTLOADER_STATES;

typedef union
{
    uint8_t buffer[BOOTLOADER_BUFFER_SIZE + BOOTLOADER_BUFFER_SIZE];

    struct
    {
        /* Buffer to hold the data received */
        uint8_t inputBuff[BOOTLOADER_BUFFER_SIZE];
        /* Buffer to hold the data to be processed */
        uint8_t procBuff[BOOTLOADER_BUFFER_SIZE];
    } buffers;
} BOOTLOADER_BUFFER;

typedef struct
{
    /* Bootloader current state */
    BOOTLOADER_STATES currentState;

    /* Bootloader previous state */
    BOOTLOADER_STATES prevState;

    /* Datastream buffer size */
    uint32_t buffSize;

    /* Command buffer length */
    uint32_t cmdBuffLen;

    /* Stream handle */
    DATASTREAM_HANDLE streamHandle;

    /* Datastream status */
    DRV_CLIENT_STATUS datastreamStatus;

    /* Flag to indicate the host message is been processed */
    bool usrBufferEventComplete;

} BOOTLOADER_DATA;

static const uint8_t BootInfo[2] =
{
    MAJOR_VERSION,
    MINOR_VERSION
};

BOOTLOADER_BUFFER CACHE_ALIGN dataBuff;

BOOTLOADER_DATA btlData =
{
    .currentState = BOOTLOADER_OPEN_DATASTREAM,
    .usrBufferEventComplete = false
};

bool __WEAK bootloader_Trigger(void)
{
    /* Function can be overriden with custom implementation */
    return false;
}

void bootloader_TriggerReset(void)
{
    /* Perform system unlock sequence */
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;

    RSWRSTSET = _RSWRST_SWRST_MASK;
    (void)RSWRST;
}

void run_Application(void)
{
    uint32_t msp            = *(uint32_t *)(APP_START_ADDRESS);

    void (*fptr)(void);

    /* Set default to APP_RESET_ADDRESS */
    fptr = (void (*)(void))APP_START_ADDRESS;

    if (msp == 0xffffffff)
    {
        return;
    }

    fptr();
}

static const uint16_t crc_table[16] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

static uint32_t bootloader_CalculateCrc(uint8_t *data, uint32_t len)
{   
    uint32_t i;
    uint16_t crc = 0;

    while(len--)
    {
        i = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }
    return (crc & 0xFFFF);
}

static void bootloader_BufferEventHandler
(
    DATASTREAM_BUFFER_EVENT buffEvent,
    DATASTREAM_HANDLE handle,
    uintptr_t context
)
{
    uint8_t crcExpected;
    uint8_t msgServiceId;
    uint16_t msgPayloadLen;
    uint8_t ignoreHandle;

    if (buffEvent != DATASTREAM_BUFFER_EVENT_COMPLETE)
    {
        ignoreHandle = 1;
    }
    else
    {
        ignoreHandle = 0;
    }

    if (btlData.prevState != BOOTLOADER_GET_COMMAND)
    {
        /* SEND RESPONSE */
        btlData.usrBufferEventComplete = true;
        return;
    }

    if(ignoreHandle == 0)
    {
        crcExpected = Ipc_CalculateCrc(dataBuff.buffers.inputBuff, context - 1 );

        /* Check crc */
        if( crcExpected == dataBuff.buffers.inputBuff[context-1] )
        {
            
            msgServiceId = dataBuff.buffers.inputBuff[0];

            msgPayloadLen = (uint16_t)((uint16_t)dataBuff.buffers.inputBuff[1] | dataBuff.buffers.inputBuff[2] << 8U);

            /* Check service id and ipc Payload length */
            if(msgServiceId == SERVICE_FW_UPDATE && msgPayloadLen == (context - 4))
            {
                btlData.cmdBuffLen = msgPayloadLen;
                
                /* Default data */
                memset(dataBuff.buffers.procBuff, 0x00, btlData.cmdBuffLen);

                /* Get and copy data into buffer */
                memcpy(dataBuff.buffers.procBuff, &dataBuff.buffers.inputBuff[3], btlData.cmdBuffLen);
                    
                btlData.usrBufferEventComplete = true;
                
                return;
            }
        }
    }

    DATASTREAM_Data_Read(btlData.streamHandle, dataBuff.buffers.inputBuff, btlData.buffSize);
}

static void bootloader_ProcessBuffer( BOOTLOADER_DATA *handle )
{
    uint8_t Cmd;
    uint32_t Length;
    uint16_t crc;
    
    /* First, check that we have a valid command. */
    Cmd = dataBuff.buffers.procBuff[0];

    /* Build the response frame from the command. */
    dataBuff.buffers.inputBuff[0] = dataBuff.buffers.procBuff[0];
    handle->buffSize = 0;

    switch (Cmd)
    {
        case READ_BOOT_INFO:
        {
            memcpy(&dataBuff.buffers.inputBuff[1], BootInfo, 2);
            handle->buffSize = 2 + 1;
            handle->currentState = BOOTLOADER_SEND_RESPONSE;
            break;
        }

        case ERASE_FLASH:
        {
            bootloader_NvmAppErase();
            handle->currentState = BOOTLOADER_SEND_RESPONSE;
            handle->buffSize = 1;
            break;
        }

        case PROGRAM_FLASH:
        {
            if(bootloader_NvmProgramHexRecord(&dataBuff.buffers.procBuff[1], handle->cmdBuffLen-1) != HEX_REC_NORMAL)
            {
                break;
            }
            handle->buffSize = 1;
            handle->currentState = BOOTLOADER_SEND_RESPONSE;
            break;
        }

        case READ_CRC:
        {
            /* Calculate size of firmware */
            Length = BldNvm_appEndAddress - APP_START_ADDRESS;
            
            /* Check sum of firmware in flash */
            crc = bootloader_CalculateCrc((uint8_t *)(APP_START_ADDRESS), Length);

            /* Check sum after and before is equal */
            if((crc & 0xFFFF) == (prevCrc & 0xFFFF))
            {
                
                memcpy(&dataBuff.buffers.inputBuff[1], &crc, 2);

                handle->buffSize = 1;

                handle->currentState = BOOTLOADER_SEND_RESPONSE;
            }
            else
            {
                handle->currentState = BOOTLOADER_GET_COMMAND;
            }
            
            break;
        }

        case JMP_TO_APP:
        {
            handle->buffSize = 1;
            handle->prevState = BOOTLOADER_ENTER_APPLICATION;
            handle->currentState = BOOTLOADER_SEND_RESPONSE;

            break;
        }

        default:
            break;
    }
}

uint8_t  bldRespIntoBldMode = BLD_RESPONSE_INTO_BOOT_MODE;
uint8_t  bldRespMes[BLD_RESPONSE_MAX_SIZE];
uint32_t lengthOfRespMes;

void bootloader_Tasks( void )
{
    uint32_t BuffLen=0;
    switch ( btlData.currentState )
    {
        case BOOTLOADER_OPEN_DATASTREAM:
        {
            btlData.streamHandle = DATASTREAM_HANDLE_INVALID;

            btlData.streamHandle = DATASTREAM_Open(
                    DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);

            if (btlData.streamHandle != DRV_HANDLE_INVALID )
            {
                DATASTREAM_BufferEventHandlerSet(btlData.streamHandle,
                        bootloader_BufferEventHandler, APP_USR_CONTEXT);

                btlData.currentState = BOOTLOADER_GET_COMMAND;
            }
            break;
        }
               
        case BOOTLOADER_GET_COMMAND:
        {
            btlData.datastreamStatus = DRV_CLIENT_STATUS_ERROR;

            /* Get the data stream driver status */
            btlData.datastreamStatus = DATASTREAM_ClientStatus( btlData.streamHandle );

            /* Check if client is ready or not */
            if ( btlData.datastreamStatus == DRV_CLIENT_STATUS_READY )
            {
                
                btlData.buffSize = BOOTLOADER_BUFFER_SIZE;

                DATASTREAM_Data_Read( btlData.streamHandle, dataBuff.buffers.inputBuff, btlData.buffSize);

                /* Set the App. state to wait for done */
                btlData.prevState    = BOOTLOADER_GET_COMMAND;
                btlData.currentState = BOOTLOADER_WAIT_FOR_DONE;
            }
            break;
        }
        
        case BOOTLOADER_WAIT_FOR_DONE:
        {
            /* check if the data stream buffer event is complete or not */
            if (btlData.usrBufferEventComplete == true)
            {
                btlData.usrBufferEventComplete = false;

                /* Get the next App. State */
                if (btlData.prevState == BOOTLOADER_GET_COMMAND)
                {
                    btlData.currentState = BOOTLOADER_PROCESS_COMMAND;
                }
                else if (btlData.prevState == BOOTLOADER_ENTER_APPLICATION)
                {
                    btlData.currentState = BOOTLOADER_ENTER_APPLICATION;
                }
                else
                {
                    btlData.currentState = BOOTLOADER_GET_COMMAND;
                }
            }
               
            break;
        }

        case BOOTLOADER_PROCESS_COMMAND:
        {
            bootloader_ProcessBuffer(&btlData);
            break;
        }

        case BOOTLOADER_SEND_RESPONSE:
        {
            if(btlData.buffSize)
           {                
                Ipc_PackMessage(SERVICE_FW_UPDATE, dataBuff.buffers.inputBuff, btlData.buffSize, dataBuff.buffers.procBuff, &BuffLen);

                btlData.buffSize = 0;

                DATASTREAM_Data_Write( btlData.streamHandle, dataBuff.buffers.procBuff, BuffLen);
                   
                if (btlData.prevState != BOOTLOADER_ENTER_APPLICATION)
                {
                    btlData.prevState = BOOTLOADER_SEND_RESPONSE;
                }

                    btlData.currentState = BOOTLOADER_WAIT_FOR_DONE;
            }
            break;
        }

        case BOOTLOADER_ENTER_APPLICATION:
        {
            bootloader_TriggerReset();
            break;
        }

        case BOOTLOADER_ERROR:
        default:
            btlData.currentState = BOOTLOADER_ERROR;
            break;
    }

    /* Maintain Device Drivers */
    DATASTREAM_Tasks();
}

