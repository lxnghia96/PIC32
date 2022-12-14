/*******************************************************************************
 Data Stream USB_DEVICE_HID Source File

  File Name:
    datastream_usb_device_hid.c

  Summary:
    Data Stream USB_DEVICE_HID source

  Description:
    This file contains source code necessary for the data stream interface.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#include "bootloader/datastream/datastream.h"
#include "usb/usb_device_cdc.h"

static uint8_t cdcReadDataLength;
typedef struct
{
    USB_DEVICE_HANDLE usbHandle;
    USB_DEVICE_CDC_TRANSFER_HANDLE transferHandle;
    /** Set Line Coding Data */
    USB_CDC_LINE_CODING setLineCodingData;
    /** Get Line Coding Data */
    USB_CDC_LINE_CODING getLineCodingData;
    /** Control Line State */
    USB_CDC_CONTROL_LINE_STATE controlLineStateData;

    /** Device layer handle returned by device layer open function */
    DATASTREAM_HandlerType handler;

    eDIR currDir;
    bool deviceConfigured;
    bool readRequest;
    bool DataSent;
    bool DataReceived;
    uint8_t *rxBuffer;
    uint32_t rxMaxSize;
    uint32_t rxCurSize;
    uint8_t *txBuffer;
    uint32_t txMaxSize;
    uint32_t txCurPos;
    uintptr_t context;
} USB_CDC_DATA;

USB_CDC_DATA usbCDCData;

static USB_DEVICE_CDC_EVENT_RESPONSE DATASTREAM_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX iCDC,
    USB_DEVICE_CDC_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    
    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_GET_LINE_CODING <br>
             * This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.
             */

            USB_DEVICE_ControlSend(usbCDCData.usbHandle, &usbCDCData.getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_SET_LINE_CODING <br>
             * This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host
             */
            USB_DEVICE_ControlReceive(usbCDCData.usbHandle, &usbCDCData.setLineCodingData, sizeof(USB_CDC_LINE_CODING));
            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE <br>
             * Read the control line state. We will accept this request for now.
             */
            USB_DEVICE_ControlStatus(usbCDCData.usbHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:
            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(usbCDCData.usbHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED <br>
             * The data stage of the last control transfer is
             * complete. For now we accept all the data
             */
            USB_DEVICE_ControlStatus(usbCDCData.usbHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:
            usbCDCData.DataSent = true;
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            cdcReadDataLength = ((USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*)eventData)->length;
            usbCDCData.DataReceived = true;
            break;

        default:
            // Nothing to do.
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

static void DATASTREAM_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */
            usbCDCData.deviceConfigured = false;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Set the flag indicating device is configured. */
            usbCDCData.deviceConfigured = true;

            /* Register application CDC event handler */
            USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, DATASTREAM_USBDeviceCDCEventHandler, (uintptr_t)&usbCDCData);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach (usbCDCData.usbHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* VBUS is not available */
            USB_DEVICE_Detach(usbCDCData.usbHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

void DATASTREAM_Tasks(void)
{
    if (false == usbCDCData.deviceConfigured)
    {
        return;
    }

    if (RX == usbCDCData.currDir)
    {
        if (usbCDCData.DataReceived == true)
        {
            usbCDCData.DataReceived = false;
            usbCDCData.readRequest = false;
            usbCDCData.currDir = IDLE;
            usbCDCData.handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_HANDLE)usbCDCData.usbHandle, cdcReadDataLength);
        }
        else if (usbCDCData.readRequest == false)
        {
            usbCDCData.DataReceived = false;

            /* Place a new read request. */
            USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                &usbCDCData.transferHandle, usbCDCData.rxBuffer, 64 );

            usbCDCData.readRequest = true;
        }
    }
    else if (TX == usbCDCData.currDir)
    {
        if ( (usbCDCData.DataSent == false) &&
             (usbCDCData.readRequest == false) &&
             (usbCDCData.txCurPos < usbCDCData.txMaxSize))
        {
            /* Prepare the USB module to send the data packet to the host */
            USB_DEVICE_CDC_Write (USB_DEVICE_CDC_INDEX_0,
                                &usbCDCData.transferHandle,
                                (usbCDCData.txBuffer + usbCDCData.txCurPos),
                                usbCDCData.txMaxSize,
                                USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

            usbCDCData.txCurPos += usbCDCData.txMaxSize;
            usbCDCData.readRequest = true;
        }
        else if ( (usbCDCData.DataSent == true) &&
                  (usbCDCData.txCurPos >= usbCDCData.txMaxSize)) // All data has been sent or is in the buffer
        {
            usbCDCData.readRequest = false;
            usbCDCData.currDir = IDLE;
            usbCDCData.DataSent = false;
            usbCDCData.handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_HANDLE)usbCDCData.usbHandle, usbCDCData.context);
        }
        else if (usbCDCData.DataSent == true)
        {
            usbCDCData.readRequest = false;
            usbCDCData.DataSent = false;
        }
    }
}

DATASTREAM_HANDLE DATASTREAM_Open(const DRV_IO_INTENT ioIntent)
{
    usbCDCData.usbHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

    if(usbCDCData.usbHandle != USB_DEVICE_HANDLE_INVALID)
    {
        /* This means host operation is enabled. We can
        * move on to the next state */
        USB_DEVICE_EventHandlerSet(usbCDCData.usbHandle, DATASTREAM_USBDeviceEventHandler, 0);
    }

    return((DATASTREAM_HANDLE)usbCDCData.usbHandle);
}

void DATASTREAM_Close(void)
{
    if (usbCDCData.deviceConfigured)
    {
        USB_DEVICE_Detach(usbCDCData.usbHandle);
        USB_DEVICE_Close(usbCDCData.usbHandle);
    }
    USB_DEVICE_Deinitialize(usbCDCData.usbHandle);
}

DRV_CLIENT_STATUS DATASTREAM_ClientStatus(DATASTREAM_HANDLE handle)
{
    if(usbCDCData.deviceConfigured == true)
    {
        return DRV_CLIENT_STATUS_READY;
    }
    else
    {
        return DRV_CLIENT_STATUS_BUSY;
    }
}

int DATASTREAM_Data_Read(DATASTREAM_HANDLE handle, uint8_t *buffer, const uint32_t rxSize)
{
    usbCDCData.rxBuffer = buffer;
    usbCDCData.rxMaxSize = rxSize;
    usbCDCData.rxCurSize = 0;
    usbCDCData.currDir = RX;
    return(0);
}

int DATASTREAM_Data_Write(DATASTREAM_HANDLE handle, uint8_t *buffer, const uint32_t txSize)
{
    usbCDCData.txBuffer = buffer;
    usbCDCData.txMaxSize = txSize;
    usbCDCData.txCurPos = 0;
    usbCDCData.currDir = TX;
    return(0);
}

void DATASTREAM_BufferEventHandlerSet
(
    const DATASTREAM_HANDLE hClient,
    const void * eventHandler,
    const uintptr_t context
)
{
    usbCDCData.handler = (DATASTREAM_HandlerType)eventHandler;
    usbCDCData.context = context;
}
