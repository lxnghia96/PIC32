#include "task.h"
#include "usb_transmit_task.h"
#include "timers.h"
#include "ipc_fifo.h"
#include "ipc_interfaces.h"
#include "definitions.h"
#include "usb_handler.h"
#include "bootloader.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define CAN_FIFO_NOT_EMPTY            (1U << 0U)
#define TIMER_100MS_TRIGGER           (1U << 1U)
#define USB_TRANSMIT_TIMER_PERIOD_MS  (10U)
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the USB_TRANSMIT_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/


xTimerHandle usb_timerHandle10Ms;
xTimerHandle usb_timerHandle100Ms;
static uint8_t numBytesWrite;
uint8_t USB_ALIGN cdcWriteBuffer[USB_TRANSMIT_DATA_BUFF_SIZE];
EventGroupHandle_t usbTransmitTaskEvents;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void usb_TimerCallback(xTimerHandle xTimer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xTimer == usb_timerHandle10Ms)
    {
        if (Ipc_BufferAvailable(&ipcCanFifo) > 0)
        {
            /* Notify BC FIFO has data */
            xEventGroupSetBitsFromISR( usbTransmitTaskEvents,
                                        CAN_FIFO_NOT_EMPTY,
                                        &xHigherPriorityTaskWoken);
        }
    }

    if (xTimer == usb_timerHandle100Ms)
    {
        xEventGroupSetBitsFromISR( usbTransmitTaskEvents,
                                        TIMER_100MS_TRIGGER,
                                        &xHigherPriorityTaskWoken);
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void USB_TRANSMIT_TASK_Initialize ( void )

  Remarks:
    See prototype in usb_transmit_task.h.
 */

void USB_TRANSMIT_TASK_Initialize ( void )
{
    usb_timerHandle10Ms = xTimerCreate("timer10Ms",
                                        pdMS_TO_TICKS(USB_TRANSMIT_TIMER_PERIOD_MS),
                                        pdTRUE,
                                        (void*)0,
                                        usb_TimerCallback);

    usb_timerHandle100Ms = xTimerCreate("timer100Ms",
                                        pdMS_TO_TICKS(100U),
                                        pdTRUE,
                                        (void*)1,
                                        usb_TimerCallback);

    usbTransmitTaskEvents = xEventGroupCreate();
}

/******************************************************************************
  Function:
    void USB_TRANSMIT_TASK_Tasks ( void )

  Remarks:
    See prototype in usb_transmit_task.h.
 */
void USB_TRANSMIT_TASK_Tasks ( void )
{
    EventBits_t uxEvents;
    EventBits_t uxEventsUsbCdc;

    ipc_message_t ipcMsg;
    uint8_t retVal;

    USB_DEVICE_CDC_TRANSFER_HANDLE transferHandle;
    transferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    xTimerStart(usb_timerHandle10Ms, 0U);
    xTimerStart(usb_timerHandle100Ms, 0U);

    while (1)
    {
        /* Wait events */
        uxEvents = xEventGroupWaitBits( usbTransmitTaskEvents,
                                        CAN_FIFO_NOT_EMPTY | TIMER_100MS_TRIGGER,
                                        pdTRUE,
                                        pdFALSE,
                                        portMAX_DELAY );
        /* If there is an event coming  */
        if ((uxEvents & CAN_FIFO_NOT_EMPTY) != USB_HANDLER_NO_EVENT)
        {
            /* Read and send all data in queue */
            do
            {
                xSemaphoreTake(xMutexIpcFifo, portMAX_DELAY);
                /* Read CAN Fifo buffer */
                retVal = Ipc_FifoRead(&ipcCanFifo, &ipcMsg);
                xSemaphoreGive(xMutexIpcFifo);
                LED_RED_Toggle();
                if (SUCCESS == retVal)
                {
                    /* Pack IPC message to buffer */
                    numBytesWrite = Ipc_PackMsgToBuffer(cdcWriteBuffer, ipcMsg);

                    if ((true == usb_isConfigured) && (ipcHeartBeat.pcConnected == true))
                    {
                        if ((numBytesWrite > 0U) && (numBytesWrite < USB_TRANSMIT_DATA_BUFF_SIZE))
                        {
                            transferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
                            /* Send buffer to usb cdc */
                            USB_DEVICE_CDC_Write
                                    (
                                    USB_DEVICE_CDC_INDEX_0,
                                    &transferHandle,
                                    cdcWriteBuffer, 
                                    numBytesWrite,
                                    USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE
                                    );

                            /* Wait for usb transmit complete */
                            uxEventsUsbCdc = xEventGroupWaitBits( usbHandleEventsGroup,
                                                    RTOS_EVENT_USB_TRANSMIT_COMPLETE,
                                                    pdTRUE,
                                                    pdFALSE,
                                                    pdMS_TO_TICKS(10));

                            /* If timeout => pc is disconnect */
                            if ((uxEventsUsbCdc & RTOS_EVENT_USB_TRANSMIT_COMPLETE) == 0U)
                            {
                                ipcHeartBeat.pcConnected = false;
                            }
                            else
                            {
                                Bld_resetReady(cdcWriteBuffer,numBytesWrite);
                            }
                            
                        }
                    }
                    else
                    {
                       transferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
                       ipcHeartBeat.pcConnected = false;
                    }
                }
            } while (SUCCESS == retVal);
        }

        /* If event 100MS */
        if ((uxEvents & TIMER_100MS_TRIGGER) != 0U)
        { 
            LED_YELLOW_Toggle();
            /* Every 100ms, check if usb is configured and pc is connected */
            /* Then send heartbeat to PC Core */
            if ((true == usb_isConfigured) && (true == ipcHeartBeat.pcConnected))
            {
                numBytesWrite = ipcHeartBeat.data_length;
                if ((numBytesWrite > 0U) && (numBytesWrite < USB_TRANSMIT_DATA_BUFF_SIZE))
                {
                    memcpy(cdcWriteBuffer, ipcHeartBeat.data, numBytesWrite);
                    transferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
                    /* If heartbeat is received, re-send heartbeat to PC Core */
                    if (ipcHeartBeat.isReceived == true)
                    {
                        ipcHeartBeat.isReceived = false;
                        /* Send heart beat to CPU Core */
                        USB_DEVICE_CDC_Write
                                        (
                                        USB_DEVICE_CDC_INDEX_0,
                                        &transferHandle,
                                        cdcWriteBuffer, 
                                        numBytesWrite,
                                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE
                                        );
                    }

                    /* Wait for usb transmit complete */
                    uxEventsUsbCdc = xEventGroupWaitBits( usbHandleEventsGroup,
                                            RTOS_EVENT_USB_TRANSMIT_COMPLETE,
                                            pdTRUE,
                                            pdFALSE,
                                            pdMS_TO_TICKS(10));

                    /* if timeout => pc is disconnect */
                    if ((uxEventsUsbCdc & RTOS_EVENT_USB_TRANSMIT_COMPLETE) == 0U)
                    {
                        ipcHeartBeat.pcConnected = false;
                    }
                    
                }
            }
            else
            {
                ipcHeartBeat.pcConnected = false;
            }
        }
    }
}


/*******************************************************************************
 End of File
 */
