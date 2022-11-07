// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "usb_receive_task.h"
#include "definitions.h"
#include "ipc_interfaces.h"
#include "usb_handler.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the USB_RECEIVE_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

volatile uint8_t    cdcReadDataLength;
 /** \brief Hold the <b> data read from USB Host </b> */
uint8_t CACHE_ALIGN cdcReadBuffer[USB_RECEIVE_APP_READ_BUFFER_SIZE];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

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
    void USB_RECEIVE_TASK_Initialize ( void )

  Remarks:
    See prototype in usb_receive_task.h.
 */

void USB_RECEIVE_TASK_Initialize ( void )
{
}


/******************************************************************************
  Function:
    void USB_RECEIVE_TASK_Tasks ( void )

  Remarks:
    See prototype in usb_receive_task.h.
 */

void USB_RECEIVE_TASK_Tasks ( void )
{
    EventBits_t uxEvents;
    uint8_t ipcRetVal;
    uint8_t tempReadBuffer[USB_RECEIVE_APP_READ_BUFFER_SIZE];
    uint8_t dataLen = 0U;
    USB_DEVICE_CDC_TRANSFER_HANDLE transferHandle;
    transferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    USB_DEVICE_CDC_RESULT readRequestResult;

    while (1)
    {
        /* Wait for an event from usb handler */
        uxEvents = xEventGroupWaitBits( usbHandleEventsGroup,
                                        RTOS_EVENT_USB_RECEIVE_COMPLETE,
                                        pdTRUE,
                                        pdFALSE,
                                        portMAX_DELAY );
        /* Check event occur */
        if ((uxEvents & RTOS_EVENT_USB_RECEIVE_COMPLETE) !=\
            USB_HANDLER_NO_EVENT)
        {
            /* Copy new data */
            dataLen = cdcReadDataLength;
            memcpy(tempReadBuffer, cdcReadBuffer, dataLen);

            /* Ipc handling data */
            ipcRetVal = Ipc_HandleMsgFromUsbCdc(tempReadBuffer, dataLen);
            if ( SUCCESS != ipcRetVal )
            {
              /* Do error handling */
            }
            else
            {
                /* Clear buffer */
                memset(tempReadBuffer, 0, USB_RECEIVE_APP_READ_BUFFER_SIZE);
            }
            if (true == usb_isConfigured)
            {
                readRequestResult = 
                    USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                                  &transferHandle,
                                  cdcReadBuffer,
                                  USB_RECEIVE_APP_READ_BUFFER_SIZE);
            }
            else
            {
                transferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            }
            if ( ( USB_DEVICE_CDC_RESULT_OK != readRequestResult ) ||\
                ( USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID == transferHandle ) )
            {
                /* Do error handling */
            }
        }
    }
}


/*******************************************************************************
 End of File
 */
