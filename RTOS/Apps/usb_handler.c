// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "FreeRTOS.h"
#include "event_groups.h"

#include "usb_handler.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
EventGroupHandle_t  usbHandleEventsGroup;

// *****************************************************************************

/** \brief This mean USB CDC Device is configured */
#define USB_HANDLER_DEVICE_CONFIGURE          (1U)
/** \brief Size of queue which used to hold USB Event */
#define USB_HANDLER_QUEUE_SIZE                (15U)
/** \brief Initial value of USB speed (bps) */
#define USB_HANDLER_LINE_CODING_DTE_RATE      (9600U)
/** \brief Initial value of Stop Bit */
#define USB_HANDLER_LINE_CODING_STOP_BIT      (USB_CDC_LINE_CODING_STOP_1_BIT)
/** \brief Initial value of Parity Bit */
#define USB_HANDLER_LINE_CODING_PARITY_BIT    (USB_CDC_LINE_CODING_PARITY_NONE)
/** \brief Initial value of Data Bit */
#define USB_HANDLER_LINE_CODING_DATA_BIT      (USB_CDC_LINE_CODING_DATA_8_BIT)
/** \brief <b> Application Data </b> <br>
   \b Summary:
    Holds application data <br>
   \b Description:
    This structure holds the application's data. <br>
   \b Remarks:
    This structure should be initialized by the USB_HANDLER_Initialize function.
    Application strings and buffers are be defined outside this structure.
*/
USB_HANDLER_DATA usb_handlerData;
/** \brief
 * Create a \b QueueHandle instance to store events which created by USB CDC Event
 */
QueueHandle_t USBDeviceTask_EventQueue_Handle;

volatile bool usb_isConfigured = false;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/**
 * \brief 
 * @name 
 * @param[in]     event USB Device CDC Function Driver Index
 * @param[in]     pData 
 * @param[inout]  context
 * 
 * @return \b void
 */
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * pData,
                               uintptr_t context);
/**
 * \brief 
 * @name 
 * @param[in]    index USB Device CDC Function Driver Index
 * @param[in]    event USB Device CDC Function Driver Events
 *                    * <b> USB_DEVICE_CDC_EVENT </b>
 *                      + USB <br>
 * @param[in]    pData
 * @param[inout] userData
 * 
 * @return USB_DEVICE_CDC_EVENT_RESPONSE \b None
 */
USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void* pData,
    uintptr_t userData
);
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/************************************************
 * CDC Function Driver Application Event Handler
 ************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index,
    USB_DEVICE_CDC_EVENT event,
    void* pData,
    uintptr_t userData
)
{
    USB_HANDLER_DATA * appDataObject;
    appDataObject = (USB_HANDLER_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    portBASE_TYPE xHigherPriorityTaskWoken1 = pdFALSE;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_GET_LINE_CODING <br>
             * This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.
             */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->appCOMPortObjects[index].getLineCodingData,
                    sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_SET_LINE_CODING <br>
             * This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host
             */
            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->appCOMPortObjects[index].setLineCodingData,
                    sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE <br>
             * Read the control line state. We will accept this request for now.
             */
            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->appCOMPortObjects[index].controlLineStateData.dtr
                    = controlLineStateData->dtr;
            appDataObject->appCOMPortObjects[index].controlLineStateData.carrier
                    = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_SEND_BREAK <br>
             * This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration
             */
            // breakData = (uint16_t *)pData;
            appDataObject->appCOMPortObjects[index].breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_READ_COMPLETE <br>
             * This means that the USB device received data from USB Host. Read
             * data from USB stream and save it into user buffer
             */
            appDataObject->numBytesRead = ((USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*)pData)->length;
            cdcReadDataLength = ((USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*)pData)->length;

            /* Notify Usb receive Done */
            xEventGroupSetBitsFromISR( usbHandleEventsGroup,
                                        RTOS_EVENT_USB_RECEIVE_COMPLETE,
                                        &xHigherPriorityTaskWoken1);

            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED <br>
             * The data stage of the last control transfer is
             * complete. For now we accept all the data
             */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT <br>
             * This means the GET LINE CODING function data is valid.
             */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:
            /**
             * \brief When event is \b USB_DEVICE_CDC_EVENT_WRITE_COMPLETE <br>
             * Notify USB_TRANSMIT_TASK_Tasks when transmit request 
             */
            /* Notify USB_TRANSMIT_TASK_Tasks */
            xEventGroupSetBitsFromISR( usbHandleEventsGroup,
                                        RTOS_EVENT_USB_TRANSMIT_COMPLETE,
                                        &xHigherPriorityTaskWoken1);


            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
            break;

        default:
            break;
    }
    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/*************************************************
 * Application Device Layer Event Handler
 *************************************************/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * pData,
                               uintptr_t context)
{
    uint8_t configurationValue;
    uint32_t USB_Event = 0UL;
    portBASE_TYPE xHigherPriorityTaskWoken1 = pdFALSE;

/*! 
   *  A list of events:
   *  <ul>
   *  <li> <b> USB Device Event </b>
   *     <ol>
   *     <li> \b USB_DEVICE_EVENT_POWER_REMOVED <br>
   *             VBUS off, detach USB device
   *     <li> \b USB_DEVICE_EVENT_RESET <br>
   *             More info about the click event.
   *     <li> \b USB_DEVICE_EVENT_DECONFIGURED
   *     <li> \b USB_DEVICE_EVENT_CONFIGURED
   *     <li> \b USB_DEVICE_EVENT_SOF
   *     <li> \b USB_DEVICE_EVENT_SUSPENDED
   *     <li> \b USB_DEVICE_EVENT_RESUMED
   *     <li> \b USB_DEVICE_EVENT_POWER_DETECTED <br>
   *             VBUS on, attach USB device
   *     <li> \b USB_DEVICE_EVENT_ERROR <br>
   *             This mean USB Device has an error, do error handling in here <br>
   *     </ol>
   *  More text here.
   */
    switch( event )
    {
        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* Detach the device */
            USB_DEVICE_Detach(usb_handlerData.deviceHandle); 
            break;

        case USB_DEVICE_EVENT_RESET:
            usb_isConfigured = false;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* pData will point to the configuration. Check the configuration */
            configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED *)pData)->configurationValue;
            if (USB_HANDLER_DEVICE_CONFIGURE == configurationValue)
            {
                /**
                 * Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data
                 * @return \b USB_DEVICE_CDC_RESULT_OK \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_TRANSFER_SIZE_INVALID \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_TRANSFER_QUEUE_FULL \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_INSTANCE_INVALID \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_INSTANCE_NOT_CONFIGURED \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_PARAMETER_INVALID \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_ENDPOINT_HALTED \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR_TERMINATED_BY_HOST \n
                 *         \b USB_DEVICE_CDC_RESULT_ERROR
                 */
                USB_DEVICE_CDC_EventHandlerSet(0,
                        APP_USBDeviceCDCEventHandler, (uintptr_t)&usb_handlerData);

                usb_isConfigured = true;

                /*let processing USB Task know USB if configured..*/
                USB_Event = USB_HANDLER_TASK_USB_CONFIGURED_EVENT; 

                xQueueSendToBackFromISR(USBDeviceTask_EventQueue_Handle, &USB_Event, 
                    &xHigherPriorityTaskWoken1);
                portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
            }

            break;

        case USB_DEVICE_EVENT_SOF:

            /* Received SOF Event */
            USB_Event = USB_HANDLER_TASK_SOF_EVENT;

            xQueueSendToBackFromISR(USBDeviceTask_EventQueue_Handle, &USB_Event, 
                &xHigherPriorityTaskWoken1);
            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );

            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(usb_handlerData.deviceHandle);

            /* let processing USB Task know USB is powered..*/
            USB_Event = USB_HANDLER_TASK_USB_POWERED_EVENT;

            xQueueSendToBackFromISR(USBDeviceTask_EventQueue_Handle, &USB_Event, 
                &xHigherPriorityTaskWoken1);
            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************


void USB_HANDLER_Initialize ( void )
{
    USBDeviceTask_EventQueue_Handle = xQueueCreate(USB_HANDLER_QUEUE_SIZE, \
                                                    sizeof(uint32_t));

    /* dont proceed if queue was not created... */
    if(NULL == USBDeviceTask_EventQueue_Handle)
    {
        while(1);
    }
    /* Initialize the application object, user must modify usb_handler.h to define USB Configuration */
    usb_handlerData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    usb_handlerData.appCOMPortObjects[0].getLineCodingData.dwDTERate   = USB_HANDLER_LINE_CODING_DTE_RATE;
    usb_handlerData.appCOMPortObjects[0].getLineCodingData.bDataBits   = USB_HANDLER_LINE_CODING_DATA_BIT;
    usb_handlerData.appCOMPortObjects[0].getLineCodingData.bParityType = USB_HANDLER_LINE_CODING_PARITY_BIT;
    usb_handlerData.appCOMPortObjects[0].getLineCodingData.bCharFormat = USB_HANDLER_LINE_CODING_STOP_BIT;
    /* Create events group handler */
    usbHandleEventsGroup = xEventGroupCreate();
}


/******************************************************************************
  Function:
    void USB_HANDLER_Tasks ( void )

  Remarks:
    See prototype in usb_handler.h.
 */
void USB_HANDLER_Tasks ( void )
{
    BaseType_t errStatus;
    uint32_t USBDeviceTask_State = USB_HANDLER_TASK_OPEN_USB_STATE;
    uint32_t USBDeviceTask_Event = 0UL;
    USB_DEVICE_CDC_TRANSFER_HANDLE COM1Read_Handle;

    COM1Read_Handle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    for(;;)
    {
        switch(USBDeviceTask_State)
        {
            case USB_HANDLER_TASK_OPEN_USB_STATE:
                /* Open the device layer */
                usb_handlerData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );
                /* Check if USB Device stack returned a valid handle */
                if(USB_DEVICE_HANDLE_INVALID != usb_handlerData.deviceHandle)
                {
                    /* Register a callback with USB Device layer to get event 
                     * notification (for Endpoint 0) */
                    USB_DEVICE_EventHandlerSet(usb_handlerData.deviceHandle, APP_USBDeviceEventHandler, 0);
                    USBDeviceTask_State = USB_HANDLER_TASK_PROCESS_USB_EVENTS_STATE;
                }
                else
                {
                    /* Try again in 10 msec */
                    USBDeviceTask_State = USB_HANDLER_TASK_OPEN_USB_STATE;
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
                break;
            case USB_HANDLER_TASK_PROCESS_USB_EVENTS_STATE:
                /* Once here, USB task becomes event driven, user input will 
                  will generate events. */
                USBDeviceTask_State = USB_HANDLER_TASK_PROCESS_USB_EVENTS_STATE;

                /* Wait for an event to occur and process, see event handler*/
                errStatus = xQueueReceive(USBDeviceTask_EventQueue_Handle,
                                &USBDeviceTask_Event,portMAX_DELAY);

                /* Make sure event was successfully received*/
                if(pdFALSE == errStatus)
                    break;

                switch(USBDeviceTask_Event)
                {
                    case USB_HANDLER_TASK_USB_POWERED_EVENT:
                        break;

                    case USB_HANDLER_TASK_USB_CONFIGURED_EVENT:
                        if (true == usb_isConfigured)
                        {
                            /* USB ready, do the first read data from COM PORT */
                            (void) USB_DEVICE_CDC_Read
                            (
                                USB_DEVICE_CDC_INDEX_0, 
                                &COM1Read_Handle,
                                cdcReadBuffer,
                                USB_RECEIVE_APP_READ_BUFFER_SIZE
                            );
                        }
                        else
                        {
                            COM1Read_Handle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
                        }
                        break;

                    case USB_HANDLER_TASK_READ_DONE_EVENT:
                        break;

                    case USB_HANDLER_TASK_WRITE_DONE_EVENT:
                        break;

                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }
}


/*******************************************************************************
 End of File
 */
