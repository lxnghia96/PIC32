// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "Basic_can_receive_task.h"
#include "Basic_can.h"
#include "ipc_interfaces.h"
#include "Basic_can_handle_errors.h"
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
    This structure should be initialized by the BC_CAN_RECEIVE_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

EventGroupHandle_t BcCan_ModeEventGroup;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void BC_CAN_RECEIVE_TASK_Initialize ( void )

  Remarks:
    See prototype in bc_can_receive_task.h.
 */

void BC_CAN_RECEIVE_TASK_Initialize ( void )
{
}


/******************************************************************************
  Function:
    void BC_CAN_RECEIVE_TASK_Tasks ( void )

  Remarks:
    See prototype in bc_can_receive_task.h.
 */

void BC_CAN_RECEIVE_TASK_Tasks ( void )
{
    /* Variable to hold error occur */
    uint8_t bcError;

    /* Variable to hold status */
    bc_status_t bcStatus;

    /* Declare variable to hold message length */
    uint8_t msgLen;

    /* Declare variable to hold message data */
    uint8_t rx_message[BC_LENGTH];

    /* Declare variable to hold message id */
    uint32_t rx_messageID;

    /* Declare variable to hold message payload */
    bc_payload_t bcPayLoad;

    while (1)
    {
        /* Ready to receive a new message */
        bcStatus = BcCan_ReceiveFromCanBus(&rx_messageID, &msgLen, rx_message);

        /* Receive is complete */
        if(bcStatus == BC_OK)
        {
            /* Check integrity feature of message */
            bcError = BcCan_CheckDataIntegrity(rx_messageID, rx_message, msgLen);

            /* Message has no error */
            if(bcError == BC_NO_ERROR)
            {

                /* Pack IPC payload */
                bcPayLoad = BcCan_PackPayload(rx_messageID, rx_message, msgLen);

                xSemaphoreTake(xMutexIpcFifo, portMAX_DELAY);
                
                /* Pack IPC message and push into queue */
                Ipc_PackCanPayloadIntoQueue(SERV_BC_CAN, bcPayLoad.payLoad, bcPayLoad.lengthOfPayload);

                xSemaphoreGive(xMutexIpcFifo);
            }
            /* Message has error */
            else
            {
                /* Handle error */
                BcCan_HandleError(bcError);
            }
        }
    }
}


/*******************************************************************************
 End of File
 */
