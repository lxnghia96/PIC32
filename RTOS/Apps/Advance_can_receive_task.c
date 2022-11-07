/*******************************************************************************
    MPLAB Harmony Application Source File

    Company:
        Microchip Technology Inc.

    File Name:
        Advance_Can_receive_task.c

    Summary:
        This file contains the source code for the MPLAB Harmony application.

    Description:
        This file contains the source code for the MPLAB Harmony application.  It
        implements the logic of the application's state machine and it may call
        API routines of other MPLAB Harmony modules in the system, such as drivers,
        system services, and middleware.  However, it does not call any of the
        system interfaces (such as the "Initialize" and "Tasks" functions) of any of
        the modules in the system or make any assumptions about when those functions
        are called.  That is the responsibility of the configuration-specific system
        files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "FreeRTOS.h"
#include "event_groups.h"
#include "Advance_can_receive_task.h"
#include "Advance_can.h"
#include "ipc_interfaces.h"

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
        This structure should be initialized by the Advance_Can_RECEIVE_TASK_Initialize function.

        Application strings and buffers are be defined outside this structure.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Declare a variable to hold the created receive event group. */

extern TaskHandle_t xADVANCE_CAN_RECEIVE_TASK_Tasks;

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
        void ADVANCE_CAN_RECEIVE_TASK_Initialize ( void )
        
    Remarks:
        See prototype in ADVANCE_CAN_receive_task.h.
 */

void ADVANCE_CAN_RECEIVE_TASK_Initialize ( void )
{
}


/******************************************************************************
    Function:
        void ADVANCE_CAN_RECEIVE_TASK_Tasks ( void )
        
    Remarks:
        See prototype in ADVANCE_CAN_receive_task.h.
 */

void ADVANCE_CAN_RECEIVE_TASK_Tasks ( void )
{
    /* Declare status variable */
    clp_status_t clpStatus;
    
    /* Message length */
    uint8_t msgLen;
        
    /* Message id */
    uint32_t rxMessageId;
    
    /* Buffer message */
    uint8_t rxmessage[CLP_8_BYTES_LENGTH];
 
    /* DID type in received message */
    uint8_t didType;
    
    /* Structure of clp payload */
    clp_payload_t clpPayload;
    
    /* Structure of clp multio-packet */
    clp_multi_packet_t clpMultiPacketInfor;
            
    while(1)
    {
        /* Ready to receive a new message */
        clpStatus =  ClpCan_ReceiveFromCanBus(&rxMessageId, &msgLen,rxmessage);
        
        if(CLP_OK == clpStatus)
        { 
            /* Handle the received message from Canbus */
            clpStatus = ClpCan_CanBusDataReceiveHandle(rxMessageId,rxmessage,msgLen,
                                                        &didType,&clpMultiPacketInfor);
            
            if(CLP_OK == clpStatus)
            {
                /* Pack message for multi packet */
                if((clpMultiPacketInfor.isReceivedMultiPacket == true)
                    && ( clpMultiPacketInfor.isInProcessing == false))
                {
                    /* Clp pack payload */
                    ClpCan_PackMsgToPayloadOfClp(clpMultiPacketInfor.pgnInCm,
                                                clpMultiPacketInfor.completeReceivedData,
                                                clpMultiPacketInfor.totalSizeReceived,
                                                &clpPayload,&didType);
                                
                    xSemaphoreTake(xMutexIpcFifo, portMAX_DELAY);

                    /* IPC push into queue */
                    Ipc_PackCanPayloadIntoQueue(SERV_ADVANCE_CAN, clpPayload.payload, clpPayload.length);
                    
                    xSemaphoreGive(xMutexIpcFifo);
                    
                }
                /* Pack message for single packet */
                else if((clpMultiPacketInfor.isReceivedMultiPacket == false) 
                            && (clpMultiPacketInfor.isInProcessing == false))
                { 
                    clpMultiPacketInfor.pgnInCm = ((rxMessageId & CLP_PGN_MASK)>> 8U);
                    
                    ClpCan_PackMsgToPayloadOfClp(clpMultiPacketInfor.pgnInCm,rxmessage,
                                                 msgLen,&clpPayload,&didType);
                                
                    xSemaphoreTake(xMutexIpcFifo, portMAX_DELAY);
                    
                    /* IPC push into queue */
                    Ipc_PackCanPayloadIntoQueue(SERV_ADVANCE_CAN, clpPayload.payload, clpPayload.length);
                    
                    xSemaphoreGive(xMutexIpcFifo);
                    
                }
                else
                {
                    clpMultiPacketInfor.isInProcessing = true;
                }
            }
            else
            {
                /*Do nothing*/
            }
                
        }        
         
    }
}


/*******************************************************************************
 End of File
 */
