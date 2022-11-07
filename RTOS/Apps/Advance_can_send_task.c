/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    advance_can_send_task.c

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
#include "advance_can_send_task.h"
#include "timers.h"
#include "advance_can.h"
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
    This structure should be initialized by the advance_CAN_SEND_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

extern TaskHandle_t xadvance_CAN_SEND_TASK_Tasks;

/* Declare a variable to hold the transmit event group */
extern EventGroupHandle_t advanceCan_TransmitEventGroup;

xTimerHandle advance_timerHandle500ms;
xTimerHandle advance_timerHandle100ms;
xTimerHandle advance_timerHandle1000ms;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void advance_TimerCallback(xTimerHandle xTimer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xTimer == advance_timerHandle100ms)
    {
        /* Inform trigger event for 100ms occur */
        xEventGroupSetBitsFromISR(
                              advanceCan_TransmitEventGroup,   /* The event group being updated. */
                              advance_TRIGGER_EVENT_100MS_BIT, /* The bits being set. */
                              &xHigherPriorityTaskWoken );
    }
    
    if (xTimer == advance_timerHandle500ms)
    {
        /* Inform trigger event for 500ms occur */
        xEventGroupSetBitsFromISR(
                              advanceCan_TransmitEventGroup,   /* The event group being updated. */
                              advance_TRIGGER_EVENT_500MS_BIT, /* The bits being set. */
                              &xHigherPriorityTaskWoken );
    }
    
    if (xTimer == advance_timerHandle1000ms)
    {
        /* Inform trigger event for 1000ms occur */
        xEventGroupSetBitsFromISR(
                              advanceCan_TransmitEventGroup,   /* The event group being updated. */
                              advance_TRIGGER_EVENT_1000MS_BIT, /* The bits being set. */
                              &xHigherPriorityTaskWoken );
    }
}
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
    void advance_CAN_SEND_TASK_Initialize ( void )

  Remarks:
    See prototype in advance_can_send_task.h.
 */

void advance_CAN_SEND_TASK_Initialize ( void )
{

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    advance_timerHandle100ms = xTimerCreate("timer1",
                                    pdMS_TO_TICKS(100U),
                                    pdTRUE,
                                    (void*) 2,
                                    advance_TimerCallback);
                                    
    advance_timerHandle500ms = xTimerCreate("timer2",
                                    pdMS_TO_TICKS(500U),
                                    pdTRUE,
                                    (void*) 1,
                                    advance_TimerCallback);
    
    advance_timerHandle1000ms = xTimerCreate("timer3",
                                    pdMS_TO_TICKS(1000U),
                                    pdTRUE,
                                    (void*) 3,
                                    advance_TimerCallback);
                                    
    /* Create the event group inform a message will be transmiited. */
    advanceCan_TransmitEventGroup = xEventGroupCreate();
    
}


/******************************************************************************
  Function:
    void advance_CAN_SEND_TASK_Tasks ( void )

  Remarks:
    See prototype in advance_can_send_task.h.
 */

void advance_CAN_SEND_TASK_Tasks ( void )
{
    /*Event return value*/
    EventBits_t uxEvents;
    
    /*Index of message transmit*/
    uint8_t idxOfMsg;
    
    /*Counter for while loop*/
    uint8_t idxArray;
    
    xTimerStart(advance_timerHandle500ms, 0U);
    xTimerStart(advance_timerHandle100ms, 0U);
    xTimerStart(advance_timerHandle1000ms, 0U);
    
    while(1)
    {
        /* Wait timer trigger event */
        uxEvents = xEventGroupWaitBits(
            advanceCan_TransmitEventGroup,      /* The event group being tested. */
            advance_EVENT_GROUP, /* The bits within the event group to wait for. */
            pdTRUE,        /* should be cleared before returning. */
            pdFALSE,       /* Don't wait for both bits, either bit will do. */
            portMAX_DELAY );/* Wait a maximum time for either bit to be set. */
        
        if ((uxEvents & advance_TRIGGER_EVENT_100MS_BIT) != 0U)
        {   
            /* Transmit advance can message 100ms cycle */
            for(idxArray = 0; idxArray < NUM_OF_advance_TRANS_MSG_100ms; idxArray++)
            {
                /*index of advance can message to transmit*/
                idxOfMsg = advanceCan_ArrayIndexSend100ms[idxArray];
                    
                /*Send message to can bus*/
                 advanceCan_SendToCanBus(&advance_CAN_TransmitMessage[idxOfMsg]);
            }
        }
        
        if ((uxEvents & advance_TRIGGER_EVENT_500MS_BIT) != 0U)
        {
            /* Transmit advance can message 500ms cycle */
            for(idxArray= 0; idxArray < NUM_OF_advance_TRANS_MSG_500ms; idxArray++)
            {
                /* Index of advance can message to transmit */
                idxOfMsg = advanceCan_ArrayIndexSend500ms[idxArray];
                    
                /* Send message to can bus */
                 advanceCan_SendToCanBus(&advance_CAN_TransmitMessage[idxOfMsg]);
            }
        }
        
        if ((uxEvents & advance_TRIGGER_EVENT_1000MS_BIT) != 0U)
        {
            /* Transmit advance can message 1000ms cycle */
            for(idxArray = 0; idxArray < NUM_OF_advance_TRANS_MSG_1000ms; idxArray++)
            {
                /* Index of advance can message to transmit */
                idxOfMsg = advanceCan_ArrayIndexSend1000ms[idxArray];
                    
                /* Send message to can bus */
                 advanceCan_SendToCanBus(&advance_CAN_TransmitMessage[idxOfMsg]);
            }
        }
        
        if ((uxEvents & advance_SET_EVENT_SETTING_BIT) != 0U) 
        {           
            /* Send message to can bus */          
            CAN2_MessageTransmit(advance_CAN_TransmitEventSettingMsg->idMsg,
                                 advance_CAN_TransmitEventSettingMsg->lengthOfMsg ,
                                 advance_CAN_TransmitEventSettingMsg->pData,
                                 advance_SEND_FIFO, CAN_MSG_TX_DATA_FRAME);
        
            xEventGroupClearBits(advanceCan_TransmitEventGroup, advance_SET_EVENT_SETTING_BIT); 
        }
        if ((uxEvents & advance_SET_REQ_PARAM_BIT) != 0U)
        {
            /* Send message to can bus */
            CAN2_MessageTransmit(advance_CAN_TransmitReqParameterMsg->idMsg,
                                 advance_CAN_TransmitReqParameterMsg->lengthOfMsg ,
                                 advance_CAN_TransmitReqParameterMsg->pData,
                                 advance_SEND_FIFO, CAN_MSG_TX_DATA_FRAME);
        
            xEventGroupClearBits(advanceCan_TransmitEventGroup, advance_SET_REQ_PARAM_BIT); 
        }
        if ((uxEvents & advance_SET_SEND_PARAM_BIT) != 0U)
        {
            /* Send message to can bus */
            CAN2_MessageTransmit(advance_CAN_TransmitSendParameterMsg->idMsg,
                                 advance_CAN_TransmitSendParameterMsg->lengthOfMsg ,
                                 advance_CAN_TransmitSendParameterMsg->pData,
                                 advance_SEND_FIFO, CAN_MSG_TX_DATA_FRAME);

            xEventGroupClearBits(advanceCan_TransmitEventGroup, advance_SET_SEND_PARAM_BIT);
        }
        if ((uxEvents & advance_SET_CONFIG_PARAM_BIT) != 0U)
        {
            /* Send message to can bus */
            CAN2_MessageTransmit(advance_CAN_TransmitConfigParametersMsg->idMsg,
                                 advance_CAN_TransmitConfigParametersMsg->lengthOfMsg ,
                                 advance_CAN_TransmitConfigParametersMsg->pData,
                                 advance_SEND_FIFO, CAN_MSG_TX_DATA_FRAME);
                                 
            xEventGroupClearBits(advanceCan_TransmitEventGroup, advance_SET_CONFIG_PARAM_BIT);
        }
        if ((uxEvents & advance_SET_BCU2_INFOR_BIT) != 0U)
        {             
            /* Send message to can bus */
            CAN2_MessageTransmit(advance_CAN_TransmitBcu2InforMsg->idMsg,
                                 advance_CAN_TransmitBcu2InforMsg->lengthOfMsg ,
                                 advance_CAN_TransmitBcu2InforMsg->pData,
                                 advance_SEND_FIFO, CAN_MSG_TX_DATA_FRAME);     
            
            xEventGroupClearBits(advanceCan_TransmitEventGroup, advance_SET_BCU2_INFOR_BIT);
        }
        if ((uxEvents & advance_SET_BCU2_DIAG_BIT) != 0U)
        {             
            /* Send message to can bus */
            CAN2_MessageTransmit(advance_CAN_TransmitBcu2DiagData1Msg->idMsg,
                                 advance_CAN_TransmitBcu2DiagData1Msg->lengthOfMsg ,
                                 advance_CAN_TransmitBcu2DiagData1Msg->pData,
                                 advance_SEND_FIFO, CAN_MSG_TX_DATA_FRAME);
            
            xEventGroupClearBits(advanceCan_TransmitEventGroup, advance_SET_BCU2_DIAG_BIT);
        }
       
    }
}


/*******************************************************************************
 End of File
 */
