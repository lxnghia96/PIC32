// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "FreeRTOS.h"
#include "timers.h"
#include "event_groups.h"
#include "bc_can_send_task.h"
#include "bc_can.h"

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
    This structure should be initialized by the BC_CAN_SEND_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

#define TRIGGER_EVENT_50MS_BIT                            (1U << 0U)
#define TRIGGER_EVENT_100MS_BIT                           (1U << 1U)

EventGroupHandle_t BcCan_TimerEventGroup;

xTimerHandle bc_timerHandle100Ms;

xTimerHandle bc_timerHandle50Ms;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
void bc_TimerCallback(xTimerHandle xTimer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xTimer == bc_timerHandle100Ms)
    {   
        /* Inform trigger event for 100ms occur */
        xEventGroupSetBitsFromISR(
                              BcCan_TimerEventGroup,   /* The event group being updated. */
                              TRIGGER_EVENT_100MS_BIT, /* The bits being set. */
                              &xHigherPriorityTaskWoken );
    }
    
    if (xTimer == bc_timerHandle50Ms)
    {
        /* Inform trigger event for 50ms occur */
        xEventGroupSetBitsFromISR(
                              BcCan_TimerEventGroup,   /* The event group being updated. */
                              TRIGGER_EVENT_50MS_BIT, /* The bits being set. */
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
    void BC_CAN_SEND_TASK_Initialize ( void )

  Remarks:
    See prototype in bc_can_send_task.h.
 */

void BC_CAN_SEND_TASK_Initialize ( void )
{
    bc_timerHandle100Ms = xTimerCreate("timer100Ms",
                                    pdMS_TO_TICKS(100U),
                                    pdTRUE,
                                    (void*)0,
                                    bc_TimerCallback);

    bc_timerHandle50Ms = xTimerCreate("timer50Ms",
                                    pdMS_TO_TICKS(50U),
                                    pdTRUE,
                                    (void*)1,
                                    bc_TimerCallback);  
    
    /*Create the event group inform timer trigger occur.*/
    BcCan_TimerEventGroup = xEventGroupCreate();
    
}


/******************************************************************************
  Function:
    void BC_CAN_SEND_TASK_Tasks ( void )

  Remarks:
    See prototype in bc_can_send_task.h.
 */

void BC_CAN_SEND_TASK_Tasks ( void )
{

    /* Event return timer value */
    EventBits_t timerBits;

    /* Index of message transmit */
    uint8_t idxOfMsg;

    /* Counter for while loop */
    uint8_t i;

    xTimerStart(bc_timerHandle100Ms, 0U); 

    xTimerStart(bc_timerHandle50Ms, 0U);
    
    while (1)
    {
        /* Wait timer trigger event */
        timerBits = xEventGroupWaitBits(
            BcCan_TimerEventGroup,      /* The event group being tested. */
            TRIGGER_EVENT_100MS_BIT|TRIGGER_EVENT_50MS_BIT, /* The bits within the event group to wait for. */
            pdTRUE,        /* should be cleared before returning. */
            pdFALSE,       /* Don't wait for both bits, either bit will do. */
            portMAX_DELAY );/* Wait a maximum time for either bit to be set. */

        /* Event is Timer 100MS */
        if ((timerBits & TRIGGER_EVENT_100MS_BIT) != 0U)
        {
            /* Transmit BC can message */
            for(i = 0; i < NUM_OF_BC_TRANS_MSG_100MS; i++)
            {
                /* index of BC can message to transmit */
                idxOfMsg = BcCan_ArrayIndexSend100Ms[i];
                
                /* Send message to can bus */
                BcCan_SendToCanBus(&BcCan_TransmitMessage[idxOfMsg]);
            }
        }
        /* Event is Timer 50MS */
        if ((timerBits & TRIGGER_EVENT_50MS_BIT) != 0U)
        {
            /* In AD mode */
            if (AD_MODE_ENABLE == BcCan_GetAutoDockModeStatus())
            {   
                /* Transmit BC can message */
                for(i = 0; i < NUM_OF_BC_TRANS_MSG_50MS; i++)
                {
                    /* Index of BC can message to transmit */
                    idxOfMsg = BcCan_ArrayIndexSend50Ms[i];

                    /* Send message to can bus */
                    BcCan_SendToCanBus(&BcCan_TransmitMessage[idxOfMsg]);
                }
            }
        }
    }
}



/*******************************************************************************
 End of File
 */
