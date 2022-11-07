/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    bootmode_request.c

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
#include "bootloader.h"
#include "bootmode_request.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "definitions.h"
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
    This structure should be initialized by the BOOTMODE_REQUEST_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/
BOOTMODE_REQUEST_DATA bootmode_requestData;

extern EventGroupHandle_t Bld_eventGroup;
extern EventGroupHandle_t Bld_resetEventGroup;

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
    void BOOTMODE_REQUEST_Initialize ( void )

  Remarks:
    See prototype in bootmode_request.h.
 */

void BOOTMODE_REQUEST_Initialize ( void )
{

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void BOOTMODE_REQUEST_Tasks ( void )

  Remarks:
    See prototype in bootmode_request.h.
 */

void BOOTMODE_REQUEST_Tasks ( void )
{
    uint8_t respMes;
    uint8_t lengthOfRespMes;

    /* Event return timer value */
    EventBits_t eventBits;
    EventBits_t resetEventBits;
    while(1)
    {
        eventBits = xEventGroupWaitBits(
            Bld_eventGroup,      /* The event group being tested. */
            BLD_REQUEST_PROGRAMING_BIT| BLD_REQUEST_RESET_BIT, /* The bits within the event group to wait for. */
            pdTRUE,        /* should be cleared before returning. */
            pdFALSE,       /* Don't wait for both bits, either bit will do. */
            portMAX_DELAY );/* Wait a maximum time for either bit to be set. */
        
        /* Event is Request programming */
        if ((eventBits & BLD_REQUEST_PROGRAMING_BIT) != 0U)
        {

            Bld_setTriggerBootLoader();

            respMes = BLD_REQUEST_PROGRAMING;

            lengthOfRespMes = 1;
            
            Bld_senResponse(SERV_BOOTLOADER, &respMes, lengthOfRespMes);
        }

        /* Event is Request reset */
        if ((eventBits & BLD_REQUEST_RESET_BIT) != 0U)
        {
            respMes = BLD_REQUEST_RESET;
            
            lengthOfRespMes = 1;
            
            Bld_senResponse(SERV_BOOTLOADER, &respMes, lengthOfRespMes);
            
            resetEventBits = xEventGroupWaitBits(
                        Bld_resetEventGroup,      /* The event group being tested. */
                        BLD_RESET_READY_BIT, /* The bits within the event group to wait for. */
                        pdTRUE,        /* should be cleared before returning. */
                        pdFALSE,       /* Don't wait for both bits, either bit will do. */
                        portMAX_DELAY );/* Wait a maximum time for either bit to be set. */
             if ((resetEventBits & BLD_RESET_READY_BIT) != 0U)
             {
                    Bld_softWareReset();
             }
        }
              
    }
}


/*******************************************************************************
 End of File
 */
