/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <stdio.h>
#include <stdint.h>

#include "app.h"
#include "pwm_initialize.h"

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
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

//Proto:
void TestLed(void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
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
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{	
	#define MY_BUFFER_SIZE   5
    char          myBuffer[MY_BUFFER_SIZE] = { 11, 22, 33, 44, 55};
    unsigned int        numBytes;
    DRV_SPI_BUFFER_HANDLE bufHandle;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            DRV_SPI1_Initialize();
            OSCILLATOR_Initialize();
            PIN_MANAGER_Initialize();
            TMR_Initialize();
            PWM_Initialize();
            
            /* Update the state to transfer data */
            appData.state = APP_STATE_DATA_PUT;
            break;
        }

        /* TODO: implement your application state machine.*/
        case APP_STATE_DATA_PUT:
        {
            bufHandle = DRV_SPI1_BufferAddWriteRead (NULL, myBuffer,
                                                 5, NULL, NULL );
            /* Update the state to status check */
            break;
        }
        
        case APP_STATE_READ_SENSOR:
        {
			ReadGyro();
            break;
        }
        
        case APP_STATE_LED_CROSS:
        {
            //ShowLEDCross();
            break;
        }
		
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void ReadGyro(void)
{
    uint16_t* readBuff = 0b0000000000000000;
    DRV_SPI1_Initialize();
    DRV_SPI1_BufferAddWriteRead(NULL, readBuff, 0);
    
    printf("%c\r\n", readBuff);
}
/**
 * Sets the register for LEDs and sensors
 */
void InitRegister(void)
{
    
    // set LED cross TRIS register
    // horizontal LED line
    TRISAbits.TRISA7=0;
    TRISEbits.TRISE0=0;
    TRISEbits.TRISE1=0;
    TRISGbits.TRISG14=0;
    TRISGbits.TRISG12=0;
    TRISGbits.TRISG13=0;
    TRISDbits.TRISD4=0;       // cross center
    TRISEbits.TRISE2=0;
    TRISEbits.TRISE3=0;
    TRISEbits.TRISE4=0;
    TRISGbits.TRISG15=0;
    TRISAbits.TRISA5=0;
    TRISEbits.TRISE6=0;
    
    // vertical LED line
    TRISBbits.TRISB4=0;
    TRISAbits.TRISA9=0;
    TRISBbits.TRISB8=0;
	TRISAbits.TRISA0=0;       // cross center
	TRISFbits.TRISF12=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=0;
    
    // TODO: set TRIS register for sensor
}

void ShowLEDCross(void) {
    LATEbits.LATE0=1;
    LATAbits.LATA7=1;
    LATEbits.LATE1=1;
    LATGbits.LATG12=1;
    LATGbits.LATG13=1;
    LATGbits.LATG14=1;
    LATDbits.LATD4=1;
    LATEbits.LATE2=1;
    LATEbits.LATE3=1;
    LATEbits.LATE4=1;
    LATGbits.LATG15=1;
    LATAbits.LATA5=1;
    LATEbits.LATE6=1;
    
    LATBbits.LATB4=1;
    LATAbits.LATA9=1;
    LATBbits.LATB8=1;
    LATFbits.LATF12=1;
    LATBbits.LATB12=1;
    LATBbits.LATB13=1;
    LATAbits.LATA0=1;
}


void TestLed(void)
{
    /*
     * Thus each port is associated with three registers TRIS, PORT and LAT. 
     * As before TRIS register determines the direction of each digital IO pin, 
     * ie Input or Output. PORT register should be used to read data from Input 
     * pins, ie it reads Actual Voltage Levels of the pins as in 16F. LAT 
     * register is used to write data to Output pins. 
     */
    
    static int tick=0;
	static int l=0;
    
    // horizontal LED line
    TRISAbits.TRISA7=0;
    TRISEbits.TRISE0=0;
    TRISEbits.TRISE1=0;
    TRISGbits.TRISG14=0;
    TRISGbits.TRISG12=0;
    TRISGbits.TRISG13=0;
    TRISDbits.TRISD4=0;       // cross center
    TRISEbits.TRISE2=0;
    TRISEbits.TRISE3=0;
    TRISEbits.TRISE4=0;
    TRISGbits.TRISG15=0;
    TRISAbits.TRISA5=0;
    TRISEbits.TRISE6=0;
    
    // vertical LED line
    TRISBbits.TRISB4=0;
    TRISAbits.TRISA9=0;
    TRISBbits.TRISB8=0;
	TRISAbits.TRISA0=0;       // cross center
	TRISFbits.TRISF12=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=0;
    
    tick++;
	if(tick == 1000000 && l == 0)
	{
		tick=0;
		l=1;
        LATEbits.LATE0=1;
        LATAbits.LATA7=1;
        LATEbits.LATE1=1;
        LATGbits.LATG12=1;
        LATGbits.LATG13=1;
        LATGbits.LATG14=1;
        LATDbits.LATD4=1;
        LATEbits.LATE2=1;
        LATEbits.LATE3=1;
        LATEbits.LATE4=1;
        LATGbits.LATG15=1;
        LATAbits.LATA5=1;
        LATEbits.LATE6=1;
        
        LATBbits.LATB4=0;
        LATAbits.LATA9=0;
        LATBbits.LATB8=0;
        LATFbits.LATF12=0;
        LATBbits.LATB12=0;
        LATBbits.LATB13=0;
        LATAbits.LATA0=0;
    }
    
    if(tick >= 1000000 && l == 1)
	{
		tick=0;
		l=0;
        LATEbits.LATE0=0;
        LATAbits.LATA7=0;
        LATEbits.LATE1=0;
        LATGbits.LATG12=0;
        LATGbits.LATG13=0;
        LATGbits.LATG14=0;
        LATDbits.LATD4=0;
        LATEbits.LATE2=0;
        LATEbits.LATE3=0;
        LATEbits.LATE4=0;
        LATGbits.LATG15=0;
        LATAbits.LATA5=0;
        LATEbits.LATE6=0;
        
        LATBbits.LATB4=1;
        LATAbits.LATA9=1;
        LATBbits.LATB8=1;
        LATFbits.LATF12=1;
        LATBbits.LATB12=1;
        LATBbits.LATB13=1;
        LATAbits.LATA0=1;
    }
}