/*******************************************************************************
  OC Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_oc_static.h

  Summary:
    OC driver interface declarations for the static single instance driver.

  Description:
    The OC device driver provides a simple interface to manage the OC
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the OC driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_OC_STATIC_H
#define _DRV_OC_STATIC_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 0 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC0_Initialize(void);

void DRV_OC0_Enable(void);

void DRV_OC0_Disable(void);

void DRV_OC0_Start(void);

void DRV_OC0_Stop(void);

bool DRV_OC0_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 1 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC1_Initialize(void);

void DRV_OC1_Enable(void);

void DRV_OC1_Disable(void);

void DRV_OC1_Start(void);

void DRV_OC1_Stop(void);

bool DRV_OC1_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 2 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC2_Initialize(void);

void DRV_OC2_Enable(void);

void DRV_OC2_Disable(void);

void DRV_OC2_Start(void);

void DRV_OC2_Stop(void);

bool DRV_OC2_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 3 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC3_Initialize(void);

void DRV_OC3_Enable(void);

void DRV_OC3_Disable(void);

void DRV_OC3_Start(void);

void DRV_OC3_Stop(void);

bool DRV_OC3_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 4 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC4_Initialize(void);

void DRV_OC4_Enable(void);

void DRV_OC4_Disable(void);

void DRV_OC4_Start(void);

void DRV_OC4_Stop(void);

bool DRV_OC4_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 5 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC5_Initialize(void);

void DRV_OC5_Enable(void);

void DRV_OC5_Disable(void);

void DRV_OC5_Start(void);

void DRV_OC5_Stop(void);

bool DRV_OC5_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 6 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC6_Initialize(void);

void DRV_OC6_Enable(void);

void DRV_OC6_Disable(void);

void DRV_OC6_Start(void);

void DRV_OC6_Stop(void);

bool DRV_OC6_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 7 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC7_Initialize(void);

void DRV_OC7_Enable(void);

void DRV_OC7_Disable(void);

void DRV_OC7_Start(void);

void DRV_OC7_Stop(void);

bool DRV_OC7_FaultHasOccurred(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 8 for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_OC8_Initialize(void);

void DRV_OC8_Enable(void);

void DRV_OC8_Disable(void);

void DRV_OC8_Start(void);

void DRV_OC8_Stop(void);

bool DRV_OC8_FaultHasOccurred(void);
#endif // #ifndef _DRV_OC_STATIC_H

/*******************************************************************************
 End of File
*/
