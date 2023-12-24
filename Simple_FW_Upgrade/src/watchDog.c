/*
 * watchDog.c
 *
 *  Created on: Dec 6, 2022
 *      Author: xbie
 */
#include "hal_data.h"
#include "watchdog.h"

static uint32_t _registeredTasks = 0;       // Bit map for registered tasks
static uint32_t _activeTasks = 0;           // Bit map for active tasks.

/**************************************************************************
Description: Check all the task bit maps to see if we can feed the watchdog
                to prevent reset.
Inputs:      None
Outputs:     None
Author :     xb, 12/6/2022
****************************************************************************/
static void _checkWatchdog(void)
{
    if((_activeTasks & _registeredTasks) == _registeredTasks)
    {   // All registered tasks are active! Because their active bit map matches registered bit map.
        _activeTasks = 0;       // Clear active bit map and let individual task to set active flag.

        // Feed the dog
        R_WDT_Refresh(&g_wdt0_ctrl);
    }
}

/**************************************************************************
Description: Register task to watchdog monitor
Inputs:      Task ID
Outputs:     None
Author :     xb, 12/6/2022
****************************************************************************/
void watchdogRegisterTask(uint32_t taskID)
{
    // Disable Interrupts?
    _registeredTasks |= (1 << taskID);
    // Enable Interrupts?
}

/**************************************************************************
Description: Called by monitored task to set active task bit flag.
Inputs:      Task ID
Outputs:     None
Author :     xb, 12/6/2022
****************************************************************************/
void watchdogTaskSetActive(uint32_t taskID)
{
    // Disable Interrupts?
    _activeTasks |= (1 << taskID);
    _checkWatchdog();
    // Enable Interrupts?
}

