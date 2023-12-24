/*
 * watchDog.h
 *
 *  Created on: Dec 6, 2022
 *      Author: xbie
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

typedef enum
{
    TASK_ID_APPLICATION = 0,
    TASK_ID_EVENT,
    TASK_ID_PROTOCOL,
    TASK_ID_MAX
}ETaskID_t;

extern void watchdogRegisterTask(uint32_t taskID);
extern void watchdogTaskSetActive(uint32_t taskID);

#endif /* WATCHDOG_H_ */
