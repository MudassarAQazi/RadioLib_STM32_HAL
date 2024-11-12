
#ifndef Timer_milliseconds_H
#define Timer_milliseconds_H

#include "stm32l0xx_hal.h"

///*------------------------------------------ [Timer STARTs] --------------------------------------------*/
// Timer for running timed based tasks.
/**
 * @brief  Reports Either the Assigned timer is Completed or Not.
 * @param  Time: Pointer to the variable that stores Last Function Accessed Time.
 * @param  Duration: Pointer to the Variable that stores timer overflow duration.
 * @retval True or False.
 */
bool timerCompleted(uint32_t *LastTime, uint32_t *Duration)
{
	bool tick = 0;
	// Getting Time Stamp
	uint32_t TimeStamp_ms = HAL_GetTick();
	// Time Overflow protection
//	if(*lastTime >= TimeStamp_ms)  {
//		*lastTime = 0;
//	}

	*LastTime = (*LastTime >= TimeStamp_ms)? (uint32_t)NULL:*LastTime;
	// HearBeat
	if ((TimeStamp_ms - *LastTime) >= *Duration) {
		*LastTime = TimeStamp_ms;
		tick = 1;
	}
	return tick;
}
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [Timer ENDs] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

#endif
