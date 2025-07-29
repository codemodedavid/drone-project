/*
 * time_utils.h
 *
 *  Created on: Jul 28, 2025
 *      Author: davv
 */

#ifndef INC_TIME_UTILS_H_
#define INC_TIME_UTILS_H_

#include "stm32f4xx_hal.h"

void enable_cycle_counter(void);
float get_dt_us(uint32_t *last_time);

#endif /* INC_TIME_UTILS_H_ */
