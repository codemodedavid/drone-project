/*
 * time_utils.c
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */

#include "time_utils.h"

void enable_cycle_counter(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace & debug
    DWT->CYCCNT = 0;                                 // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable CYCCNT
}

float get_dt_us(uint32_t *last_time) {
    uint32_t now = DWT->CYCCNT;
    uint32_t elapsed_cycles = now - *last_time;
    float dt_us = (float)elapsed_cycles / (SystemCoreClock); // convert cycles to microseconds
    *last_time = now;
    return dt_us;
}
