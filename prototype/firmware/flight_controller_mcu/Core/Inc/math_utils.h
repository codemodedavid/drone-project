/*
 * math_utils.h
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_

#include "stm32f4xx_hal.h"

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#endif /* INC_MATH_UTILS_H_ */
