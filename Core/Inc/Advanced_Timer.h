/*
 * Advanced_Timer.h
 *
 *  Created on: 3 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_ADVANCED_TIMER_H_
#define SRC_ADVANCED_TIMER_H_

#include "General_Timer.h"
#include <new>

class Advanced_Timer : public General_Timer {
public:
	Advanced_Timer(uint32_t Period, uint32_t Prescaler, uint32_t ClockDivision, uint32_t CounterMode, uint32_t AutoReloadPreload, uint32_t RepetitionCounter);
	enum TIMx { TIM1 = TIM1_BASE };
	void* operator new(size_t, Advanced_Timer::TIMx number = TIM1);
};


#endif /* SRC_ADVANCED_TIMER_H_ */
