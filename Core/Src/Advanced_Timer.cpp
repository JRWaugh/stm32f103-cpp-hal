/*
 * Advanced_Timer.cpp
 *
 *  Created on: 3 Jan 2021
 *      Author: krabb
 */

#include "Advanced_Timer.h"

Advanced_Timer::Advanced_Timer(uint32_t Period, uint32_t Prescaler, uint32_t ClockDivision, uint32_t CounterMode, uint32_t AutoReloadPreload, uint32_t RepetitionCounter) :
General_Timer(Period, Prescaler, ClockDivision, CounterMode, AutoReloadPreload) {
	RCR = RepetitionCounter;
}

void* Advanced_Timer::operator new(size_t, Advanced_Timer::TIMx number) {
	if (number == TIM1) {
		RCC1.APB2ENR |= RCC_APB2ENR_TIM1EN;
		(void)(RCC1.APB2ENR & RCC_APB2ENR_TIM1EN);
		NVIC_EnableIRQ(TIM1_CC_IRQn);
	}
	return reinterpret_cast<void *>(number);
}
