/*
 * TIM.cpp
 *
 *  Created on: 2 Jan 2021
 *      Author: krabb
 */

#include "General_Timer.h"

#define TIM_FIRST_IRQ 	TIM1_BRK_IRQn
#define TIM_LAST_IRQ 	TIM4_IRQn

#define CALLBACKS 0

General_Timer::onIRQCallback callbacks[(TIM_LAST_IRQ - TIM_FIRST_IRQ) + 1] { nullptr };

extern "C" {
void TIM1_BRK_IRQHandler() {
	timer2.handle_interrupt();
}
void TIM1_UP_IRQHandler() {
	timer2.handle_interrupt();
}
void TIM1_TRG_COM_IRQHandler() {
	timer2.handle_interrupt();
}
void TIM1_CC_IRQHandler() {
	timer2.handle_interrupt();
}
void TIM2_IRQHandler() {
	timer2.handle_interrupt();
	callbacks[TIM2_IRQn - TIM_FIRST_IRQ]();
}
void TIM3_IRQHandler() {
	timer2.handle_interrupt();
}
void TIM4_IRQHandler() {
	timer2.handle_interrupt();
}
}

General_Timer::General_Timer(uint32_t Period, uint32_t Prescaler, uint32_t ClockDivision, uint32_t CounterMode, uint32_t AutoReloadPreload) {
	/* Set the Time Base configuration */
	uint32_t tmpcr1 = CR1;

	/* Set TIM Time Base Unit parameters ---------------------------------------*/
	/* Select the Counter Mode */
	tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	tmpcr1 |= CounterMode;

	/* Set the clock division */
	tmpcr1 &= ~TIM_CR1_CKD;
	tmpcr1 |= ClockDivision;

	/* Set the auto-reload preload */
	MODIFY_REG(tmpcr1, TIM_CR1_ARPE, AutoReloadPreload);
	CR1 = tmpcr1;
	ARR = Period;
	PSC = Prescaler;

	/* Generate an update event to reload the Prescaler and the repetition counter (only for advanced timer) value immediately */
	EGR = TIM_EGR_UG;
}

void* General_Timer::operator new(size_t, General_Timer::TIMx number) {
	if (number == TIM2) {
		RCC1.APB1ENR |= RCC_APB1ENR_TIM2EN;
		(void)(RCC1.APB1ENR & RCC_APB1ENR_TIM2EN);
		NVIC_EnableIRQ(TIM2_IRQn);
	} else if (number == TIM3) {
		RCC1.APB1ENR |= RCC_APB1ENR_TIM3EN;
		(void)(RCC1.APB1ENR & RCC_APB1ENR_TIM3EN);
		NVIC_EnableIRQ(TIM3_IRQn);
	} else if (number == TIM4) {
		RCC1.APB1ENR |= RCC_APB1ENR_TIM4EN;
		(void)(RCC1.APB1ENR & RCC_APB1ENR_TIM4EN);
		NVIC_EnableIRQ(TIM4_IRQn);
	}

	return reinterpret_cast<void *>(number);
}

void General_Timer::enable_interrupt(onIRQCallback callback) {
	callbacks[TIM2_IRQn - TIM_FIRST_IRQ] = callback;

	/* Enable the TIM Update interrupt */
	DIER |= TIM_IT_UPDATE;

	/* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
	if ((SMCR & TIM_SMCR_SMS) != TIM_SLAVEMODE_TRIGGER)
		CR1 |= TIM_CR1_CEN;
}

void General_Timer::handle_interrupt() {
	/* Capture compare 1 event */
	if (SR & TIM_SR_CC1IF) {
		if (DIER & TIM_DIER_CC1IE) {
			SR = ~TIM_SR_CC1IF;
#if CALLBACKS
			if (tim.CCMR1 & TIM_CCMR1_CC1S)
				HAL_TIM_IC_CaptureCallback(htim);
			else {
				HAL_TIM_OC_DelayElapsedCallback(htim);
				HAL_TIM_PWM_PulseFinishedCallback(htim);
			}
#endif
		}
	}

	/* Capture compare 2 event */
	if (SR & TIM_SR_CC2IF) {
		if (DIER & TIM_DIER_CC2IE) {
			SR = ~TIM_SR_CC2IF;
#if CALLBACKS
			if (tim.CCMR1 & TIM_CCMR1_CC2S)
				HAL_TIM_IC_CaptureCallback(htim);
			else {
				HAL_TIM_OC_DelayElapsedCallback(htim);
				HAL_TIM_PWM_PulseFinishedCallback(htim);
			}
#endif
		}
	}

	/* Capture compare 3 event */
	if (SR & TIM_SR_CC3IF) {
		if (DIER & TIM_DIER_CC3IE) {
			SR = ~TIM_SR_CC3IF;
#if CALLBACKS
			if (tim.CCMR1 & TIM_CCMR1_CC3S)
				HAL_TIM_IC_CaptureCallback(htim);
			else {
				HAL_TIM_OC_DelayElapsedCallback(htim);
				HAL_TIM_PWM_PulseFinishedCallback(htim);
			}
#endif
		}
	}

	/* Capture compare 4 event */
	if (SR & TIM_SR_CC4IF) {
		if (DIER & TIM_DIER_CC4IE) {
			SR = ~TIM_SR_CC4IF;
#if CALLBACKS
			if (tim.CCMR1 & TIM_CCMR1_CC4S)
				HAL_TIM_IC_CaptureCallback(htim);
			else {
				HAL_TIM_OC_DelayElapsedCallback(htim);
				HAL_TIM_PWM_PulseFinishedCallback(htim);
			}
#endif
		}
	}

	/* TIM Update event */
	if (SR & TIM_SR_UIF) {
		if (DIER & TIM_DIER_UIE) {
			SR = ~TIM_SR_UIF;
#if CALLBACKS
			HAL_TIM_PeriodElapsedCallback(htim);
#endif
		}
	}
	/* TIM Break input event */
	if (SR & TIM_SR_BIF) {
		if (DIER & TIM_DIER_BIE) {
			SR = ~TIM_SR_BIF;
#if CALLBACKS
			HAL_TIMEx_BreakCallback(htim);
#endif
		}
	}

	/* TIM Trigger detection event */
	if (SR & TIM_SR_TIF) {
		if (DIER & TIM_DIER_TIE) {
			SR = ~TIM_SR_TIF;
#if CALLBACKS
			HAL_TIM_TriggerCallback(htim);
#endif
		}
	}

	/* TIM commutation event */
	if (SR & TIM_SR_COMIF) {
		if (DIER & TIM_DIER_COMIE) {
			SR = ~TIM_SR_COMIF;
#if CALLBACKS
			HAL_TIMEx_CommutCallback(htim);
#endif
		}
	}
}
