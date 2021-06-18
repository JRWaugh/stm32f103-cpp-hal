/*
 * TIM.h
 *
 *  Created on: 2 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_GENERAL_TIMER_H_
#define SRC_GENERAL_TIMER_H_

#include "stm32f1xx_hal.h"
#include <new>
#include "RCC.h"

extern class General_Timer {
public:
	using onIRQCallback = void (*)();
	General_Timer(uint32_t Period, uint32_t Prescaler, uint32_t ClockDivision, uint32_t CounterMode, uint32_t AutoReloadPreload);
	enum TIMx { TIM2 = TIM2_BASE, TIM3 = TIM3_BASE, TIM4 = TIM4_BASE };
	void* operator new(size_t, General_Timer::TIMx number);
	void enable_interrupt(onIRQCallback callback);
	void handle_interrupt();

	__IO uint32_t CR1;             /*!< TIM control register 1,                      Address offset: 0x00 */
	__IO uint32_t CR2;             /*!< TIM control register 2,                      Address offset: 0x04 */
	__IO uint32_t SMCR;            /*!< TIM slave Mode Control register,             Address offset: 0x08 */
	__IO uint32_t DIER;            /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
	__IO uint32_t SR;              /*!< TIM status register,                         Address offset: 0x10 */
	__IO uint32_t EGR;             /*!< TIM event generation register,               Address offset: 0x14 */
	__IO uint32_t CCMR1;           /*!< TIM  capture/compare mode register 1,        Address offset: 0x18 */
	__IO uint32_t CCMR2;           /*!< TIM  capture/compare mode register 2,        Address offset: 0x1C */
	__IO uint32_t CCER;            /*!< TIM capture/compare enable register,         Address offset: 0x20 */
	__IO uint32_t CNT;             /*!< TIM counter register,                        Address offset: 0x24 */
	__IO uint32_t PSC;             /*!< TIM prescaler register,                      Address offset: 0x28 */
	__IO uint32_t ARR;             /*!< TIM auto-reload register,                    Address offset: 0x2C */
	__IO uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
	__IO uint32_t CCR1;            /*!< TIM capture/compare register 1,              Address offset: 0x34 */
	__IO uint32_t CCR2;            /*!< TIM capture/compare register 2,              Address offset: 0x38 */
	__IO uint32_t CCR3;            /*!< TIM capture/compare register 3,              Address offset: 0x3C */
	__IO uint32_t CCR4;            /*!< TIM capture/compare register 4,              Address offset: 0x40 */
	__IO uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
	__IO uint32_t DCR;             /*!< TIM DMA control register,                    Address offset: 0x48 */
	__IO uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
	__IO uint32_t OR;              /*!< TIM option register,                         Address offset: 0x50 */
}& timer2;

#endif /* SRC_GENERAL_TIMER_H_ */
