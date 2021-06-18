#include "stm32f1xx_hal.h"
#include "RCC.h"
#include "EXTI.h"
#include "General_Timer.h"
#include "GPIO.h"
#include "AFIO.h"
#include "USART.h"
#include "I2C.h"

// Placeholder definitions until memory map class is made
AFIO& afio = *new AFIO;
RCC& RCC1 = *new RCC;
EXTI& exti = *new EXTI;
GPIO& GPIOA = *new (GPIO::A) GPIO;
GPIO& GPIOB = *new (GPIO::B) GPIO;
GPIO& GPIOC = *new (GPIO::C) GPIO;
General_Timer& timer2 = *new (General_Timer::TIM2) General_Timer {
	SystemCoreClock / 1000 - 1,
	SystemCoreClock / 1000 - 1,
	0,
	TIM_COUNTERMODE_UP,
	TIM_AUTORELOAD_PRELOAD_DISABLE
};
I2C& i2c = *new (I2C::NUMBER::I2C_1) I2C{ 400000, I2C_DUTYCYCLE_2, 0x30F, I2C::AddressMode::SevenBit  };

// Find somewhere nice to hide the UART pins later
Output_Pin TX{ GPIOA, 9, Output_Pin::Mode::AF_PP, Output_Pin::Speed::FREQ_HIGH };
Output_Pin RX{ GPIOA, 10, Output_Pin::Mode::AF_PP, Output_Pin::Speed::FREQ_HIGH };

Output_Pin SCL{ GPIOB, 6, Output_Pin::Mode::AF_OD, Output_Pin::Speed::FREQ_HIGH };
Output_Pin SDA{ GPIOB, 7, Output_Pin::Mode::AF_OD, Output_Pin::Speed::FREQ_HIGH };

Output_Pin LED{ GPIOC, 13, Output_Pin::Mode::PP, Output_Pin::Speed::FREQ_LOW };
Input_Pin button{ GPIOC, 15, Input_Pin::Pull::Up };
UART& uart = *new (USART::USART_1) UART{
	9600,
	USART_WORDLENGTH_8B,
	USART_STOPBITS_1,
	USART_PARITY_NONE,
	USART_MODE_TX_RX,
	HW_CONTROL::NONE,
	UART_OVERSAMPLING_16
};

int main(void) {
	button.enable_interrupt(EXTI::TRIGGER::Falling, [](){ LED.toggle(); });
	//timer2.enable_interrupt([]() { LED.toggle(); });
	char test_buffer[10]{ 0 };
	uart.transmit("Mymyteststrwwer");
	uart.receive_it(test_buffer, [](auto sv) {
		LED.toggle();
	});

	while (true) {
		HAL_Delay(1000);
	}
}
