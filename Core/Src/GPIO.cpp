#include "GPIO.h"
#include "RCC.h"

void* GPIO::operator new(size_t, GPIO::Port port) {
	RCC1.APB2ENR |= 0x1UL << (2 + port);
	(void)(RCC1.APB2ENR & (0x1UL << (2 + port)));
	return reinterpret_cast<void *>(GPIOA_BASE + port * 0x400U);
}

Pin::Pin(GPIO& port, uint32_t const pin) : port{ port }, pin{ pin } {
	// Empty
}

Pin::~Pin() {
	__IO uint32_t& configregister = pin < 8 ? port.CRL : port.CRH;
	uint32_t const registeroffset = pin < 8 ? pin << 2 : (pin - 8) << 2;
	configregister = (configregister & ~((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registeroffset)) | (GPIO_CRL_CNF0_0 << registeroffset);
	/* ODR default value is 0 */
	port.ODR &= ~1UL << pin;
}

