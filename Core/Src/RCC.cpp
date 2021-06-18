#include "RCC.h"
#include <new>

void* RCC::operator new(size_t) {
	return reinterpret_cast<void *>(RCC_BASE);
}

