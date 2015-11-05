#include "mbed-drivers/InterruptIn.h"

#if DEVICE_INTERRUPTIN

extern "C" int gpio_check_handler(uint32_t id, gpio_irq_event event);

namespace mbed {

class StInterruptIn : public InterruptIn {
public:
    bool gpio_check_handler(gpio_irq_event event) {
	switch(event) {
	case IRQ_RISE: return (bool)_rise;
	case IRQ_FALL: return (bool)_fall;
	default: return false;
	}
    }
};

} // namespace mbed

int gpio_check_handler(uint32_t id, gpio_irq_event event) {
    mbed::StInterruptIn *handler = (mbed::StInterruptIn*)id;
    return (int)handler->gpio_check_handler(event);
}

#endif // DEVICE_INTERRUPTIN
