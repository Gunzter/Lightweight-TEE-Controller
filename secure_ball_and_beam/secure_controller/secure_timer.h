#include <stdint.h>

#include <hal/nrf_timer.h>
#include <nrf_board.h>

/* Start timer, ticks is in MHz. 1000*1000 ticks = 1s */
void tfm_secure_controller_secure_timer_start(uint32_t ticks);

void tfm_secure_controller_secure_timer_clear_intr(void);
