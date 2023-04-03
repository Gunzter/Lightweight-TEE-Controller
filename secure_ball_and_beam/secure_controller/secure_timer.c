#include <stdint.h>

#include <hal/nrf_timer.h>
#include <nrf_board.h>




static void timer_init(NRF_TIMER_Type * TIMER, uint32_t ticks)
{
    nrf_timer_mode_set(TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_frequency_set(TIMER, NRF_TIMER_FREQ_1MHz);
    nrf_timer_cc_set(TIMER, NRF_TIMER_CC_CHANNEL0, ticks);
    /* Clear the timer once event is generated. */
    nrf_timer_shorts_enable(TIMER, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
}

static void timer_stop(NRF_TIMER_Type * TIMER)
{
    nrf_timer_task_trigger(TIMER, NRF_TIMER_TASK_STOP);
    nrf_timer_int_disable(TIMER, NRF_TIMER_INT_COMPARE0_MASK);
    nrf_timer_event_clear(TIMER, NRF_TIMER_EVENT_COMPARE0);
}

static void timer_start(NRF_TIMER_Type * TIMER)
{
    timer_stop(TIMER);

    nrf_timer_task_trigger(TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_int_enable(TIMER, NRF_TIMER_INT_COMPARE0_MASK);

    nrf_timer_task_trigger(TIMER, NRF_TIMER_TASK_START);
}

static void timer_event_clear(NRF_TIMER_Type *TIMER)
{
    nrf_timer_event_clear(TIMER, NRF_TIMER_EVENT_COMPARE0);
}

void tfm_secure_controller_secure_timer_start(uint32_t ticks)
{
    timer_init(NRF_TIMER1, ticks);
    timer_start(NRF_TIMER1);
}

void tfm_secure_controller_secure_timer_clear_intr(void)
{
    timer_event_clear(NRF_TIMER1);
}
