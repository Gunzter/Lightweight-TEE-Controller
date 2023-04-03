#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "tfm_api.h"
#include "tfm_secure_api.h"

#include "psa/service.h"
#include "tfm_sp_log.h"

#include "psa_manifest/pid.h"
#include "psa_manifest/tfm_secure_controller.h"

#include "secure_timer.h"
#include "tfm_memory_utils.h"

//For the UART
#include "serial2002.h"
#include "driver/Driver_USART.h"
#define LOG_UART Driver_USART1
extern ARM_DRIVER_USART LOG_UART;

/* Static function for controller file */
static float get_ref(void);
static int ref_ctr = 0;

static float convert_in(uint32_t);

static float calc_ctrl(float*, float);
static void update_ctrl(float*, float);
static float ctrl_signal = 0.0f;

static int ctr = 0;
static int running = 1;

/* Shared with serial2002.c */
uint32_t raw_meas[2];

static float meas[2];
static float z1 = 0.0f, z2 = 0.0f, z3 = 0.0f;
extern int32_t uart_status;
extern uint32_t received_bytes;

/* Shared variables and data can be here */
#define SAMPLE_TIME_MICRO       ( (uint32_t) 10000 )
#define SAMPLE_TIME_SECONDS     ( ( (float) SAMPLE_TIME_MICRO ) / 1000000.0f )
#define REFERENCE_PERIOD_ITR    ( (uint32_t) (20.0f / SAMPLE_TIME_SECONDS ) )
#define REFERENCE_AMPLITUDE     ( 6.0f )
#define DEAD_TIME_ITR           ( (uint32_t) (10.0f / SAMPLE_TIME_SECONDS ) )

//GPIO test
#define IRS_PIN             (28UL)
#define CTRL_PIN            (29UL)

psa_flih_result_t tfm_timer1_irq_flih(void)
{
    tfm_secure_controller_secure_timer_clear_intr();
    nrf_gpio_pin_set(CTRL_PIN);
    /* Get measurement values and perform proper conversion */
    get_ang();                              // Ang
    receive_value(); //blocking
    meas[0] = convert_in(raw_meas[0]);
    
    get_pos();                              // Pos
    receive_value(); //blocking 
    meas[1] = convert_in(raw_meas[1]);

    /* Fail-Safe */
    if (meas[0] > 6 || meas[0] < -6)
    {
        set_ctrl(0.0f);
        running = 0;
    } else if (running && ctr++ > DEAD_TIME_ITR)
    {
        /* Get reference signal */
    //    float ref = 0.0f;
        float ref = get_ref();

        /* Calculate next control signal */
        ctrl_signal = -1.0f * calc_ctrl(meas, ref);
    
        /* set actuation signals (first velocity and then acceleration) */
        set_ctrl(ctrl_signal);

        /* Update control state */
        update_ctrl(meas, ref);
    } else
    {
        set_ctrl(0.0f);
    }
    nrf_gpio_pin_clear(CTRL_PIN);
    return PSA_FLIH_NO_SIGNAL;
}

void psa_write_response(void *handle, uint8_t *response, uint32_t response_size)
{
    psa_write((psa_handle_t)handle, 0, response, response_size);
}

size_t format_log_msg(char* buffer) {
    char pos_buf[12]    = {0};
    char ang_buf[12]    = {0};
    char ctrl_buf[12]   = {0}; 
    gcvt(meas[0], 4, ang_buf);
    gcvt(meas[1], 4, pos_buf);
    gcvt(ctrl_signal, 4, ctrl_buf);

    sprintf(buffer, "ang: %-.9s pos: %-.9s u %-.9s", ang_buf, pos_buf, ctrl_buf);
    return strlen(buffer) + 1;

}

static psa_status_t tfm_sc_set_process(psa_msg_t *msg)
{
    //return message
    char response[80];
    size_t len = format_log_msg(response); 

    psa_write_response((void*)(msg->handle), (uint8_t*)response, len);

    return PSA_SUCCESS;
}


void tfm_secure_controller_entry(void)
{

    /* Enable GPIO pins to mesure execution time and jitter */
    nrf_gpio_cfg_output(IRS_PIN);
    nrf_gpio_cfg_output(CTRL_PIN);

    nrf_gpio_pin_clear(IRS_PIN);
    nrf_gpio_pin_clear(CTRL_PIN);

    /* Starting Timer1 and enabling interrupts */ 
    tfm_secure_controller_secure_timer_start(SAMPLE_TIME_MICRO);
    psa_irq_enable(TFM_TIMER1_IRQ_SIGNAL);
    psa_signal_t signals = 0;

    while (1) {
        signals = psa_wait(PSA_WAIT_ANY, PSA_BLOCK);
        if ( (signals & TFM_TIMER1_IRQ_SIGNAL) == TFM_TIMER1_IRQ_SIGNAL) {
            psa_reset_signal(TFM_TIMER1_IRQ_SIGNAL);
        }

        if ( (signals & TFM_SC_SET_PROCESS_SIGNAL) == TFM_SC_SET_PROCESS_SIGNAL) {
            psa_msg_t msg;
            psa_get(signals, &msg);
            psa_status_t status = tfm_sc_set_process(&msg);
            psa_reply(msg.handle, status);
        }
    }   
}

/* Get position reference (triangle wave) */
static float get_ref(void)
{
    ref_ctr++;
    int ref_tmp = ref_ctr % REFERENCE_PERIOD_ITR;
    if ( ref_tmp < (uint32_t) (REFERENCE_PERIOD_ITR / 4) )
        return 4*REFERENCE_AMPLITUDE * ref_tmp / REFERENCE_PERIOD_ITR; 
    else if ( ref_tmp > (uint32_t) (3 * REFERENCE_PERIOD_ITR / 4) )
        return 4*REFERENCE_AMPLITUDE * ref_tmp / REFERENCE_PERIOD_ITR - 4*REFERENCE_AMPLITUDE; 
    else 
        return -4*REFERENCE_AMPLITUDE * ref_tmp / REFERENCE_PERIOD_ITR + 2*REFERENCE_AMPLITUDE; 
}

#define IN_CONVERTION_K   ( 0.00007629424f )
#define IN_CONVERTION_M   ( -10.0f )

/* Converts raw position p_raw to a position in meters */
static float convert_in(uint32_t pos_raw)
{
    return (float) ( IN_CONVERTION_K * pos_raw + IN_CONVERTION_M );
}

/*********************************************************** 
 * TODO: NOTE THAT WE ARE CURRENTLY IGNORING REFERENCE VALUE 
 ***********************************************************/

#include "lqg_variables.h"

/* Calculate control signal */
static float calc_ctrl(float *meas, float ref)
{
    return H_1 * z1 + H_2 * z2 + H_3 * z3 + K_1 * meas[0] + K_2 * (meas[1] - ref);
}

/* Update control state */
static void update_ctrl(float *meas, float ref)
{ 
    float z1new, z2new, z3new;
    z1new = F_11 * z1 + F_12 * z2 + F_13 * z3 + G_11 * meas[0] + G_12 * (meas[1] - ref);
    z2new = F_21 * z1 + F_22 * z2 + F_23 * z3 + G_21 * meas[0] + G_22 * (meas[1] - ref);
    z3new = F_31 * z1 + F_32 * z2 + F_33 * z3 + G_31 * meas[0] + G_32 * (meas[1] - ref);
    z1 = z1new;
    z2 = z2new;
    z3 = z3new;
}
