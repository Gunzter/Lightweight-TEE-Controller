#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <string.h>
#include <stdio.h>

#include "serial2002.h"

/* uart1 is defined in an .overlay, so DT_CHOSEN() does not work for some
 * reason. 
 */
#define UART1_DEVICE_NODE DT_NODELABEL(uart1)
const struct device *serial2002_uart_dev = DEVICE_DT_GET(UART1_DEVICE_NODE);

#define GPIO0_DEVICE_NODE DT_NODELABEL(gpio0)
const struct device *gpio_dev = DEVICE_DT_GET(GPIO0_DEVICE_NODE);
#define MALICIOUS_PIN 27
#define MAL_WAIT_PIN  28
#define CTRL_PIN      29


/* Static function for controller file */
static float get_ref(void);

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
#define SAMPLE_TIME_MILLI       ( (uint32_t) 10 )
#define SAMPLE_TIME_MICRO       ( (uint32_t) 10000 )
#define SAMPLE_TIME_SECONDS     ( ( (float) SAMPLE_TIME_MICRO ) / 1000000.0f )
#define REFERENCE_PERIOD_MILLIS ( (int64_t) 20000 )
#define REFERENCE_AMPLITUDE     ( 6.0f )
#define DEADTIME_MILLIS         ( (int64_t) 10000 )

/* Defines for Zephyr OS threads */
#define STACKSIZE 1024
#define PRIORITY    1
#define MALICIOUS_PRIORITY    -5

static uint64_t log_ctr = 0;
void print_log(){
  if(log_ctr++ % 10 == 0){
    printf("ang: %f pos: %f u %f\n", meas[0], meas[1], ctrl_signal);
  }
}


/* This define activate the resource starvation attack. */
#define ATTACK 1

#define TAU_M 1.5f
#define MAL_DEADTIME_MILLIS 3*DEADTIME_MILLIS
uint32_t mal_ctr = 0;

struct k_thread malicious_task;
static K_KERNEL_STACK_DEFINE(malicious_thread_stack, STACKSIZE);
void malicious_wait_thread(void){
      k_busy_wait(TAU_M * SAMPLE_TIME_MICRO);
      return;
}


void control_loop(void)
{
  /* Get current time in ms */
  int64_t t_per = k_uptime_get(), dur;

  while(true) {
    gpio_pin_set(gpio_dev, CTRL_PIN, true);
    
    /* Get measurement values and perform proper conversion */
    get_ang();                              // Ang
    receive_value(); //blocking
    meas[0] = convert_in(raw_meas[0]);
    get_pos();                              // Pos
    receive_value(); //blocking 
    meas[1] = convert_in(raw_meas[1]);

    /* Malicious wait here */
    if( ATTACK && (k_uptime_get() > MAL_DEADTIME_MILLIS) ){
      /*Pin high when attack is on, led on on GPIO 0*/
      gpio_pin_set(gpio_dev, MALICIOUS_PIN, false);

      gpio_pin_set(gpio_dev, MAL_WAIT_PIN, true);
      k_thread_create(&malicious_task, malicious_thread_stack, STACKSIZE, (k_thread_entry_t)malicious_wait_thread, NULL, NULL, NULL, MALICIOUS_PRIORITY, 0, K_NO_WAIT);
      gpio_pin_set(gpio_dev, MAL_WAIT_PIN, false);
    }
    
    /* Fail-Safe */
    if (meas[0] > 6 || meas[0] < -6)
    {
      set_ctrl(0.0f);
      running = 0;
    } else if ( running && (k_uptime_get() > DEADTIME_MILLIS) )
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

    /* Print loggin values every 50th iteration */
    //print_log();
    
    /* Get duration to sleep, in ms. */
    t_per += SAMPLE_TIME_MILLI;
    dur = t_per - k_uptime_get();

    gpio_pin_set(gpio_dev, CTRL_PIN, false);
    /* Wait */
    if (dur > 0) {
      k_msleep(dur);
    }
  }
}

K_THREAD_DEFINE(control_thread_id, STACKSIZE, control_loop, NULL, NULL, NULL, PRIORITY, 0, 0);

void main(void)
{

  if (!device_is_ready(serial2002_uart_dev)) {
    printk("Secondary UART device not found!");
    return;
  }

  /* configure interrupt and callback to receive data */
  uart_irq_rx_enable(serial2002_uart_dev);

  /* Configure GPIO pins to output */
  int ret;
  ret = gpio_pin_configure(gpio_dev, MALICIOUS_PIN, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return;
  }
  /* turn of led */
  gpio_pin_set(gpio_dev, MALICIOUS_PIN, true);
  ret = gpio_pin_configure(gpio_dev, MAL_WAIT_PIN, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return;
  }
  ret = gpio_pin_configure(gpio_dev, CTRL_PIN, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return;
  }


  printk("Starting Controller\r\n");

}

/* Get position reference (triangle wave) */
static float get_ref(void)
{
  int64_t t = k_uptime_get();
  int64_t ref_tmp = t % REFERENCE_PERIOD_MILLIS;
  if ( ref_tmp < (int64_t) (REFERENCE_PERIOD_MILLIS / 4) )
    return 4*REFERENCE_AMPLITUDE * ref_tmp / REFERENCE_PERIOD_MILLIS;
  else if ( ref_tmp > (int64_t) (3 * REFERENCE_PERIOD_MILLIS / 4) )
    return 4*REFERENCE_AMPLITUDE * ref_tmp / REFERENCE_PERIOD_MILLIS - 4*REFERENCE_AMPLITUDE;
  else
    return -4*REFERENCE_AMPLITUDE * ref_tmp / REFERENCE_PERIOD_MILLIS + 2*REFERENCE_AMPLITUDE;
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

