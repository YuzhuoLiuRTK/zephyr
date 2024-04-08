/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <trace.h>
#include <cmsis_core.h>
#include <rtl_pinmux.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>

struct k_timer my_timer;
void my_expiry_function(struct k_timer *timer_id)
{
	DBG_DIRECT("every 3s, in the timer cb");
}

int main(void)
{
    k_timer_init(&my_timer, my_expiry_function, NULL);
	k_timer_start(&my_timer, K_SECONDS(3), K_SECONDS(3));
	while(1) {
		k_msleep(2000); 
		//k_yield();
		//DBG_DIRECT("every 3s, wake up in main thread");
	}

	return 0;
}