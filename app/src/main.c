/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nrf.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <shared_types.h>
#include <zephyr/drivers/uart.h>

K_SEM_DEFINE(sem_on_update, 0, 1);

static void update_timer_handler(struct k_timer *timer_id)
{
    //k_sem_give(&sem_on_update);
}

K_TIMER_DEFINE(update_timer, update_timer_handler, NULL);

#define MY_RTC NRF_RTC10

static void start_rtc(void)
{
	MY_RTC->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
	MY_RTC->INTENSET = RTC_INTENSET_COMPARE0_Msk;
	MY_RTC->CC[0] = 32768 / 1;
	MY_RTC->PRESCALER = 0;
	MY_RTC->TASKS_START = 1;
}

int *static_ram_pointer = (int*)0x2003C000;

static void send_to_vpr(void *data, uint32_t len)
{
	NRF_P1->OUTSET = BIT(14);
	static_ram_pointer[0] = len;
	memcpy(&static_ram_pointer[1], data, len);
	NRF_VPR00->TASKS_TRIGGER[16] = 1;
}

int main(void)
{
	printf("App core start\n");

	start_rtc();

	uint32_t dt_values[] = {0, 150, 250, 400, 550};
	uint32_t dt_values_count = sizeof(dt_values) / sizeof(dt_values[0]);
	uint32_t time_values[] = {500, 1000, 2000};
	int index = 0;
	pwm_3p_config_t pwm_config;

	while (1) {
		MY_RTC->EVENTS_COMPARE[0] = 0;
		while(MY_RTC->EVENTS_COMPARE[0] == 0);
		MY_RTC->TASKS_CLEAR = 1;
		pwm_config.duty_cycle = dt_values[index % dt_values_count];
		pwm_config.steptime_us = time_values[index / dt_values_count];
		//printf("Changing PWM settings from appcore. Duty cycle %i, time %i\n", pwm_config.duty_cycle, pwm_config.steptime_us);
		//send_to_vpr((void*)&pwm_config, sizeof(pwm_config));
		index = (index + 1) % 9;
	}

	return 0;
}
