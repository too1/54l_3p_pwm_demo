/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <nrf.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <shared_types.h>

#define MY_TIMER NRF_TIMER20
#define OUT_PIN 8
#define OUT_PORT 1
#define PWM_TOP_VALUE 500
#define DEAD_TIME_US 2
#define PPI_CH_A 0
#define PPI_CH_B 1
#define PPI_CH_EN 0x80000000

//static uint32_t pwm_phase_duration_table_us[6] = {100, 1000, 100, 1000, 100, 1000};

static void init_gpio(void)
{
	NRF_P1->DIRSET = 0xFFFFFFFF;
	NRF_P1->OUTSET = 0xFFFFFFFF;
}

#if 0
static void init_gpiote(void)
{
	NRF_GPIOTE20->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
							GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
							OUT_PIN << GPIOTE_CONFIG_PSEL_Pos |
							OUT_PORT << GPIOTE_CONFIG_PORT_Pos;
}
#endif

struct pwm_conf_t {
	NRF_PWM_Type *pwm;
	uint16_t *buf;
	uint16_t *buf2;
	uint32_t bufsize;
	uint32_t gpio[3];
};

#define MULT 2
static uint16_t seq0_ram[12*MULT+4] = {0};
static uint16_t seq0_ram_b[12*MULT+4] = {0};
static uint16_t seq1_ram[12*MULT+4] = {0};
static uint16_t seq1_ram_b[12*MULT+4] = {0};

static void set_sequence_values(uint32_t ph_dt)
{
	uint16_t dt_offset_factor_half = ((16 * DEAD_TIME_US) * 2) / 2;
	uint16_t lower = (ph_dt < dt_offset_factor_half) ? 0 : (ph_dt - dt_offset_factor_half);
	uint16_t higher = (ph_dt > (PWM_TOP_VALUE - dt_offset_factor_half)) ? PWM_TOP_VALUE : (ph_dt + dt_offset_factor_half);
	//printk("SetSeqVal %i\n", ph_dt);
	//memset(seq0_ram, 0, sizeof(seq0_ram));
	//memset(seq0_ram_b, 0, sizeof(seq0_ram_b));
	//memset(seq1_ram, 0, sizeof(seq1_ram));
	//memset(seq1_ram_b, 0, sizeof(seq1_ram_b));
	for(int iter = 0; iter < (12*MULT); iter += 12) {
		seq0_ram[0+iter] = seq1_ram[0+iter] = 0x8000 | lower;
		seq0_ram[4+iter] = seq1_ram[4+iter] = 0x8000 | lower;
		seq0_ram[8+iter] = seq1_ram[8+iter] = PWM_TOP_VALUE;
		seq0_ram[1+iter] = seq1_ram[1+iter] = PWM_TOP_VALUE;
		seq0_ram[5+iter] = seq1_ram[5+iter] = 0x8000 | lower;
		seq0_ram[9+iter] = seq1_ram[9+iter] = 0x8000 | lower;
		seq0_ram[2+iter] = seq1_ram[2+iter] = 0x8000 | lower;
		seq0_ram[6+iter] = seq1_ram[6+iter] = PWM_TOP_VALUE;
		seq0_ram[10+iter] = seq1_ram[10+iter] = 0x8000 | lower;
		seq0_ram_b[0+iter] = seq1_ram_b[0+iter] = higher;
		seq0_ram_b[4+iter] = seq1_ram_b[4+iter] = higher;
		seq0_ram_b[8+iter] = seq1_ram_b[8+iter] = PWM_TOP_VALUE;
		seq0_ram_b[1+iter] = seq1_ram_b[1+iter] = PWM_TOP_VALUE;
		seq0_ram_b[5+iter] = seq1_ram_b[5+iter] = higher;
		seq0_ram_b[9+iter] = seq1_ram_b[9+iter] = higher;
		seq0_ram_b[2+iter] = seq1_ram_b[2+iter] = higher;
		seq0_ram_b[6+iter] = seq1_ram_b[6+iter] = PWM_TOP_VALUE;
		seq0_ram_b[10+iter] = seq1_ram_b[10+iter] = higher;
	}
}

static void pwm_config(struct pwm_conf_t *conf)
{
	conf->pwm->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
	conf->pwm->MODE        = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos);
	conf->pwm->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
	conf->pwm->COUNTERTOP  = (PWM_TOP_VALUE << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
	

	conf->pwm->PSEL.OUT[0] = (conf->gpio[0] << PWM_PSEL_OUT_PIN_Pos) |
							 (1 << PWM_PSEL_OUT_PORT_Pos) |
                        	(PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	conf->pwm->PSEL.OUT[1] = (conf->gpio[1] << PWM_PSEL_OUT_PIN_Pos) |
							 (1 << PWM_PSEL_OUT_PORT_Pos) |
                        	(PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	conf->pwm->PSEL.OUT[2] = (conf->gpio[2] << PWM_PSEL_OUT_PIN_Pos) |
							 (1 << PWM_PSEL_OUT_PORT_Pos) |
                        	(PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

	// Enable the shortcut from LOOPSDONE event to SEQSTART1 task for infinite loop
	conf->pwm->SHORTS      = PWM_SHORTS_LOOPSDONE_DMA_SEQ0_START_Msk;
	
	// LOOP_CNT must be greater than 0 for the LOOPSDONE event to trigger and enable looping
	conf->pwm->LOOP        = (1 << PWM_LOOP_CNT_Pos);
	conf->pwm->DECODER     = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | 
						(PWM_DECODER_MODE_NextStep << PWM_DECODER_MODE_Pos);
	
	// To repeat a single sequence until stopped, it must be configured in SEQ[1]
	conf->pwm->DMA.SEQ[1].PTR  = ((uint32_t)(conf->buf) << PWM_DMA_SEQ_PTR_PTR_Pos);
	conf->pwm->DMA.SEQ[1].MAXCNT  = (conf->bufsize+8) << PWM_DMA_SEQ_MAXCNT_MAXCNT_Pos;
	conf->pwm->DMA.SEQ[0].PTR  = ((uint32_t)(conf->buf2) << PWM_DMA_SEQ_PTR_PTR_Pos);
	conf->pwm->DMA.SEQ[0].MAXCNT  = conf->bufsize << PWM_DMA_SEQ_MAXCNT_MAXCNT_Pos;
	conf->pwm->SEQ[1].REFRESH  = 0;
	conf->pwm->SEQ[1].ENDDELAY = 0;
	conf->pwm->SEQ[0].REFRESH  = 0;
	conf->pwm->SEQ[0].ENDDELAY = 0;
}

static void init_pwm(void)
{
	struct pwm_conf_t pwm1_conf = {
		.pwm = NRF_PWM20,
		.buf = seq0_ram,
		.buf2 = seq1_ram,
		.bufsize = 24,
		.gpio = {8, 10, 12},
	};
	pwm_config(&pwm1_conf);

	struct pwm_conf_t pwm2_conf = {
		.pwm = NRF_PWM21,
		.buf = seq0_ram_b,
		.buf2 = seq1_ram_b,
		.bufsize = 24,
		.gpio = {9, 11, 13},
	};
	pwm_config(&pwm2_conf);

}

static void init_ppi(void)
{
	// Use PPI channel C to start both PWM channels
	NRF_PWM20->SUBSCRIBE_DMA.SEQ[0].START = PPI_CH_EN | PPI_CH_A;
	NRF_PWM21->SUBSCRIBE_DMA.SEQ[0].START = PPI_CH_EN | PPI_CH_A;
	MY_TIMER->PUBLISH_COMPARE[0] = PPI_CH_EN | PPI_CH_A;
	NRF_DPPIC20->CHENSET = BIT(PPI_CH_A);

	// Use PPI channel B to trigger next step 
	NRF_PWM20->SUBSCRIBE_NEXTSTEP = PPI_CH_EN | PPI_CH_B;
	NRF_PWM21->SUBSCRIBE_NEXTSTEP = PPI_CH_EN | PPI_CH_B;
	MY_TIMER->PUBLISH_COMPARE[2] = PPI_CH_EN | PPI_CH_B;
	MY_TIMER->PUBLISH_COMPARE[3] = PPI_CH_EN | PPI_CH_B;
	NRF_DPPIC20->CHENSET = BIT(PPI_CH_B);
}

static volatile bool irq_set = false;

static void my_timer_irq_handler(void)
{
	if(MY_TIMER->EVENTS_COMPARE[0]) {
		MY_TIMER->EVENTS_COMPARE[0] = 0;
		NRF_DPPIC20->CHENCLR = BIT(PPI_CH_A);
		MY_TIMER->INTENCLR = TIMER_INTENSET_COMPARE1_Msk;
	} 	
}

static void init_timer(void)
{
	MY_TIMER->PRESCALER = 4;
	MY_TIMER->BITMODE = 3; // 32 bit
	MY_TIMER->CC[0] = 1000;
	MY_TIMER->CC[2] = 10000;
	MY_TIMER->CC[3] = 10000;
	MY_TIMER->EVENTS_COMPARE[0] = 0;
	MY_TIMER->SHORTS = TIMER_SHORTS_COMPARE2_CLEAR_Msk | TIMER_SHORTS_COMPARE3_CLEAR_Msk;
	MY_TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	IRQ_CONNECT(TIMER20_IRQn, 0, my_timer_irq_handler, 0, 0);
	irq_enable(TIMER20_IRQn);
}

K_SEM_DEFINE(sem_msg_from_cm33, 0, 1);

int *static_ram_pointer = (int*)0x2003C000;
char msg_buf[32];

static pwm_3p_config_t pwm_settings;
static pwm_3p_config_t pwm_settings_current = {.duty_cycle = 140, .steptime_us = 10000};

static void vpr_irq_handler(void)
{
	k_sem_give(&sem_msg_from_cm33);
	csr_write(VPRCSR_NORDIC_TASKS, 0);
	NRF_P1->OUTCLR = BIT(14);
	memcpy(&pwm_settings, &static_ram_pointer[1], sizeof(pwm_settings));
	if (pwm_settings.steptime_us < pwm_settings_current.steptime_us) {
		MY_TIMER->CC[3] = pwm_settings.steptime_us;
	} else if (pwm_settings.steptime_us > pwm_settings_current.steptime_us) {
		MY_TIMER->CC[2] = pwm_settings.steptime_us;
		MY_TIMER->CC[3] = pwm_settings.steptime_us;
	}
	if (pwm_settings_current.duty_cycle != pwm_settings.duty_cycle) {
		set_sequence_values(pwm_settings.duty_cycle);
	}
	pwm_settings_current = pwm_settings;
	NRF_P1->OUTSET = BIT(14);
}	

static void rcv_from_arm_init(void)
{
	IRQ_CONNECT(VPRCLIC_16_IRQn, 0, vpr_irq_handler, 0, 0);
	irq_enable(VPRCLIC_16_IRQn);
}

int main(void)
{
	printf("VPR peripheral test2\n");
	
	rcv_from_arm_init();

	init_gpio();

	init_timer();

	set_sequence_values(400);

	init_pwm();

	init_ppi();
	
	MY_TIMER->TASKS_START = 1;

	k_msleep(1);

	while (1) {
		if (k_sem_take(&sem_msg_from_cm33, K_MSEC(1000)) == 0) {
			printf("MSG received form CM33: Duty cycle: %i, steptime %i\n", pwm_settings_current.duty_cycle, pwm_settings_current.steptime_us);
			NRF_P1->OUTCLR = BIT(14);
		}
	}

	return 0;
}
