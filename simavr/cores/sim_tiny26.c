/*
    sim_tiny26.c

    Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>
                         Jon Escombe <lists@dresco.co.uk>

     This file is part of simavr.

    simavr is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    simavr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sim_avr.h"
#include "avr_eeprom.h"
#include "avr_watchdog.h"
#include "avr_extint.h"
#include "avr_ioport.h"
#include "avr_timer.h"
#include "avr_adc.h"
#include "avr_acomp.h"
#include "avr_usi.h"

#define SIM_VECTOR_SIZE	2
#define _AVR_IO_H_
#define __ASSEMBLER__
#include "avr/iotn26.h"
// instantiate the new core
#include "sim_core_declare.h"

static void init(struct avr_t * avr);
static void reset(struct avr_t * avr);

static const struct mcu_t {
	avr_t core;
	avr_eeprom_t 	eeprom;
	avr_watchdog_t	watchdog;
	avr_extint_t	extint;
	avr_ioport_t	porta, portb;
	avr_timer_t		timer0, timer1;
	avr_acomp_t		acomp;
	avr_adc_t       adc;
	avr_usi_t		usi;
} mcu = {
	.core = {
		.mmcu = "attiny26",
		DEFAULT_CORE(SIM_VECTOR_SIZE),
		.init = init,
		.reset = reset,
	},
    #define WDIF INTF0
    #define WDIE INT0
    #define EEPE EERE
    #define EEMPE EERE
    #define EEPM1 EERE
    #define EEPM0 EERE
	AVR_EEPROM_DECLARE_8BIT(EE_RDY_vect),
	.extint = {
		AVR_EXTINT_TINY_DECLARE(0, 'B', PB6, GIFR),
	},
	.porta = {
			.name = 'A',  .r_port = PORTA, .r_ddr = DDRA, .r_pin = PINA,
			.pcint = {
				.enable = AVR_IO_REGBIT(GIMSK, PCIE1),
				.raised = AVR_IO_REGBIT(GIFR, PCIF),
				.vector = IO_PINS_vect,
			},
	},
	.portb = {
		.name = 'B',  .r_port = PORTB, .r_ddr = DDRB, .r_pin = PINB,
		.pcint = {
			.enable = AVR_IO_REGBIT(GIMSK, PCIE0),
			.raised = AVR_IO_REGBIT(GIFR, PCIF),
			.vector = IO_PINS_vect,
		},
	},
	.timer0 = {
		.name = '0',
		.cs = { AVR_IO_REGBIT(TCCR0, CS00), AVR_IO_REGBIT(TCCR0, CS01), AVR_IO_REGBIT(TCCR0, CS02) },
		.cs_div = { 0, 0, 3 /* 8 */, 6 /* 64 */, 8 /* 256 */, 10 /* 1024 */ },

		.r_tcnt = TCNT0,

		.overflow = {
			.enable = AVR_IO_REGBIT(TIMSK, TOIE0),
			.raised = AVR_IO_REGBIT(TIFR, TOV0),
			.vector = TIMER0_OVF0_vect,
		},
	},
	.timer1 = {
			.name = '1',
			.cs = { AVR_IO_REGBIT(TCCR1B, CS10), AVR_IO_REGBIT(TCCR1B, CS11), AVR_IO_REGBIT(TCCR1B, CS12), AVR_IO_REGBIT(TCCR1B, CS13) },
			.cs_div = { 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 },

			.r_tcnt = TCNT1,

			.overflow = {
				.enable = AVR_IO_REGBIT(TIMSK, TOIE1),
				.raised = AVR_IO_REGBIT(TIFR, TOV1),
				.vector = TIMER1_OVF1_vect,
			},
			.comp = {
			            [AVR_TIMER_COMPA] = {
			                .r_ocr = OCR1A,
			                .com = AVR_IO_REGBITS(TCCR1A, COM1A0, 0x3),
			                .com_pin = AVR_IO_REGBIT(PORTB, 1),
			                .interrupt = {
			                    .enable = AVR_IO_REGBIT(TIMSK, OCIE1A),
			                    .raised = AVR_IO_REGBIT(TIFR, OCF1A),
			                    .vector = TIMER1_CMPA_vect,
			                },
			            },
			            [AVR_TIMER_COMPB] = {
			                .r_ocr = OCR1B,
			                .com = AVR_IO_REGBITS(TCCR1A, COM1B0, 0x3),
			                .com_pin = AVR_IO_REGBIT(PORTB, 3),
			                .interrupt = {
			                    .enable = AVR_IO_REGBIT(TIMSK, OCIE1B),
			                    .raised = AVR_IO_REGBIT(TIFR, OCF1B),
			                    .vector = TIMER1_CMPB_vect,
			                },
			            },
			        },
		},

	.acomp = {
		.mux_inputs = 11,
		.mux = { AVR_IO_REGBIT(ADMUX, MUX0), AVR_IO_REGBIT(ADMUX, MUX1), AVR_IO_REGBIT(ADMUX, MUX2), AVR_IO_REGBIT(ADMUX, MUX3), },
		.aden = AVR_IO_REGBIT(ADCSR, ADEN),
		.acme = AVR_IO_REGBIT(ACSR, ACME),

		.r_acsr = ACSR,
		.acis = { AVR_IO_REGBIT(ACSR, ACIS0), AVR_IO_REGBIT(ACSR, ACIS1) },
		.aco = AVR_IO_REGBIT(ACSR, ACO),
		.acbg = AVR_IO_REGBIT(ACSR, ACBG),
		.disabled = AVR_IO_REGBIT(ACSR, ACD),

		.ac = {
			.enable = AVR_IO_REGBIT(ACSR, ACIE),
			.raised = AVR_IO_REGBIT(ACSR, ACI),
			.vector = ANA_COMP_vect,
		}
	},

	.adc = {
		.r_admux = ADMUX,
		.mux = { AVR_IO_REGBIT(ADMUX, MUX0),
			 AVR_IO_REGBIT(ADMUX, MUX1),
			 AVR_IO_REGBIT(ADMUX, MUX2),
			 AVR_IO_REGBIT(ADMUX, MUX3),
			 AVR_IO_REGBIT(ADMUX, MUX4),},
		.ref = { AVR_IO_REGBIT(ADMUX, REFS0), AVR_IO_REGBIT(ADMUX, REFS1),},
		.ref_values = {
			[0] = ADC_VREF_VCC, [1] = ADC_VREF_AREF,
			[2] = ADC_VREF_V256, [3] = ADC_VREF_V256,
		},

		.adlar = AVR_IO_REGBIT(ADMUX, ADLAR),
		.r_adcsra = ADCSR,
		.aden = AVR_IO_REGBIT(ADCSR, ADEN),
		.adsc = AVR_IO_REGBIT(ADCSR, ADSC),
		.adps = { AVR_IO_REGBIT(ADCSR, ADPS0),
			  AVR_IO_REGBIT(ADCSR, ADPS1),
			  AVR_IO_REGBIT(ADCSR, ADPS2),},

		.r_adch = ADCH,
		.r_adcl = ADCL,
		.adts = { AVR_IO_REGBIT(ADCSR, ADFR),},
		.adts_op = {
			[0] = avr_adts_none,
			[1] = avr_adts_free_running,
		},
		.muxmode = {
			[0] = AVR_ADC_SINGLE(0), [1] = AVR_ADC_SINGLE(1),
			[2] = AVR_ADC_SINGLE(2), [3] = AVR_ADC_SINGLE(3),
			[4] = AVR_ADC_SINGLE(4), [5] = AVR_ADC_SINGLE(5),
			[6] = AVR_ADC_SINGLE(6), [7] = AVR_ADC_SINGLE(7),
			[8] = AVR_ADC_SINGLE(8), [9] = AVR_ADC_SINGLE(9),
			[10] = AVR_ADC_SINGLE(10), [11] = AVR_ADC_SINGLE(11),
			[12] = AVR_ADC_SINGLE(12), [13] = AVR_ADC_SINGLE(13),
			[14] = AVR_ADC_SINGLE(14), [15] = AVR_ADC_SINGLE(15),
			[16] = AVR_ADC_SINGLE(16), [17] = AVR_ADC_SINGLE(17),
			[18] = AVR_ADC_SINGLE(18), [19] = AVR_ADC_SINGLE(19),
		},

		.adc = {
			.enable = AVR_IO_REGBIT(ADCSR, ADIE),
			.raised = AVR_IO_REGBIT(ADCSR, ADIF),
			.vector = ADC_vect,
		},
	},
    #define USIBR USIDR
    #define USI_START_vect USI_STRT_vect
	AVR_USI_DECLARE('B', PORTB, 0, 1, 2),
};

static avr_t * make()
{
	return avr_core_allocate(&mcu.core, sizeof(struct mcu_t));
}

avr_kind_t tiny26 = {
	.names = { "attiny26" },
	.make = make
};

static void init(struct avr_t * avr)
{
	struct mcu_t * mcu = (struct mcu_t*)avr;
	avr_extint_init(avr, &mcu->extint);
	avr_ioport_init(avr, &mcu->porta);
	avr_ioport_init(avr, &mcu->portb);
	avr_timer_init(avr, &mcu->timer0);
	avr_timer_init(avr, &mcu->timer1);
	avr_acomp_init(avr, &mcu->acomp);
	avr_adc_init(avr, &mcu->adc);
	avr_usi_init(avr, &mcu->usi);
}

static void reset(struct avr_t * avr)
{
//	struct mcu_t * mcu = (struct mcu_t*)avr;
}
