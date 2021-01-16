/*-
 * Copyright (c) 2018-2020 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/systm.h>
#include <sys/console.h>
#include <sys/thread.h>

#include <mips/microchip/pic32.h>
#include <mips/microchip/pic32mm.h>

#include <machine/cpuregs.h>
#include <machine/cpufunc.h>

PIC32MM_DEVCFG;

#include <k-thermocouple-lib/thermocouple.h>

/* Software contexts */
static struct pic32_uart_softc uart_sc;
static struct pic32_port_softc port_sc;
static struct pic32_pps_softc pps_sc;
static struct pic32_ccp_softc ccp_sc;
static struct pic32_adc_softc adc_sc;
static struct pic32_cdac_softc cdac_sc;

// RB12 SDA
// RB13 SCL

void
udelay(uint32_t usec)
{

	pic32_ccp_delay(&ccp_sc, usec);
}

void
uart_putchar(int c, void *arg)
{
	struct pic32_uart_softc *sc;

	sc = arg;

	if (c == '\n')
		pic32_putc(sc, '\r');

	pic32_putc(sc, c);
}

#define C0_COMPARE      11 
#define C0_COUNT            9 

void
pic32_led_b(struct pic32_port_softc *sc, uint8_t enable)
{
	uint32_t reg;

	pic32_port_ansel(sc, PORT_B, 8, 1);
	pic32_port_tris(sc, PORT_B, 8, PORT_OUTPUT);
	pic32_port_lat(sc, PORT_B, 8, enable);
}

void
pic32_led_w(struct pic32_port_softc *sc, uint8_t enable)
{
	uint32_t reg;

	pic32_port_ansel(sc, PORT_A, 3, 1);
	pic32_port_tris(sc, PORT_A, 3, PORT_OUTPUT);
	pic32_port_lat(sc, PORT_A, 3, enable);
}

void
pic32_gate(struct pic32_port_softc *sc, uint8_t enable)
{
	uint32_t reg;

	//printf("%s: %d\n", __func__, enable);
	/* Relay */
	pic32_port_ansel(sc, PORT_B, 9, 1);
	pic32_port_tris(sc, PORT_B, 9, PORT_OUTPUT);
	pic32_port_lat(sc, PORT_B, 9, enable);
}

void
solder_ports_init(struct pic32_port_softc *sc)
{

	/* UART1 digital */
	//pic32_port_ansel(&port_sc, PORT_B, 14, 1);
	//pic32_port_ansel(&port_sc, PORT_B, 15, 1);

	/* LED light */
	pic32_port_lat(&port_sc, PORT_C, 9, 0);
	pic32_port_tris(&port_sc, PORT_C, 9, PORT_OUTPUT);
}

static void
i2c_sda(struct pic32_port_softc *sc, uint8_t enable)
{

	if (enable)
		pic32_port_tris(sc, PORT_B, 12, PORT_INPUT);
	else
		pic32_port_tris(sc, PORT_B, 12, PORT_OUTPUT);
}

static void
i2c_scl(struct pic32_port_softc *sc, uint8_t enable)
{

	if (enable)
		pic32_port_tris(sc, PORT_B, 13, PORT_INPUT);
	else
		pic32_port_tris(sc, PORT_B, 13, PORT_OUTPUT);
}

static void
i2c_start_condition(struct pic32_port_softc *sc)
{

	i2c_scl(sc, 1);
	i2c_sda(sc, 1);
	udelay(5);
	i2c_sda(sc, 0);
	udelay(5);
	i2c_scl(sc, 0);
	udelay(5);
}

static void
i2c_stop_condition(struct pic32_port_softc *sc)
{

	i2c_sda(sc, 0);
	udelay(5);
	i2c_scl(sc, 1);
	udelay(5);
	i2c_sda(sc, 1);
	udelay(5);
}

static void
i2c_write_bit(struct pic32_port_softc *sc, uint8_t b)
{

	if (b > 0)
		i2c_sda(sc, 1);
	else
		i2c_sda(sc, 0);

	udelay(5);
	i2c_scl(sc, 1);
	udelay(5);
	i2c_scl(sc, 0);
}

static uint8_t
i2c_read_bit(struct pic32_port_softc *sc)
{
	uint8_t b;

	i2c_sda(sc, 1);
	udelay(5);
	i2c_scl(sc, 1);
	udelay(5);

	if (pic32_port_port(sc, PORT_B, 12))
		b = 1;
	else
		b = 0;

	i2c_scl(sc, 0);
 
	return (b);
}

static int
i2c_write_byte(struct pic32_port_softc *sc,
    uint8_t b, int start, int stop)
{
	uint8_t ack;
	uint8_t i;

	ack = 0;

	if (start)
		i2c_start_condition(sc);

	for (i = 0; i < 8; i++) {
		i2c_write_bit(sc, (b & 0x80));// write the most-significant bit
		b <<= 1;
	}
 
	ack = i2c_read_bit(sc);

	if (stop)
		i2c_stop_condition(sc);

	return (ack);
}

static uint8_t
i2c_read_byte(struct pic32_port_softc *sc,
    int ack, int stop)
{
	uint8_t b;
	uint8_t i;

	b = 0;

	for (i = 0; i < 8; i++) {
		b <<= 1;
		b |= i2c_read_bit(sc);
	}

	if (ack)
		i2c_write_bit(sc, 0);
	else
		i2c_write_bit(sc, 1);

	if (stop)
		i2c_stop_condition(sc);

	return (b);
}

static void
i2c_write_config(struct pic32_port_softc *sc, uint8_t cfg)
{

}

int
get_mv_single(struct pic32_port_softc *sc, uint16_t *result)
{
	int b0, b1, b2, b3;
	uint16_t mv;
	int error;

	error = i2c_write_byte(sc, 0xd0 | 1, 1, 0);
	if (error == 0) {
		b2 = i2c_read_byte(sc, 1, 0);
		b1 = i2c_read_byte(sc, 1, 0);
		b0 = i2c_read_byte(sc, 1, 1);
		//printf("%s: read %02x %02x %02x\n", __func__, b2, b1, b0);

		mv = (b2 & 0xff) << 8;
		mv |= (b1 & 0xff);
		*result = mv;
	}

	return (error);
}

static int
hotair_main(void)
{
	struct pic32_port_softc *sc;
	int vr1, vr2;
	uint8_t cfg;
	int ret;

	sc = &port_sc;

	printf("Hello world\n");
	pic32_port_ansel(sc, PORT_B, 12, 1);
	pic32_port_ansel(sc, PORT_B, 13, 1);

	//pic32_port_tris(sc, PORT_B, 12, PORT_OUTPUT);
	//pic32_port_tris(sc, PORT_B, 13, PORT_OUTPUT);
	//pic32_port_tris(sc, PORT_B, 12, PORT_INPUT);
	//pic32_port_tris(sc, PORT_B, 13, PORT_INPUT);

	//port input drives 1
	//port output drives 0

#if 0
	pic32_port_ansel(sc, PORT_A, 0, 0);
	pic32_port_tris(sc, PORT_A, 0, PORT_INPUT);
	pic32_port_ansel(sc, PORT_A, 1, 0);
	pic32_port_tris(sc, PORT_A, 1, PORT_INPUT);
#endif

	pic32_port_ansel(sc, PORT_B, 14, 0);
	pic32_port_tris(sc, PORT_B, 14, PORT_OUTPUT);

	pic32_cdac_init(&cdac_sc, CDAC1_BASE);

	ret = i2c_write_byte(sc, 0xd0, 1, 0);
	if (ret == 0) {
		cfg = 0x10 | (0 << 2);
		ret = i2c_write_byte(sc, cfg, 0, 1);
		if (ret == 0)
			printf("cfg wrotten\n");
	}

	uint16_t mv;
	uint32_t celsius;

	//pic32_port_tris(&port_sc, PORT_B, 15, PORT_INPUT);
	//pic32_port_tris(&port_sc, PORT_B, 15, PORT_OUTPUT);
	//pic32_port_lat(&port_sc, PORT_B, 15, 1);
	//pic32_port_cnpu(&port_sc, PORT_B, 15, 1);
	//pic32_port_cnpd(&port_sc, PORT_B, 15, 1);
	//pic32_port_odc(&port_sc, PORT_B, 15, 0);

#if 0
	pic32_port_ansel(sc, PORT_A, 0, 0);
	pic32_port_tris(sc, PORT_A, 0, PORT_INPUT);
	//pic32_port_tris(sc, PORT_A, 0, PORT_OUTPUT);

	pic32_port_ansel(sc, PORT_A, 1, 0);
	pic32_port_tris(sc, PORT_A, 1, PORT_INPUT);
	//pic32_port_tris(sc, PORT_A, 1, PORT_OUTPUT);
#endif

	pic32_port_ansel(sc, PORT_A, 3, 0);
	pic32_port_tris(sc, PORT_A, 3, PORT_INPUT);
	pic32_port_tris(sc, PORT_A, 3, PORT_OUTPUT);

	/* Button ADC */
	pic32_port_ansel(sc, PORT_B, 15, 0);
	pic32_port_tris(sc, PORT_B, 15, PORT_INPUT);

	/* VR1 */
	pic32_port_ansel(sc, PORT_B, 2, 0);
	pic32_port_tris(sc, PORT_B, 2, PORT_INPUT);

	/* VR2 */
	pic32_port_ansel(sc, PORT_A, 2, 0);
	pic32_port_tris(sc, PORT_A, 2, PORT_INPUT);

	pic32_port_ansel(sc, PORT_B, 3, 0);
	pic32_port_tris(sc, PORT_B, 3, PORT_INPUT);
	pic32_port_tris(sc, PORT_B, 3, PORT_OUTPUT);

	pic32_adc_init(&adc_sc, ADC1_BASE);

	uint32_t val;
	uint32_t val_prev;
	int enable;

	enable = 0;

#if 0
	pic32_cdac_control(&cdac_sc, 102);
	pic32_gate(&port_sc, 1);
	pic32_led_w(&port_sc, 1);
	pic32_led_b(&port_sc, 1);
	while (1)
		printf("test\n");
#endif

	int mv_zero_count;
	int enable_count;
	int error;

	mv_zero_count = 0;
	enable_count = 0;

	//pic32_port_ansel(sc, PORT_A, 1, 0);
	//pic32_port_tris(sc, PORT_A, 1, PORT_INPUT);

	while (1) {
		//printf("char %d\n", pic32_getc(&uart_sc));
		val = pic32_adc_convert(&adc_sc, 10);
		if (val < 50) {
			if (enable_count >= 0) {
				if (enable) {
					/* Quick turn off. */
					enable = 0;
				} else {
					/* Slow enable. */
					if (enable_count++ > 3) {
						enable = 1;
						enable_count = -3;
					}
				}
			}
			printf("val %2d, enable %d, (enable_count %2d)\n",
			    val, enable, enable_count);
		} else
			if (enable_count < 0)
				enable_count++;
			else
				enable_count = 0;

		udelay(40000);

		error = get_mv_single(sc, &mv);
		if (error != 0)
			continue;
		if (mv > 8000) {
			/* Read error ? */
			printf("Error reading, val %d\n", mv);
			continue;
		}

		vr1 = pic32_adc_convert(&adc_sc, 4); /* heat */
		vr1 = (4095 - vr1) / 204;

		vr2 = pic32_adc_convert(&adc_sc, 5); /* air */
		vr2 = ((4095 - vr2) / 256) + 6;

		celsius = thermocoupleMvToC(mv);
		printf("%s: btn %2d, heat %d, air %d, en %d, mv %2d (%3dC)\n",
		    __func__, val, vr1, vr2, enable, mv, celsius);

		if (mv >= 22) {
			/* Fault */
			pic32_cdac_control(&cdac_sc, 0x3f);
			pic32_led_b(&port_sc, 1);
			pic32_gate(&port_sc, 0);
		} else if (enable) {
			pic32_led_w(&port_sc, 1);
			pic32_led_b(&port_sc, 0);
			pic32_cdac_control(&cdac_sc, vr2);
			if (mv < vr1)
				pic32_gate(&port_sc, 1);
			else
				pic32_gate(&port_sc, 0);
		} else if (mv >= 0 && mv <= 2) {
			pic32_led_w(&port_sc, 0);
			pic32_led_b(&port_sc, 0);
			pic32_gate(&port_sc, 0);
			if (mv_zero_count++ > 50) {
				pic32_cdac_control(&cdac_sc, 0);
				mv_zero_count = 0;
			}
		} else {
			mv_zero_count = 0;
			pic32_led_w(&port_sc, 0);
			pic32_led_b(&port_sc, 0);
			pic32_gate(&port_sc, 0);
			pic32_cdac_control(&cdac_sc, 6);
		}
	}
}

void
board_init(void)
{
	uint32_t reg;
	int i;

	mtc0(C0_COUNT, 0, 0);
	mtc0(C0_COMPARE, 0, -1);
	mtc0(12, 0, 1);

	pic32_pps_init(&pps_sc, PPS_BASE);

	pic32_port_init(&port_sc, PORTS_BASE);
	solder_ports_init(&port_sc);

	//TX
	pic32_port_ansel(&port_sc, PORT_A, 0, 1);
	*(volatile uint32_t *)0xBF802590 = (1 << 0); //rp1 TX

	//RX
	pic32_port_ansel(&port_sc, PORT_A, 1, 1);
	pic32_port_tris(&port_sc, PORT_A, 1, PORT_INPUT);
	*(volatile uint32_t *)0xBF802520 = (2 << 16); //rpinr9 RX RP2

	pic32_uart_init(&uart_sc, UART2_BASE, 19200, 8000000, 1);
	mdx_console_register(uart_putchar, (void *)&uart_sc);

	pic32_ccp_init(&ccp_sc, CCP1_BASE, 122000);
}

int
main(void)
{

	hotair_main();

	/* NOT REACHED */

	panic("reached unreached place");
}
