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

#include <dev/i2c/bitbang/i2c_bitbang.h>

#include <mips/microchip/pic32.h>
#include <mips/microchip/pic32mm.h>

#include <machine/cpuregs.h>
#include <machine/cpufunc.h>

PIC32MM_DEVCFG;

#include <k-thermocouple-lib/thermocouple.h>

/* Software contexts */
static struct pic32_uart_softc uart_sc;
static struct pic32_port_softc port_sc;
static struct pic32_ccp_softc ccp_sc;
static struct pic32_adc_softc adc_sc;
static struct pic32_cdac_softc cdac_sc;
static struct i2c_bitbang_softc i2c_bitbang_sc;
static struct mdx_device dev_bitbang = { .sc = &i2c_bitbang_sc };

#define	C0_COMPARE	11
#define	C0_COUNT	9

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

	/* Relay */
	pic32_port_ansel(sc, PORT_B, 9, 1);
	pic32_port_tris(sc, PORT_B, 9, PORT_OUTPUT);
	pic32_port_lat(sc, PORT_B, 9, enable);
}

void
hotair_ports_init(struct pic32_port_softc *sc)
{

	/* LED light */
	pic32_port_lat(&port_sc, PORT_C, 9, 0);
	pic32_port_tris(&port_sc, PORT_C, 9, PORT_OUTPUT);
}

static void
i2c_sda(void *arg, bool enable)
{
	struct pic32_port_softc *sc;

	sc = &port_sc;

	if (enable)
		pic32_port_tris(sc, PORT_B, 12, PORT_INPUT);
	else
		pic32_port_tris(sc, PORT_B, 12, PORT_OUTPUT);
}

static void
i2c_scl(void *arg, bool enable)
{
	struct pic32_port_softc *sc;

	sc = &port_sc;

	if (enable)
		pic32_port_tris(sc, PORT_B, 13, PORT_INPUT);
	else
		pic32_port_tris(sc, PORT_B, 13, PORT_OUTPUT);
}

static int
i2c_sda_val(void *arg)
{
	struct pic32_port_softc *sc;

	sc = &port_sc;

	if (pic32_port_port(sc, PORT_B, 12))
		return (1);

	return (0);
}

static int
mcp3421_configure(uint8_t slave)
{
	struct i2c_msg msgs[1];
	uint8_t cfg;
	int ret;

	/* Configure the MCP3421. */
	cfg = 0x10 | (0 << 2);
	msgs[0].slave = slave;
	msgs[0].buf = &cfg;
	msgs[0].len = 1;
	msgs[0].flags =	0;
	ret = mdx_i2c_transfer(&dev_bitbang, msgs, 1);
	if (ret != 0) {
		printf("%s: could not configure mcp3421, slave %x\n",
		    __func__, slave);
		return (ret);
	}

	printf("cfg written\n");

	return (0);
}

static int
get_mv(struct pic32_port_softc *sc, uint8_t unit, uint16_t *result)
{
	struct i2c_msg msgs[1];
	uint8_t data[3];
	int b0, b1, b2;
	uint16_t mv;
	int ret;

	if (unit == 0)
		msgs[0].slave = 0x68;
	else
		msgs[0].slave = 0x69;

	msgs[0].buf = data;
	msgs[0].len = 3;
	msgs[0].flags = IIC_M_RD;

	ret = mdx_i2c_transfer(&dev_bitbang, msgs, 1);

	if (ret == 0) {
		b2 = data[0];
		b1 = data[1];
		b0 = data[2];

		//printf("%s(%d): read %d %d %02x\n", __func__,
		//    unit, b2, b1, b0);
		mv = b2 << 8 | b1;
		*result = mv;
	}

	return (ret);
}

static void
hotair_init(struct pic32_port_softc *sc)
{
	uint8_t cfg;
	int error;

	pic32_port_ansel(sc, PORT_B, 12, 1);
	pic32_port_ansel(sc, PORT_B, 13, 1);
	pic32_port_ansel(sc, PORT_B, 14, 0);
	pic32_port_tris(sc, PORT_B, 14, PORT_OUTPUT);

	pic32_cdac_init(&cdac_sc, CDAC1_BASE);

	error = mcp3421_configure(0x68);
	if (error != 0)
		panic("could not configure a0");

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
}

static int
hotair_main(struct pic32_port_softc *sc)
{
	uint32_t celsius;
	uint32_t val;
	uint16_t mv;
	int mv_zero_count;
	int enable_count;
	int vr1, vr2;
	int enable;
	int error;

	enable = 0;
	mv_zero_count = 0;
	enable_count = 0;

	while (1) {
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

		error = get_mv(sc, 0, &mv);
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

static struct i2c_bitbang_ops i2c_ops = {
	.i2c_scl = &i2c_scl,
	.i2c_sda = &i2c_sda,
	.i2c_sda_val = &i2c_sda_val,
};

void
board_init(void)
{
	uint32_t reg;
	int i;

	mtc0(C0_COUNT, 0, 0);
	mtc0(C0_COMPARE, 0, -1);
	mtc0(12, 0, 1);

	pic32_port_init(&port_sc, PORTS_BASE);
	hotair_ports_init(&port_sc);

	/* UART TX */
	pic32_port_ansel(&port_sc, PORT_A, 0, 1);
	*(volatile uint32_t *)0xBF802590 = (1 << 0); //rp1 TX

	/* UART RX */
	pic32_port_ansel(&port_sc, PORT_A, 1, 1);
	pic32_port_tris(&port_sc, PORT_A, 1, PORT_INPUT);
	*(volatile uint32_t *)0xBF802520 = (2 << 16); //rpinr9 RX RP2

	pic32_uart_init(&uart_sc, UART2_BASE, 19200, 8000000, 1);
	mdx_console_register(uart_putchar, (void *)&uart_sc);

	pic32_ccp_init(&ccp_sc, CCP1_BASE, 122000);
	i2c_bitbang_init(&dev_bitbang, &i2c_ops);
}

int
main(void)
{

	printf("Hello world\n");

	hotair_init(&port_sc);
	hotair_main(&port_sc);

	/* NOT REACHED */

	panic("reached unreached place");
}
