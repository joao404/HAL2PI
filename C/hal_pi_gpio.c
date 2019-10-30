/********************************************************************
* Description:  hal_pi_gpio.c
*               This file, 'hal_pi_gpio.c', is a HAL component that 
*               provides a driver for the GPIOs of a raspberry pi with the help of gpiod lib and /dev/gpiochip0.
*				Based on hal_parport.c
*
* Author: Marcel Maage
* License: MIT
*    
* Copyright (c) 2019 All rights reserved.
*
* Last change: 
********************************************************************/

/*
TODO:
-Adding reset time and functions
-Adding configuration of inverse signal for input and output
*/



/** This file, 'hal_pi_gpio.c', is a HAL component that provides a
    driver for the GPIO port of the Raspberry Pi based on libgpiod.

    The configuration is similar to the parallel port driver and is driven
	into the variable cfg="". In the config-string the key words 
	"out" and "in" are allowed only ones. After the keyword follows the 
	gpio numbers.
    Example command lines are as follows:

	loadrt 	hal_pi_gpio cfg="in 17 22 out 18 23"

    The driver creates HAL pins and parameters for each pin
    as follows:
    Each physical output has a correspinding HAL pin, named
    'hal_pi_gpio.pin-<pinnum>-out', and a HAL parameter
    'hal_pi_gpio.pin-<pinnum>-out-invert'.
    Each physical input has two corresponding HAL pins, named
    'hal_pi_gpio.pin-<pinnum>-in' and
    'hal_pi_gpio.pin-<pinnum>-in-not'.

	<pinnum> is the physical pin number on the GPIO connector.

    The realtime version of the driver exports two HAL functions for
    each pin, 'hal_pi_gpio.<pinnum>.read' and 'hal_pi_gpio.<pinnum>.write'.
    It also exports two additional functions, 'hal_pi_gpio.read-all' and
    'hal_pi_gpio.write-all'.  Any or all of these functions can be added
    to realtime HAL threads to update the pin data periodically.
*/

/** MIT License

Copyright (c) 2019 Marcel Maage

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_ctype.h"	/* isspace() */
#include "rtapi_app.h"		/* RTAPI realtime module decls */

#include "hal.h"		/* HAL public API decls */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gpiod.h>

/* If FASTIO is defined, uses outb() and inb() from <asm.io>,
   instead of rtapi_outb() and rtapi_inb() - the <asm.io> ones
   are inlined, and save a microsecond or two (on my 233MHz box)
*/
#include <rtapi_io.h>

/* module information */
MODULE_AUTHOR("Marcel Maage");
MODULE_DESCRIPTION("GPIO Raspberry Pi Driver for EMC HAL");
MODULE_LICENSE("GPL");
static char *cfg = "0x0278";	/* config string, default 1 output port at 278 */
RTAPI_MP_STRING(cfg, "config string");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data needed by the
   parallel port driver for a single port
*/

// typedef struct {
    // unsigned short base_addr;	/* base I/O address (0x378, etc.) */
    // unsigned char data_dir;	/* non-zero if pins 2-9 are input */
    // unsigned char use_control_in; /* non-zero if pins 1, 4, 16, 17 are input */ 
    // hal_bit_t *status_in[10];	/* ptrs for in pins 15, 13, 12, 10, 11 */
    // hal_bit_t *data_in[16];	/* ptrs for input pins 2 - 9 */
    // hal_bit_t *data_out[8];	/* ptrs for output pins 2 - 9 */
    // hal_bit_t data_inv[8];	/* polarity params for output pins 2 - 9 */
    // hal_bit_t data_reset[8];	/* reset flag for output pins 2 - 9 */
    // hal_bit_t *control_in[8];	/* ptrs for in pins 1, 14, 16, 17 */
    // hal_bit_t *control_out[4];	/* ptrs for out pins 1, 14, 16, 17 */
    // hal_bit_t control_inv[4];	/* pol. params for output pins 1, 14, 16, 17 */
    // hal_bit_t control_reset[4];	/* reset flag for output pins 1, 14, 16, 17 */
    // hal_u32_t reset_time;       /* min ns between write and reset */
    // hal_u32_t debug1, debug2;
    // long long write_time;
    // unsigned char outdata;
    // unsigned char reset_mask;       /* reset flag for pin 2..9 */
    // unsigned char reset_val;        /* reset values for pin 2..9 */
    // long long write_time_ctrl;
    // unsigned char outdata_ctrl;
    // unsigned char reset_mask_ctrl;  /* reset flag for pin 1, 14, 16, 17 */
    // unsigned char reset_val_ctrl;   /* reset values for pin 1, 14, 16, 17 */
    // struct hal_parport_t portdata;
// } parport_t;

typedef struct {
	int pinnum;
    hal_bit_t* data;	
	hal_bit_t data_inv;	/* polarity params */
    hal_bit_t data_reset;	/* reset flag for output */
	struct gpiod_line *line;
} gpio_t;
/* pointer to array of parport_t structs in shared memory, 1 per port */
static gpio_t *input_ptr_array[26];
static gpio_t *output_ptr_array[26];

static hal_u32_t reset_time;       /* min ns between write and reset */

struct gpiod_chip *chip;

/* other globals */
static int comp_id;		/* component ID */
static int num_inputs;		/* number of ports configured */
static int num_outputs;		/* number of ports configured */

static unsigned long ns2tsc_factor;
#define ns2tsc(x) (((x) * (unsigned long long)ns2tsc_factor) >> 12)

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

/* These are the functions that actually do the I/O
   everything else is just init code
*/
static void app_exit(void);
static void read_pin(void *arg,long period);
static void reset_pin(void *arg,long period);
static void write_pin(void *arg,long period);
static void read_all(void *arg,long period);
static void write_all(void *arg,long period);

// static int export_port(int portnum, parport_t * addr);
static int export_input_pin(int pin, hal_bit_t ** base);
static int export_output_pin(int pin, hal_bit_t ** dbase,
    hal_bit_t * pbase, hal_bit_t * rbase);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

#define MAX_PINS 26

//#define MAX_TOK ((MAX_PORTS*2)+3)
#define MAX_TOK 31 //26+in+out+3

int rtapi_app_main(void)
{
    char *cp;
    char *argv[MAX_TOK];
    char name[HAL_NAME_LEN + 1];
    int n, retval,i;


#ifdef __KERNEL__
    // this calculation fits in a 32-bit unsigned 
    // as long as CPUs are under about 6GHz
    ns2tsc_factor = (cpu_khz << 6) / 15625ul;
#else
    ns2tsc_factor = 1ll<<12;
#endif

    /* test for config string */
    if (cfg == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "hal_pi_gpio: ERROR: no config string\n");
	return -1;
    }
rtapi_print ( "config string '%s'\n", cfg );
    /* as a RT module, we don't get a nice argc/argv command line, we only
       get a single string... so we need to tokenize it ourselves */
    /* in addition, it seems that insmod under kernel 2.6 will truncate 
       a string parameter at the first whitespace.  So we allow '_' as
       an alternate token separator. */
    cp = cfg;
    for (n = 0; n < MAX_TOK; n++) {
	/* strip leading whitespace */
	while ((*cp != '\0') && ( isspace(*cp) || ( *cp == '_') ))
	    cp++;
	/* mark beginning of token */
	argv[n] = cp;
	/* find end of token */
	while ((*cp != '\0') && !( isspace(*cp) || ( *cp == '_') ))
	    cp++;
	/* mark end of this token, prepare to search for next one */
	if (*cp != '\0') {
	    *cp = '\0';
	    cp++;
	}
    }
    for (n = 0; n < MAX_TOK; n++) {
	/* is token empty? */
	if (argv[n][0] == '\0') {
	    /* yes - make pointer NULL */
	    argv[n] = NULL;
	}
    }
	
	
	//argv is ready
	
	
	// i=0;
	// // rtapi_print_msg(RTAPI_MSG_INFO,
	    // // "hal_pi_gpio:");
	// rtapi_print ( "hal_pi_gpio" );
	// while(argv[i]!=NULL)
	// {
		// // rtapi_print_msg(RTAPI_MSG_INFO,
		// rtapi_print (
	    // " %s",argv[i]);
		// i++;
	// }
	// // rtapi_print_msg(RTAPI_MSG_INFO,
	// rtapi_print (
	    // "\n");
	
	
	/* have good config info, connect to the HAL */
    comp_id = hal_init("hal_pi_gpio");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "hal_pi_gpio: ERROR: hal_init() failed\n");
	return -1;
    }
	
	
	/*connecting to hardware chip*/
	chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
			"hal_pi_gpio: ERROR: gpiod_chip_open() failed\n");
		hal_exit(comp_id);
		return-1;
	}		
	
	
	//geting argv parameters
	//input is:
	//in 18 19 21 out 22 23 24 or out 22 23 24 in 18 19 21
	//no other directions are allowed
	
	//out in out is not allowed
	
	i=0;
		
	num_inputs=0;
	num_outputs=0;
	
	
	if(argv[0]==NULL)
	{
		app_exit();
	}	
	else if(strcmp(argv[0],"in")==0)
	{
		i=1;
		while(argv[i]!=NULL)
		{
			if(strcmp(argv[i],"out")==0)
			{
				i++;
				while(argv[i]!=NULL)
				{
					/* allocate shared memory for gpio data */
					output_ptr_array[num_outputs] = hal_malloc(sizeof(gpio_t));
					if (output_ptr_array[num_outputs] == 0) {
						rtapi_print_msg(RTAPI_MSG_ERR,
							"hal_pi_gpio: ERROR: hal_malloc() failed\n");
						app_exit();
						return -1;
					}
					output_ptr_array[num_outputs]->pinnum=atoi(argv[i]);
					num_outputs++;
					i++;
				}
				break;
			}
			/* allocate shared memory for gpio data */
			input_ptr_array[num_inputs] = hal_malloc(sizeof(gpio_t));
			if (input_ptr_array[num_inputs] == 0) {
				rtapi_print_msg(RTAPI_MSG_ERR,
					"hal_pi_gpio: ERROR: hal_malloc() failed\n");
				app_exit();
				return -1;
			}
			input_ptr_array[num_inputs]->pinnum=atoi(argv[i]);
			num_inputs++;
			i++;
		}
		
	}
	else if(strcmp(argv[0],"out")==0)
	{
		i=1;
		while(argv[i]!=NULL)
		{
			if(strcmp(argv[i],"in")==0)
			{
				i++;
				while(argv[i]!=NULL)
				{
					/* allocate shared memory for gpio data */
					input_ptr_array[num_inputs] = hal_malloc(sizeof(gpio_t));
					if (input_ptr_array[num_inputs] == 0) {
						rtapi_print_msg(RTAPI_MSG_ERR,
							"hal_pi_gpio: ERROR: hal_malloc() failed\n");
						app_exit();
						return -1;
					}
					input_ptr_array[num_inputs]->pinnum=atoi(argv[i]);
					num_inputs++;
					i++;
				}
				break;
			}
			/* allocate shared memory for gpio data */
			output_ptr_array[num_outputs] = hal_malloc(sizeof(gpio_t));
			if (output_ptr_array[num_outputs] == 0) {
				rtapi_print_msg(RTAPI_MSG_ERR,
					"hal_pi_gpio: ERROR: hal_malloc() failed\n");
				app_exit();
				return -1;
			}
			output_ptr_array[num_outputs]->pinnum=atoi(argv[i]);
			num_outputs++;
			i++;
		}
	}
	else{
		app_exit();
	}
		
	// rtapi_print_msg(RTAPI_MSG_INFO,
	rtapi_print(
			"hal_pi_gpio: outputs: %d inputs %d\n",num_outputs,num_inputs);
	
	//last elemnt is zero
	output_ptr_array[num_outputs]=NULL;
	input_ptr_array[num_inputs]=NULL;
	
	//line of chip
	for(i=0;i<num_inputs;i++)
	{
		input_ptr_array[i]->line = gpiod_chip_get_line(chip,input_ptr_array[i]->pinnum);
		if (!(input_ptr_array[i]->line)) {    app_exit();return-1;}
		rtapi_snprintf(name, sizeof(name), "GPIO%02d-in", input_ptr_array[i]->pinnum);
		if(gpiod_line_request_input(input_ptr_array[i]->line,name)!=0) {    app_exit();return-1;}
	}
	
	for(i=0;i<num_outputs;i++)
	{
		output_ptr_array[i]->line = gpiod_chip_get_line(chip,output_ptr_array[i]->pinnum);
		if (!(output_ptr_array[i]->line)) {    app_exit();return-1;}
		rtapi_snprintf(name, sizeof(name), "GPIO%02d-out", output_ptr_array[i]->pinnum);
		if(gpiod_line_request_output(output_ptr_array[i]->line,name,false)!=0) {    app_exit();return-1;}
	}
	
	
	
	retval=0;
	
	//export_pins by giving the data where read/write is done to
	for(i=0;i<num_inputs;i++)
	{
		retval += export_input_pin(input_ptr_array[i]->pinnum, &(input_ptr_array[i]->data)); 
	}
	
	for(i=0;i<num_outputs;i++)
	{
		retval += export_output_pin(output_ptr_array[i]->pinnum,
			&(output_ptr_array[i]->data), &(output_ptr_array[i]->data_inv), &(output_ptr_array[i]->data_reset));
	}
	
	
	//reset time for reseting everything when time is over
	//retval += hal_param_u32_newf(HAL_RW, &reset_time, comp_id, 
	//		"hal_pi_gpio.reset-time");


	//check retval if any errors occured
	if(retval!=0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
			"hal_pi_gpio: ERROR: export_pins() failed\n");
		app_exit();
		return -1;
	}

	
	
	
	//export functions which write the data to the outputs which was set by hal
	for(i=0;i<num_inputs;i++)
	{
		/* make read function name */
		rtapi_snprintf(name, sizeof(name), "hal_pi_gpio.%02d.read", i);
		/* export read function */
		retval = hal_export_funct(name, read_pin, &(input_ptr_array[i]),
			0, 0, comp_id);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"hal_pi_gpio: ERROR: pin %d read funct export failed\n", i);
			app_exit();
			return -1;
		}
	}
	
	for(i=0;i<num_outputs;i++)
	{
		/* make write function name */
		rtapi_snprintf(name, sizeof(name), "hal_pi_gpio.%02d.write", i);
		/* export write function */
		retval = hal_export_funct(name, write_pin, &(output_ptr_array[i]),
			0, 0, comp_id);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"hal_pi_gpio: ERROR: pin %d write funct export failed\n", i);
			app_exit();
			return -1;
		}
		/* make reset function name */
		rtapi_snprintf(name, sizeof(name), "hal_pi_gpio.%02d.reset", i);
		/* export write function */
		retval = hal_export_funct(name, reset_pin, &(output_ptr_array[i]),
			0, 0, comp_id);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"hal_pi_gpio: ERROR: pin %d reset funct export failed\n", i);
			app_exit();
			return -1;
		}
	}
	
	
	
		

    /* export functions that read and write all pins */
    retval = hal_export_funct("hal_pi_gpio.read-all", read_all,
	input_ptr_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "hal_pi_gpio: ERROR: read all funct export failed\n");
	app_exit();
	return -1;
    }
    retval = hal_export_funct("hal_pi_gpio.write-all", write_all,
	output_ptr_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "hal_pi_gpio: ERROR: write all funct export failed\n");
	app_exit();
	return -1;
    }
	
	
	
	
		
    // rtapi_print_msg(RTAPI_MSG_INFO,
	rtapi_print(
	"hal_pi_gpio: installed driver \n");
    hal_ready(comp_id);
    return 0;
}

void app_exit(void)
{
    // int n;
    // for (n = 0; n < num_ports; n++) {
        // hal_parport_release(&port_data_array[n].portdata);
    // }
    gpiod_chip_close(chip);
    hal_exit(comp_id);
}

void rtapi_app_exit(void)
{
    // int n;
    // for (n = 0; n < num_ports; n++) {
        // hal_parport_release(&port_data_array[n].portdata);
    // }
    gpiod_chip_close(chip);
    hal_exit(comp_id);
}

/***********************************************************************
*                  REALTIME PORT READ AND WRITE FUNCTIONS              *
************************************************************************/

static void read_pin(void *arg,long period)
{
	*(((gpio_t*)arg)->data)=gpiod_line_get_value(((gpio_t*)arg)->line);
	//*(((gpio_t*)arg)->data_inv)=!(*(((gpio_t*)arg)->data));
    // rtapi_print_msg(RTAPI_MSG_INFO,
	    // "hal_pi_gpio: read_port\n");
}

static void reset_pin(void *arg,long period) 
{
    // gpio_t *port = arg;
    // long long deadline, reset_time_tsc;
    // unsigned char outdata = (port->outdata&~port->reset_mask) ^ port->reset_val;
   
    // if(reset_time > period/4) reset_time = period/4;
    // reset_time_tsc = ns2tsc(reset_time);

    // if(outdata != port->outdata) {
        // deadline = port->write_time + reset_time_tsc;
        // while(rtapi_get_clocks() < deadline) {}
        // rtapi_outb(outdata, port->base_addr);
    // }

    // outdata = (port->outdata_ctrl&~port->reset_mask_ctrl)^port->reset_val_ctrl;

    // if(outdata != port->outdata_ctrl) {
	// /* correct for hardware inverters on pins 1, 14, & 17 */
	// outdata ^= 0x0B;
        // deadline = port->write_time_ctrl + reset_time_tsc;
        // while(rtapi_get_clocks() < deadline) {}
        // rtapi_outb(outdata, port->base_addr + 2);
    // }
}

static void write_pin(void *arg,long period)
{
	gpiod_line_set_value(((gpio_t*)arg)->line,*(((gpio_t*)arg)->data));	
    // rtapi_print_msg(RTAPI_MSG_INFO,
	    // "hal_pi_gpio: write_pin\n",);
}

void read_all(void *arg,long period)
{
	for(int i=0;((gpio_t**)arg)[i]!=NULL;i++)
	{
		read_pin((void*)(((gpio_t**)arg)[i]),period);
	}
    rtapi_print_msg(RTAPI_MSG_INFO,
	    "hal_pi_gpio: read_all\n");
}

void write_all(void *arg, long period)
{
	for(int i=0;((gpio_t**)arg)[i]!=NULL;i++)
	{
		write_pin((void*)(((gpio_t**)arg)[i]),period);
	}
    rtapi_print_msg(RTAPI_MSG_INFO,
	    "hal_pi_gpio: write_all\n");
}

static int export_input_pin(int pin, hal_bit_t ** base)
{
    int retval;

    /* export write only HAL pin for the input bit */
    retval = hal_pin_bit_newf(HAL_OUT, base, comp_id,
            "hal_pi_gpio.pin-%02d-in", pin);
    if (retval != 0) {
	return retval;
    }
    /* export another write only HAL pin for the same bit inverted */
    retval = hal_pin_bit_newf(HAL_OUT, base + 1, comp_id,
            "hal_pi_gpio.pin-%02d-in-not", pin);
    return retval;
}

static int export_output_pin(int pin, hal_bit_t ** dbase,
    hal_bit_t * pbase, hal_bit_t * rbase)
{
    int retval;

    /* export read only HAL pin for output data */
    retval = hal_pin_bit_newf(HAL_IN, dbase, comp_id,
            "hal_pi_gpio.pin-%02d-out", pin);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for polarity */
    retval = hal_param_bit_newf(HAL_RW, pbase, comp_id,
            "hal_pi_gpio.pin-%02d-out-invert", pin);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for reset */
    if (rbase)
	retval = hal_param_bit_newf(HAL_RW, rbase, comp_id,
		"hal_pi_gpio.pin-%02d-out-reset", pin);
    return retval;
}
