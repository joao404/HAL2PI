/********************************************************************
* Description:  HAL2PI.c
*               This file, 'HAL2PI.c', is a HAL component that 
*               provides a driver for the GPIOs of a raspberry pi with the help of gpiod lib and /dev/gpiochip0.
*				Based on hal_parport.c
*
* Author: Marcel Maage
* License: GPL Version 2
*    
* Copyright (c) 2019 All rights reserved.
*
* Last change: 
********************************************************************/

/*
TODO:
-
-datentypen(vor allem fuer output) ergaenzen in gpio_t (data_inv, reset_time fuer jede Funktion?)
-write und reset function schreiben/ergaenzen
-parameter in pins_and_params testen
-invers signal in export_pin correct verschalten(derzeit fehlt der zweite Speicherplatz)
-invers outputwriting einprogrammieren (data_inv)
*/



/** This file, 'hal_parport.c', is a HAL component that provides a
    driver for the standard PC parallel port.

    It supports up to eight parallel ports, and if the port hardware
    is bidirectional, the eight data bits can be configured as inputs
    or outputs.

    The configuration is determined by command line arguments for the
    user space version of the driver, and by a config string passed
    to insmod for the realtime version.  The format is similar for
    both, and consists of a port address, followed by an optional
    direction, repeated for each port.  The direction is either "in"
    or "out" and determines the direction of the 8 bit data port.
    The default is out.  The 5 bits of the status port are always
    inputs, and the 4 bits of the control port are always outputs.
    Example command lines are as follows:

    user:        hal_parport 378 in 278
    realtime:    insmod hal_parport.o cfg="378 in 278"

    Both of these commands install the driver and configure parports
    at base addresses 0x0378 (using data port as input) and 0x0278
    (using data port as output).

    The driver creates HAL pins and parameters for each port pin
    as follows:
    Each physical output has a correspinding HAL pin, named
    'parport.<portnum>.pin-<pinnum>-out', and a HAL parameter
    'parport.<portnum>.pin-<pinnum>-out-invert'.
    Each physical input has two corresponding HAL pins, named
    'parport.<portnum>.pin-<pinnum>-in' and
    'parport.<portnum>.pin-<pinnum>-in-not'.

    <portnum> is the port number, starting from zero.  <pinnum> is
    the physical pin number on the DB-25 connector.

    The realtime version of the driver exports two HAL functions for
    each port, 'parport.<portnum>.read' and 'parport.<portnum>.write'.
    It also exports two additional functions, 'parport.read-all' and
    'parport.write-all'.  Any or all of these functions can be added
    to realtime HAL threads to update the port data periodically.

    The user space version of the driver cannot export functions,
    instead it exports parameters with the same names.  The main()
    function sits in a loop checking the parameters.  If they are
    zero, it does nothing.  If any parameter is greater than zero,
    the corresponding function runs once, then the parameter is
    reset to zero.  If any parameter is less than zero, the
    corresponding function runs on every pass through the loop.
    The driver will loop forever, until it receives either
    SIGINT (ctrl-C) or SIGTERM, at which point it cleans up and
    exits.

*/

/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to www.linuxcnc.org.
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

//#include "hal_pigpio.h"

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
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL2PI: ERROR: no config string\n");
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
	    // // "HAL2PI:");
	// rtapi_print ( "HAL2PI" );
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
    comp_id = hal_init("HAL2PI");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL2PI: ERROR: hal_init() failed\n");
	return -1;
    }
	
	
	/*connecting to hardware chip*/
	chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL2PI: ERROR: gpiod_chip_open() failed\n");
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
							"HAL2PI: ERROR: hal_malloc() failed\n");
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
					"HAL2PI: ERROR: hal_malloc() failed\n");
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
							"HAL2PI: ERROR: hal_malloc() failed\n");
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
					"HAL2PI: ERROR: hal_malloc() failed\n");
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
			"HAL2PI: outputs: %d inputs %d\n",num_outputs,num_inputs);
	
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
	//		"HAL2PI.reset-time");


	//check retval if any errors occured
	if(retval!=0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL2PI: ERROR: export_pins() failed\n");
		app_exit();
		return -1;
	}

	
	
	
	//export functions which write the data to the outputs which was set by hal
	for(i=0;i<num_inputs;i++)
	{
		/* make read function name */
		rtapi_snprintf(name, sizeof(name), "HAL2PI.%02d.read", i);
		/* export read function */
		retval = hal_export_funct(name, read_pin, &(input_ptr_array[i]),
			0, 0, comp_id);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"HAL2PI: ERROR: pin %d read funct export failed\n", i);
			app_exit();
			return -1;
		}
	}
	
	for(i=0;i<num_outputs;i++)
	{
		/* make write function name */
		rtapi_snprintf(name, sizeof(name), "HAL2PI.%02d.write", i);
		/* export write function */
		retval = hal_export_funct(name, write_pin, &(output_ptr_array[i]),
			0, 0, comp_id);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"HAL2PI: ERROR: pin %d write funct export failed\n", i);
			app_exit();
			return -1;
		}
		/* make reset function name */
		rtapi_snprintf(name, sizeof(name), "HAL2PI.%02d.reset", i);
		/* export write function */
		retval = hal_export_funct(name, reset_pin, &(output_ptr_array[i]),
			0, 0, comp_id);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
				"HAL2PI: ERROR: pin %d reset funct export failed\n", i);
			app_exit();
			return -1;
		}
	}
	
	
	
		

    /* export functions that read and write all pins */
    retval = hal_export_funct("HAL2PI.read-all", read_all,
	input_ptr_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL2PI: ERROR: read all funct export failed\n");
	app_exit();
	return -1;
    }
    retval = hal_export_funct("HAL2PI.write-all", write_all,
	output_ptr_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL2PI: ERROR: write all funct export failed\n");
	app_exit();
	return -1;
    }
	
	
	
	
		
    // rtapi_print_msg(RTAPI_MSG_INFO,
	rtapi_print(
	"HAL2PI: installed driver \n");
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
    // rtapi_print_msg(RTAPI_MSG_INFO,
	    // "HAL2PI: read_port\n");
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
	    // "HAL2PI: write_pin\n",);
}

void read_all(void *arg,long period)
{
	for(int i=0;((gpio_t**)arg)[i]!=NULL;i++)
	{
		read_pin((void*)(((gpio_t**)arg)[i]),period);
	}
    rtapi_print_msg(RTAPI_MSG_INFO,
	    "HAL2PI: read_all\n");
}

void write_all(void *arg, long period)
{
	for(int i=0;((gpio_t**)arg)[i]!=NULL;i++)
	{
		write_pin((void*)(((gpio_t**)arg)[i]),period);
	}
    rtapi_print_msg(RTAPI_MSG_INFO,
	    "HAL2PI: write_all\n");
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

// static int pins_and_params(char *argv[])
// {
    // long port_addr[MAX_PORTS];
    // int data_dir[MAX_PORTS];
    // int use_control_in[MAX_PORTS];
    // int force_epp[MAX_PORTS];
    // int n, retval;

    // /* clear port_addr and data_dir arrays */
    // for (n = 0; n < MAX_PORTS; n++) {
	// port_addr[n] = 0;
	// data_dir[n] = 0;
	// use_control_in[n] = 0;
	// force_epp[n] = 0;
    // }
    // /* parse config string, results in port_addr[] and data_dir[] arrays */
    // num_ports = 0;
    // n = 0;
    // while ((num_ports < MAX_PORTS) && (argv[n] != 0)) {
	// port_addr[num_ports] = parse_port_addr(argv[n]);
	// if (port_addr[num_ports] < 0) {
	    // rtapi_print_msg(RTAPI_MSG_ERR,
		// "PARPORT: ERROR: bad port address '%s'\n", argv[n]);
	    // return -1;
	// }
	// n++;
	// if (argv[n] != 0) {
	    // /* is the next token 'in' or 'out' ? */
	    // if ((argv[n][0] == 'i') || (argv[n][0] == 'I')) {
		// /* we aren't picky, anything starting with 'i' means 'in' ;-) 
		 // */
		// data_dir[num_ports] = 1;
                // use_control_in[num_ports] = 0;
		// n++;
	    // } else if ((argv[n][0] == 'o') || (argv[n][0] == 'O')) {
		// /* anything starting with 'o' means 'out' */
		// data_dir[num_ports] = 0;
                // use_control_in[num_ports] = 0;
		// n++;
	    // } else if ((argv[n][0] == 'e') || (argv[n][0] == 'E')) {
		// /* anything starting with 'e' means 'epp', which is just
                   // like 'out' but with EPP mode requested, primarily for
                   // the G540 with its charge pump missing-pullup drive
                   // issue */
                // data_dir[num_ports] = 0;
                // use_control_in[num_ports] = 0;
                // force_epp[num_ports] = 1;
		// n++;
	    // } else if ((argv[n][0] == 'x') || (argv[n][0] == 'X')) {
                // /* experimental: some parports support a bidirectional
                 // * control port.  Enable this with pins 2-9 in output mode, 
                 // * which gives a very nice 8 outs and 9 ins. */
                // data_dir[num_ports] = 0;
                // use_control_in[num_ports] = 1;
		// n++;
            // }
	// }
	// num_ports++;
    // }
    // /* OK, now we've parsed everything */
    // if (num_ports == 0) {
	// rtapi_print_msg(RTAPI_MSG_ERR,
	    // "PARPORT: ERROR: no ports configured\n");
	// return -1;
    // }
    // /* have good config info, connect to the HAL */
    // comp_id = hal_init("hal_parport");
    // if (comp_id < 0) {
	// rtapi_print_msg(RTAPI_MSG_ERR, "PARPORT: ERROR: hal_init() failed\n");
	// return -1;
    // }
    // /* allocate shared memory for parport data */
    // port_data_array = hal_malloc(num_ports * sizeof(parport_t));
    // if (port_data_array == 0) {
	// rtapi_print_msg(RTAPI_MSG_ERR,
	    // "PARPORT: ERROR: hal_malloc() failed\n");
	// hal_exit(comp_id);
	// return -1;
    // }
    // /* export all the pins and params for each port */
    // for (n = 0; n < num_ports; n++) {
        // int modes = 0;

        // if(use_control_in[n]) {
            // modes = PARPORT_MODE_TRISTATE;
        // } else if(force_epp[n]) {
            // modes = PARPORT_MODE_EPP;
        // }

        // retval = hal_parport_get(comp_id, &port_data_array[n].portdata,
                // port_addr[n], -1, modes);

        // if(retval < 0) {
            // // failure message already printed by hal_parport_get
	    // hal_exit(comp_id);
            // return retval;
        // }

	// /* config addr and direction */
	// port_data_array[n].base_addr = port_data_array[n].portdata.base;
	// port_data_array[n].data_dir = data_dir[n];
	// port_data_array[n].use_control_in = use_control_in[n];

        // if(force_epp[n] && port_data_array[n].portdata.base_hi) {
            // /* select EPP mode in ECR */
            // outb(0x94, port_data_array[n].portdata.base_hi + 2);
        // }

	// /* set data port (pins 2-9) direction to "in" if needed */
	// if (data_dir[n]) {
	    // rtapi_outb(rtapi_inb(port_data_array[n].base_addr+2) | 0x20, port_data_array[n].base_addr+2);
	// }

	// /* export all vars */
	// retval = export_port(n, &(port_data_array[n]));
	// if (retval != 0) {
	    // rtapi_print_msg(RTAPI_MSG_ERR,
		// "PARPORT: ERROR: port %d var export failed\n", n);
	    // hal_exit(comp_id);
	    // return retval;
	// }
    // }
	
	
	
    // return 0;
// }

// static int export_port(int portnum, parport_t * port)
// {
    // int retval, msg;

    // /* This function exports a lot of stuff, which results in a lot of
       // logging if msg_level is at INFO or ALL. So we save the current value
       // of msg_level and restore it later.  If you actually need to log this
       // function's actions, change the second line below */
    // msg = rtapi_get_msg_level();
    // rtapi_set_msg_level(RTAPI_MSG_WARN);

    // retval = 0;
    // /* declare input pins (status port) */
    // retval += export_input_pin(portnum, 15, port->status_in, 0);
    // retval += export_input_pin(portnum, 13, port->status_in, 1);
    // retval += export_input_pin(portnum, 12, port->status_in, 2);
    // retval += export_input_pin(portnum, 10, port->status_in, 3);
    // retval += export_input_pin(portnum, 11, port->status_in, 4);
    // if (port->data_dir != 0) {
	// /* declare input pins (data port) */
	// retval += export_input_pin(portnum, 2, port->data_in, 0);
	// retval += export_input_pin(portnum, 3, port->data_in, 1);
	// retval += export_input_pin(portnum, 4, port->data_in, 2);
	// retval += export_input_pin(portnum, 5, port->data_in, 3);
	// retval += export_input_pin(portnum, 6, port->data_in, 4);
	// retval += export_input_pin(portnum, 7, port->data_in, 5);
	// retval += export_input_pin(portnum, 8, port->data_in, 6);
	// retval += export_input_pin(portnum, 9, port->data_in, 7);
    // } else {
	// /* declare output pins (data port) */
	// retval += export_output_pin(portnum, 2,
	    // port->data_out, port->data_inv, port->data_reset, 0);
	// retval += export_output_pin(portnum, 3,
	    // port->data_out, port->data_inv, port->data_reset, 1);
	// retval += export_output_pin(portnum, 4,
	    // port->data_out, port->data_inv, port->data_reset, 2);
	// retval += export_output_pin(portnum, 5,
	    // port->data_out, port->data_inv, port->data_reset, 3);
	// retval += export_output_pin(portnum, 6,
	    // port->data_out, port->data_inv, port->data_reset, 4);
	// retval += export_output_pin(portnum, 7,
	    // port->data_out, port->data_inv, port->data_reset, 5);
	// retval += export_output_pin(portnum, 8,
	    // port->data_out, port->data_inv, port->data_reset, 6);
	// retval += export_output_pin(portnum, 9,
	    // port->data_out, port->data_inv, port->data_reset, 7);
	// retval += hal_param_u32_newf(HAL_RW, &port->reset_time, comp_id, 
			// "parport.%d.reset-time", portnum);
	// retval += hal_param_u32_newf(HAL_RW, &port->debug1, comp_id, 
			// "parport.%d.debug1", portnum);
	// retval += hal_param_u32_newf(HAL_RW, &port->debug2, comp_id, 
			// "parport.%d.debug2", portnum);
	// port->write_time = 0;
    // }
    // if(port->use_control_in == 0) {
	// /* declare output variables (control port) */
	// retval += export_output_pin(portnum, 1,
	    // port->control_out, port->control_inv, port->control_reset, 0);
	// retval += export_output_pin(portnum, 14,
	    // port->control_out, port->control_inv, port->control_reset, 1);
	// retval += export_output_pin(portnum, 16,
	    // port->control_out, port->control_inv, port->control_reset, 2);
	// retval += export_output_pin(portnum, 17,
	    // port->control_out, port->control_inv, port->control_reset, 3);
    // } else {
	// /* declare input variables (control port) */
        // retval += export_input_pin(portnum, 1, port->control_in, 0);
        // retval += export_input_pin(portnum, 14, port->control_in, 1);
        // retval += export_input_pin(portnum, 16, port->control_in, 2);
        // retval += export_input_pin(portnum, 17, port->control_in, 3);
    // }

    // /* restore saved message level */
    // rtapi_set_msg_level(msg);
    // return retval;
	// rtapi_print_msg(RTAPI_MSG_INFO,
	    // "HAL2PI:export_port\n");

// }

static int export_input_pin(int pin, hal_bit_t ** base)
{
    int retval;

    /* export write only HAL pin for the input bit */
    retval = hal_pin_bit_newf(HAL_OUT, base, comp_id,
            "HAL2PI.pin-%02d-in", pin);
    if (retval != 0) {
	return retval;
    }
    /* export another write only HAL pin for the same bit inverted */
    retval = hal_pin_bit_newf(HAL_OUT, base + 1, comp_id,
            "HAL2PI.pin-%02d-in-not", pin);
    return retval;
}

static int export_output_pin(int pin, hal_bit_t ** dbase,
    hal_bit_t * pbase, hal_bit_t * rbase)
{
    int retval;

    /* export read only HAL pin for output data */
    retval = hal_pin_bit_newf(HAL_IN, dbase, comp_id,
            "HAL2PI.pin-%02d-out", pin);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for polarity */
    retval = hal_param_bit_newf(HAL_RW, pbase, comp_id,
            "HAL2PI.pin-%02d-out-invert", pin);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for reset */
    if (rbase)
	retval = hal_param_bit_newf(HAL_RW, rbase, comp_id,
		"HAL2PI.pin-%02d-out-reset", pin);
    return retval;
}
