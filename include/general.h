/*
*
*	Copyright (c) 2026
*	All rights reserved.
*
*	Project 		:
*	Last Updated on	:
*	Author			: Ganesh
*
*	Revision History
****************************************************************
*	Date			Version		Name		Description
****************************************************************
*	07/02/2026		1.0			Ganesh		Initial Development
*
*/
#ifndef _GENERAL_H_
#define _GENERAL_H_

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h>    /* For open, O_RDWR, O_NOCTTY, O_NDELAY */
#include <unistd.h>   /* For close, read, write */
#include <sys/select.h> /* For fd_set, FD_ZERO, FD_SET, select */
#include <sys/time.h>   /* For struct timeval */

#include "common.h"

/*
*Macros
*/
#define APP_VERSION				"CT 1.0.0 18022026"
#define DEBUG_LOG				1

//for Flags use only
extern unsigned long long flag1;

#define	POWER_ON				0

#define SET_FLAG(n)				((flag1) |= (U64)(1ULL << (n)))
#define CLR_FLAG(n)				((flag1) &= (U64)~((1ULL) << (n)))
#define CHECK_FLAG(n)			((flag1) & (U64)(1ULL<<(n)))

/*
*Structure
*/
#pragma pack(push,1)
typedef struct
{
	int				fd;                  // File descriptor
	char			device[SIZE_64];     // Device path
	speed_t			baud_rate;           // Baud rate constant
	tcflag_t		data_bits;           // CS5, CS6, CS7, CS8
	tcflag_t		parity;              // Parity flugs
	tcflag_t		stop_bits;           // CSTOPB or 0
	tcflag_t		iflags;              // Input flags
	struct			termios tios;        // Termios structure
	unsigned int	timeout_ms;          // Read timeout
	unsigned char	read_retry;          // Max read retrie
	unsigned char	*rx_buffer;          // Receive buffer
	size_t			rx_buffer_size;      // Buffer size
	size_t			rx_data_len;         // Received data length
	void			(*debug_print)(const char *fmt, ...); // Debug callback
} UART_INST;
#pragma pack(pop)

/*
*Function declarations
*/
UART_INST* uart_init(size_t rx_buffer_size);
int uart_configure(UART_INST *uart, const char *device, int baudrate, const char *mode);
int uart_open(UART_INST *uart);
ssize_t uart_write(UART_INST *uart, const void *data, size_t len);
ssize_t uart_read(UART_INST *uart, void *buffer, size_t buffer_len);
ssize_t uart_read_all(UART_INST *uart);
unsigned char* uart_get_data(UART_INST *uart, size_t *len);
int uart_flush(UART_INST *uart);
int uart_set_line(UART_INST *uart, int dtr, int rts);
int uart_close(UART_INST *uart);
void uart_destroy(UART_INST *uart);

#endif

/* EOF */
