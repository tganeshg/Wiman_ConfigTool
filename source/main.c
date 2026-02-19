/*****************************************************************
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
*****************************************************************/

/*** Includes ***/
#include "general.h"

/*** Globals ***/
unsigned long long flag1;

/****************************************************************
* Function Definition
****************************************************************/


/****************************************************************
* Main
****************************************************************/
int main(int argc, char **argv, char **envp)
{
	UART_INST *uart;
    ssize_t len;
    unsigned char *data;

	printf("Hello Ganesh\n");
    
    // Initialize UART with 1KB buffer
    uart = uart_init(1024);
    if(!uart) {
        printf("Failed to initialize UART\n");
        return RET_FAILURE;
    }
    
    // Configure UART: ttyS1, 115200, 8N1
    if(uart_configure(uart, "ttyS1", 115200, "8N1") < 0) {
        printf("Failed to configure UART\n");
        uart_destroy(uart);
        return RET_FAILURE;
    }
    
    // Open UART
    if(uart_open(uart) < 0) {
        printf("Failed to open UART\n");
        uart_destroy(uart);
        return RET_FAILURE;
    }
    
    // Send AT command
    const char *cmd = "AT\r\n";
    len = uart_write(uart, cmd, strlen(cmd));
    printf("Sent %zd bytes\n", len);
    
    // Read response
    len = uart_read_all(uart);
    if(len > 0) {
        data = uart_get_data(uart, NULL);
        printf("Received %zd bytes: %s\n", len, data);
    }
    
    // Clean up
    uart_close(uart);
    uart_destroy(uart);
	return RET_OK;
}

/* EOF */
