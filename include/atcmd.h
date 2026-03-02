/**
 * AT command handling for MT7628 daemon.
 * Registers the command parser on a UART instance.
 */
#ifndef ATCMD_H
#define ATCMD_H

#include "uart.h"

void atcmd_init(uart_inst_t *uart);

/** Call from main loop to emit WiFi async events (APJOIN, APLEAVE, STACONN, STADISCONN). */
void wifi_events_poll(uart_inst_t *uart);

#endif /* ATCMD_H */

