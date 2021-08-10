#ifndef __uart_H
#define __uart_H

#include "exchenge.h"
#include <stdint.h>
#include "nrf_libuarte_async.h"

extern nrf_libuarte_async_t *gLibuarte0;

void Uart0_Register (void *protocol, Exchange *ex);
void Uart1_Register (void *protocol, Exchange *ex);
void
uart_clear_buf (
    nrf_libuarte_async_t *p_libuarte, nrf_libuarte_async_evt_t *p_evt);


void Uart0_CLK (void);
uint8_t Uart0_Enable (void);
uint8_t Uart0_Disable (void);
void Uart0_UpdateCfg (void);


void Uart1_CLK (void);
void Uart1_Enable (void);
void Uart1_Disable (void);

#endif