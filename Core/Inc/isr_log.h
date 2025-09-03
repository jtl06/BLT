// isr_log.h
// logs time between successive isr entry for interrupt 
// find period and jitter

// defines
#pragma once
#define ISR_LOG_LEN 1024

// includes
#include <stdint.h>

// Prototypes

void isr_log_add(uint32_t t_us);
uint16_t isr_log_count(void);
uint32_t isr_log_get(uint16_t i); 
void isr_log_reset(void);
