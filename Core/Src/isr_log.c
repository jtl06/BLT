// isr_log.h
// logs time between successive isr entry for interrupt 
// find period and jitter

// includes
#include "isr_log.h"

// vars
static volatile uint32_t log_buf[ISR_LOG_LEN];
static volatile uint16_t head = 0; // next write position
static volatile uint16_t filled = 0; // valid sample count

// fx
void isr_log_add(uint32_t t_us) {
    log_buf[head] = t_us;             // timestamp
    head = (uint16_t)((head + 1) % ISR_LOG_LEN);  // wrap index
    if (filled < ISR_LOG_LEN) {
        filled++;                     // once full, stays at ISR_LOG_LEN
    }
}

// get count
uint16_t isr_log_count(void) {
    return filled;
}

// read entry by index in arrival order.
uint32_t isr_log_get(uint16_t i) {
    // Compute chronological base so index 0 is the oldest valid entry
    uint16_t base = (uint16_t)((head + ISR_LOG_LEN - filled) % ISR_LOG_LEN);
    uint16_t pos  = (uint16_t)((base + i) % ISR_LOG_LEN);
    return log_buf[pos];
}

// reset
void isr_log_reset(void) {
    head = 0;
    filled = 0;
}