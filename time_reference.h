#ifndef _TIMER_REFERENCEH
#define _TIMER_REFERENCEH
//initialice time reference. Should be called after executing other functions;
void time_reference_init(void);
uint64_t time_reference_get_current_time_us(void);
#endif