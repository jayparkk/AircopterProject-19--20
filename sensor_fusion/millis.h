#ifndef __MILLIS_H
#define __MILLIS_H

/**
 * Starts the clock. Call this function
 * at the beginning of main.
 */
void millis_begin(void);

/**
 * Returns the number of milliseconds
 * that have passed since the call
 * to millis_begin.
 */
unsigned long millis(void);
 
#endif
