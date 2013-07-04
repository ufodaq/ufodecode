#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>

typedef struct {
    struct timeval  start;
    long            seconds;
    long            useconds;
} Timer;

Timer * timer_new       (void);
void    timer_destroy   (Timer *t);
void    timer_start     (Timer *t);
void    timer_stop      (Timer *t);

#endif
