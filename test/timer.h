#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>

typedef struct _Timer Timer;

Timer * timer_new           (void);
void    timer_destroy       (Timer *t);
void    timer_start         (Timer *t);
void    timer_stop          (Timer *t);
double  timer_get_seconds   (Timer *t);

#endif
