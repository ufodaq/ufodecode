#include <stdlib.h>
#include "timer.h"


Timer *
timer_new (void)
{
    Timer *t = (Timer *) malloc (sizeof (Timer));
    t->seconds = t->useconds = 0L;
    return t;
}

void
timer_destroy (Timer *t)
{
    free (t);
}

void
timer_start (Timer *t)
{
    gettimeofday(&t->start, NULL);
}

void
timer_stop (Timer *t)
{
    struct timeval end;

    gettimeofday(&end, NULL);
    t->seconds += end.tv_sec - t->start.tv_sec;
    t->useconds += end.tv_usec - t->start.tv_usec;
}
