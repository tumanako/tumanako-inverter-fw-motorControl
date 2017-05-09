#ifndef MY_MATH_H_INCLUDED
#define MY_MATH_H_INCLUDED

#define ABS(a)   ((a) < 0?(-a) : (a))
#define MIN(a,b) ((a) < (b)?(a):(b))
#define MAX(a,b) ((a) > (b)?(a):(b))
#define IIRFILTER(l,n,c) (((n) + ((l) << (c)) - (l)) >> (c))
#define MEDIAN3(a,b,c)  ((a) > (b) ? ((b) > (c) ? (b) : ((a) > (c) ? (c) : (a))) \
                                   : ((a) > (c) ? (a) : ((b) > (c) ? (c) : (b))))

#endif // MY_MATH_H_INCLUDED
