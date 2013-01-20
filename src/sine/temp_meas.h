#ifndef TEMP_MEAS_H_INCLUDED
#define TEMP_MEAS_H_INCLUDED

#include "my_fp.h"

#ifdef __cplusplus
extern "C"
{
#endif

s32fp temp_JCurve(int digit);
s32fp temp_KTY83(int digit);

#ifdef __cplusplus
}
#endif

#ifdef __TEMP_LU_TABLES
#define JCURVE \
51	,\
68	,\
90	,\
118	,\
154	,\
198	,\
252	,\
319	,\
398	,\
493	,\
603	,\
732	,\
879	,\
1044	,\
1228	,\
1428	,\
1645	,\
1875	,\
2115	,\
2363	,\
2614	,\
2865	,\
3112	,\
3353	,\
3586	,\
3807	,\
4017

#define KTY83 \
1998	,\
1902	,\
1809	,\
1720	,\
1633	,\
1552	,\
1474	,\
1401	,\
1331	,\
1266	,\
1203	,\
1145	,\
1090	,\
1039	,\
990	,\
944	,\
901	,\
860	,\
822	,\
786	,\
752	,\
720	,\
689	,\

#endif

#endif // TEMP_MEAS_H_INCLUDED
