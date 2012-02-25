#include "params.hpp"
extern "C"
{
#include "my_string.h"
}

Parameters::Parameters(const char *params[], int *values) :
       _params(params), _values(values) {}

Parameters::Parameters() {}

int *Parameters::operator[] (const char *idx)
{
   const char **pCurCmdStr = _params;
   int *pCurVal = _values;
   int *valPtr = NULL;

   while ((NULL != *pCurCmdStr) && (NULL == valPtr))
   {
      if (0 == my_strcmp(*pCurCmdStr, idx))
      {
         valPtr = pCurVal;
      }
      pCurVal++;
      pCurCmdStr++;
   }
   return valPtr;
}

void Parameters::SetInfo(const char *params[], int *values)
{
   _params = params;
   _values = values;
}
