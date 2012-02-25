#ifndef PARAMS_HPP_INCLUDED
#define PARAMS_HPP_INCLUDED

class Parameters
{
   public:
      Parameters(const char *params[], int *values);
      Parameters();

      int *operator[] (const char *idx);

      void SetInfo(const char *params[], int *values);

   private:
      const char **_params;
      int *_values;
};

class Configurable
{
   public:
      int SetParameter(const char *paramName, int value)
      {
         Parameters *params = GetParameters();
         int *pval = (*params)[paramName];
         if (0 != pval)
         {
            *pval = value;
            return 0;
         }
         return -1;
      }

      int GetParameter(const char *paramName)
      {
         Parameters *params = GetParameters();
         int *pval = (*params)[paramName];
         if (0 != pval)
         {
            return *pval;
         }
         return -1;
      }

      virtual Parameters *GetParameters() = 0;
};

#endif // PARAMS_HPP_INCLUDED
