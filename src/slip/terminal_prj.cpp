#include "terminal_prj.hpp"
#include "params.hpp"
extern "C"
{
#include "my_string.h"
}

const char *TerminalCommandTest::GetCmdString() const
{
   return "test";
}

void TerminalCommandTest::Exec(const char *arg, char* outbuf) const
{
   outbuf[0] = 0;
   my_strcat(outbuf, "Hallo\n");
}

TerminalCommandSet::TerminalCommandSet(Configurable *module) :
   _module(module) {}

void TerminalCommandSet::Exec(const char *arg, char* outbuf) const
{
   char localArg[32];
   char *paramName;
   char *paramVal;
   int val;
   outbuf[0] = 0;
   localArg[0] = 0;

   my_strcat(localArg, arg);
   paramName = my_trim(localArg);
   paramVal = (char *)my_strchr(paramName, ' ');
   *paramVal = 0;
   paramVal++;
   if (*paramVal > 0)
   {
      val = my_atol(paramVal);
      if (0 == _module->SetParameter(paramName, val))
      {
         my_strcat(outbuf, "Set Ok.");
      }
      else
      {
         my_strcat(outbuf, "Set Failed.");
      }
   }
   else
   {
      my_strcat(outbuf, "No parameter value given");
   }

   my_strcat(outbuf, "\n");
}

const char *TerminalCommandSet::GetCmdString() const
{
   return "set";
}

TerminalCommandGet::TerminalCommandGet(Configurable *module) :
   _module(module) {}

void TerminalCommandGet::Exec(const char *arg, char* outbuf) const
{
   char localArg[32];
   char *paramName;
   int val;
   outbuf[0] = 0;
   localArg[0] = 0;

   my_strcat(localArg, arg);
   paramName = my_trim(localArg);
   val = _module->GetParameter(paramName);
   my_ltoa(outbuf, val, 10);

   my_strcat(outbuf, "\n");
}

const char *TerminalCommandGet::GetCmdString() const
{
   return "get";
}
