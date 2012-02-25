#include "terminal.hpp"
#include "params.hpp"

class TerminalCommandTest : public TerminalCommand
{
   private:
      void Exec(const char *arg, char* outbuf) const;
      const char *GetCmdString() const;
};

class TerminalCommandSet : public TerminalCommand
{
   public:
      TerminalCommandSet(Configurable *module);

   private:
      void Exec(const char *arg, char* outbuf) const;
      const char *GetCmdString() const;
      Configurable *_module;
};

class TerminalCommandGet : public TerminalCommand
{
   public:
      TerminalCommandGet(Configurable *module);

   private:
      void Exec(const char *arg, char* outbuf) const;
      const char *GetCmdString() const;
      Configurable *_module;
};
