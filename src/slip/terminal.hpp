#ifndef TERMINAL_HPP_INCLUDED
#define TERMINAL_HPP_INCLUDED

#include "textio.hpp"


class TerminalCommand
{
   public:
      virtual void Exec(const char*, char*) const = 0;
      virtual const char *GetCmdString() const = 0;
};


class Terminal : TextIO::NewCharHandler
{
   public:
      Terminal(const TerminalCommand**, TextIO*);

   private:
      const TerminalCommand **_commands;
      TextIO *textIO;

      static const int InBufSize = 64;
      char InBuf[InBufSize];
      char *pInBufCur;

      const TerminalCommand *pCurCmd;
      char *pArg;
      const TerminalCommand *pLastCmd;
      char *pArgLast;


      void ExecuteCommand(const TerminalCommand *pCmd, char* pArg);
      void AppendToInBuf(char TypedCharacter);
      void ParseCommand();
      const TerminalCommand *LookupCommand();
      void FlushCommand();
      void SetLastCommand();
      void TermWriteString(const char*);

      //Implement TextIO::NewCharHandler interface
      void NewChar(char c);
};

#endif // TERMINAL_HPP_INCLUDED
