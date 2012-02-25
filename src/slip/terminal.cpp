extern "C"
{
#include "my_string.h"
}
#include "terminal.hpp"

Terminal::Terminal(const TerminalCommand *commands[], TextIO *textIO) :
   _commands(commands), textIO(textIO)
{
    this->pInBufCur = this->InBuf;
    this->pCurCmd = 0;
    this->pArg = 0;
    textIO->SetCharHandler(this);
}

void Terminal::NewChar(char TypedCharacter)
{
   if (textIO->WantEcho())
   {
      textIO->SetChar(TypedCharacter);
   }
   if (textIO->CommitChar() == TypedCharacter)
   {
      ExecuteCommand(pCurCmd, pArg);
      SetLastCommand();
      FlushCommand();
   }
   else if ('!' == TypedCharacter)
   {
      ExecuteCommand(pLastCmd, pArgLast);
   }
   else
   {
      AppendToInBuf(TypedCharacter);
      ParseCommand();
   }
}

void Terminal::ExecuteCommand(const TerminalCommand *pCmd, char* pArg)
{
   char result[256];

   if (NULL != pCmd)
   {
      pCmd->Exec(pArg, result);
      TermWriteString(result);
   }
   else
   {
      TermWriteString("Unknown command sequence\n");
   }
   pInBufCur = InBuf;
}

void Terminal::AppendToInBuf(char TypedCharacter)
{
   *pInBufCur = TypedCharacter;
   if (pInBufCur < &InBuf[InBufSize - 1])
   {
      pInBufCur++;
   }
   *pInBufCur = 0;
}

void Terminal::ParseCommand()
{
   if (NULL == pArg)
   {
      pCurCmd = LookupCommand();
      if (NULL != pCurCmd)
      {
         pArg = pInBufCur;
      }
   }
}

const TerminalCommand *Terminal::LookupCommand()
{
   const TerminalCommand **pCmd = this->_commands;

   for (; NULL != *pCmd; pCmd++)
   {
      const char *cmdString = (*pCmd)->GetCmdString();
      if (0 == my_strcmp(InBuf, cmdString))
      {
         break;
      }
   }
   return *pCmd;
}

void Terminal::SetLastCommand()
{
   pArgLast = pArg;
   pLastCmd = pCurCmd;
}

void Terminal::FlushCommand()
{
   pArg = NULL;
   pCurCmd = NULL;
}

void Terminal::TermWriteString(const char *pString)
{
   for (;*pString > 0; pString++)
   {
      textIO->SetChar(*pString);
   }
}
