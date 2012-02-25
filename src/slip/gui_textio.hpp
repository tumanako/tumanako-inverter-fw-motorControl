#ifndef GUI_TEXTIO_HPP_INCLUDED
#define GUI_TEXTIO_HPP_INCLUDED
#include "textio.hpp"
#include <wx/string.h>

class GUITextIO : public TextIO
{
   public:
      GUITextIO(wxTextCtrl *console):
      _console(console)
      {
      }

      void TextEntered(char c)
      {
         this->handler->NewChar(c);
      }

   private:
      wxTextCtrl *_console;
      void SetChar(char c)
      {
         wxString str(c, (size_t)1);

         _console->AppendText(*str);
      }

      bool WantEcho() const
      {
         return false;
      }

      char CommitChar() const
      {
         return '\n';
      }
};



#endif // GUI_TEXTIO_HPP_INCLUDED
