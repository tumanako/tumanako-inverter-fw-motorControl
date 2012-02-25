#ifndef TEXTIO_HPP_INCLUDED
#define TEXTIO_HPP_INCLUDED

class TextIO
{
   public:
      class NewCharHandler
      {
      public:
        virtual void NewChar(char) = 0;
      };

      virtual void SetChar(char) = 0;
      virtual bool WantEcho() const = 0;
      virtual char CommitChar() const = 0;
      void SetCharHandler(NewCharHandler *handler)
      {
         this->handler = handler;
      }

   protected:
      NewCharHandler *handler;

};


#endif // TEXTIO_HPP_INCLUDED
