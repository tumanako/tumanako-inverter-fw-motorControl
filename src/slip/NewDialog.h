#ifndef NEWDIALOG_H
#define NEWDIALOG_H

//(*Headers(NewDialog)
#include <wx/dialog.h>
//*)

class NewDialog: public wxDialog
{
	public:

		NewDialog(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~NewDialog();

		//(*Declarations(NewDialog)
		//*)

	protected:

		//(*Identifiers(NewDialog)
		//*)

	private:

		//(*Handlers(NewDialog)
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
