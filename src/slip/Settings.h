#ifndef SETTINGS_H
#define SETTINGS_H

//(*Headers(Settings)
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/spinctrl.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/button.h>
//*)
#include "gui_textio.hpp"

class Settings: public wxPanel
{
	public:

		Settings(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~Settings();

		//(*Declarations(Settings)
		wxStaticText* StaticText2;
		wxTextCtrl* consoleTextCtrl;
		wxStaticText* StaticText3;
		wxChoice* pwmChoice;
		wxSpinCtrl* frqSpin;
		wxButton* buttonSet;
		wxSpinCtrl* ampSpin;
		//*)

	protected:

		//(*Identifiers(Settings)
		static const long ID_BUTTON1;
		static const long ID_STATICTEXT1;
		static const long ID_SPINCTRL1;
		static const long ID_STATICTEXT2;
		static const long ID_SPINCTRL2;
		static const long ID_CHOICE1;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL1;
		//*)

	private:
      GUITextIO *textIO;
		//(*Handlers(Settings)
		void OnLeftDClick(wxMouseEvent& event);
		void OnfrqSpinChange(wxSpinEvent& event);
		void OnampSpinChange(wxSpinEvent& event);
		void OnpwmChoiceSelect(wxCommandEvent& event);
		void OnconsoleTextCtrlTextEnter(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
