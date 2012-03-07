#ifndef SETTINGS_H
#define SETTINGS_H

//(*Headers(Settings)
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/spinctrl.h>
#include <wx/panel.h>
#include <wx/choice.h>
//*)
#include "gui_textio.hpp"

class Settings: public wxPanel
{
	public:

		Settings(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~Settings();

		//(*Declarations(Settings)
		wxSpinCtrl* throttleSpinCtrl;
		wxStaticText* StaticText2;
		wxTextCtrl* consoleTextCtrl;
		wxStaticText* StaticText3;
		wxChoice* pwmChoice;
		wxSpinCtrl* loadSpinCtrl;
		wxSpinCtrl* frqSpin;
		wxStaticText* StaticText5;
		wxStaticText* StaticText4;
		wxSpinCtrl* ampSpin;
		//*)

	protected:

		//(*Identifiers(Settings)
		static const long ID_STATICTEXT1;
		static const long ID_SPINCTRL1;
		static const long ID_STATICTEXT2;
		static const long ID_SPINCTRL2;
		static const long ID_CHOICE1;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT4;
		static const long ID_SPINCTRL3;
		static const long ID_STATICTEXT5;
		static const long ID_SPINCTRL4;
		//*)

	private:
      GUITextIO *textIO;
		//(*Handlers(Settings)
		void OnLeftDClick(wxMouseEvent& event);
		void OnfrqSpinChange(wxSpinEvent& event);
		void OnampSpinChange(wxSpinEvent& event);
		void OnpwmChoiceSelect(wxCommandEvent& event);
		void OnconsoleTextCtrlTextEnter(wxCommandEvent& event);
		void OnthrottleSpinCtrlChange(wxSpinEvent& event);
		void OnloadSpinCtrlChange(wxSpinEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
