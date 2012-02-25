#include "params.hpp"
#include "Settings.h"
#include "motor_controller.hpp"
#include "gui_textio.hpp"
#include "terminal.hpp"
#include "terminal_prj.hpp"

//(*InternalHeaders(Settings)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(Settings)
const long Settings::ID_BUTTON1 = wxNewId();
const long Settings::ID_STATICTEXT1 = wxNewId();
const long Settings::ID_SPINCTRL1 = wxNewId();
const long Settings::ID_STATICTEXT2 = wxNewId();
const long Settings::ID_SPINCTRL2 = wxNewId();
const long Settings::ID_CHOICE1 = wxNewId();
const long Settings::ID_STATICTEXT3 = wxNewId();
const long Settings::ID_TEXTCTRL1 = wxNewId();
//*)
extern SineMotorController *mc;



BEGIN_EVENT_TABLE(Settings,wxPanel)
	//(*EventTable(Settings)
	//*)
END_EVENT_TABLE()

Settings::Settings(wxWindow* parent,wxWindowID id)
{
   static const TerminalCommandSet set(mc);
   static const TerminalCommandGet get(mc);

   static const TerminalCommand *commands[] =
   {
      &set,
      &get,
      NULL
   };
	//(*Initialize(Settings)
	wxStaticText* StaticText1;

	Create(parent, id, wxDefaultPosition, wxSize(192,374), wxTAB_TRAVERSAL, _T("id"));
	buttonSet = new wxButton(this, ID_BUTTON1, _("Set"), wxPoint(0,160), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	buttonSet->SetDefault();
	StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("Frequency"), wxPoint(8,8), wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	frqSpin = new wxSpinCtrl(this, ID_SPINCTRL1, _T("100"), wxPoint(96,0), wxDefaultSize, 0, 0, 1000, 100, _T("ID_SPINCTRL1"));
	frqSpin->SetValue(_T("100"));
	StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("Amplitude"), wxPoint(8,56), wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	ampSpin = new wxSpinCtrl(this, ID_SPINCTRL2, _T("37813"), wxPoint(96,48), wxDefaultSize, 0, 0, 40000, 37813, _T("ID_SPINCTRL2"));
	ampSpin->SetValue(_T("37813"));
	pwmChoice = new wxChoice(this, ID_CHOICE1, wxPoint(96,96), wxSize(96,29), 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
	pwmChoice->SetSelection( pwmChoice->Append(_("SVPWM")) );
	pwmChoice->Append(_("SINE"));
	StaticText3 = new wxStaticText(this, ID_STATICTEXT3, _("PWM mode"), wxPoint(8,104), wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	consoleTextCtrl = new wxTextCtrl(this, ID_TEXTCTRL1, wxEmptyString, wxPoint(0,200), wxSize(264,192), wxTE_AUTO_SCROLL|wxTE_PROCESS_ENTER|wxTE_MULTILINE|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));

	Connect(ID_SPINCTRL1,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&Settings::OnfrqSpinChange);
	Connect(ID_SPINCTRL2,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&Settings::OnampSpinChange);
	Connect(ID_CHOICE1,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&Settings::OnpwmChoiceSelect);
	Connect(ID_TEXTCTRL1,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&Settings::OnconsoleTextCtrlTextEnter);
	Connect(wxID_ANY,wxEVT_LEFT_DCLICK,(wxObjectEventFunction)&Settings::OnLeftDClick);
	//*)
	textIO = new GUITextIO(consoleTextCtrl);
	Terminal *t = new Terminal(commands, textIO);
}

Settings::~Settings()
{
	//(*Destroy(Settings)
	//*)
}


void Settings::OnLeftDClick(wxMouseEvent& event)
{
}

void Settings::OnfrqSpinChange(wxSpinEvent& event)
{
   ((Configurable*)mc)->SetParameter("frqspnt", event.GetPosition());
}

void Settings::OnampSpinChange(wxSpinEvent& event)
{
   ((Configurable*)mc)->SetParameter("amp", event.GetPosition());
}

void Settings::OnpwmChoiceSelect(wxCommandEvent& event)
{
   ((Configurable*)mc)->SetParameter("pwmmod", event.GetSelection());
}


void Settings::OnconsoleTextCtrlTextEnter(wxCommandEvent& event)
{
   static bool lock = false;
   if (!lock)
   {
      lock = true;
      wxString str = event.GetString().AfterLast('\n').Append('\n');
      consoleTextCtrl->AppendText(_("\n"));
      for (wxString::iterator it = str.begin(); it != str.end(); it++)
      {
         textIO->TextEntered((char)*it);
      }
      lock = false;
   }
}
