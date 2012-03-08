/***************************************************************
 * Name:      sil2Main.cpp
 * Purpose:   Code for Application Frame
 * Author:    Johannes ()
 * Created:   2012-01-22
 * Copyright: Johannes ()
 * License:
 **************************************************************/


#include "sil2Main.h"
#include <wx/msgdlg.h>
#include <plplot/plplotP.h>
#include <plplot/wxPLplotwindow.h>
#include <wx/wx.h>
#include "Settings.h"
#include "sil_al.hpp"
#include "motor_controller.hpp"
#include "slip_controller.hpp"

//(*InternalHeaders(sil2Frame)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(sil2Frame)
const long sil2Frame::idMenuQuit = wxNewId();
const long sil2Frame::idMenuAbout = wxNewId();
const long sil2Frame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(sil2Frame,wxFrame)
    //(*EventTable(sil2Frame)
    //*)
END_EVENT_TABLE()
SilMotorControlHW *hw;
SineMotorController *mc;
SlipController *sc;
int freq;

sil2Frame::sil2Frame(wxWindow* parent,wxWindowID id)
{
   Parameters *p = new Parameters();
   Parameters *ps = new Parameters();
   Mediator<int, 10> *m = new Mediator<int, 10>();
   hw = new SilMotorControlHW(this);
   mc = new SineMotorController(hw, p);
   sc = new SlipController(hw, ps, mc, m);
   this->controller = mc;
   PLFLT xmin =-1, ymin=-10, xmax=600, ymax=3000;
   PLINT just=0, axis=0;

   //(*Initialize(sil2Frame)
   wxBoxSizer* box;
   wxMenuItem* MenuItem2;
   wxMenuItem* MenuItem1;
   wxMenu* Menu1;
   wxMenuBar* MenuBar1;
   wxMenu* Menu2;

   Create(parent, id, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
   SetClientSize(wxSize(800,600));
   box = new wxBoxSizer(wxHORIZONTAL);
   SetSizer(box);
   MenuBar1 = new wxMenuBar();
   Menu1 = new wxMenu();
   MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
   Menu1->Append(MenuItem1);
   MenuBar1->Append(Menu1, _("&File"));
   Menu2 = new wxMenu();
   MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
   Menu2->Append(MenuItem2);
   MenuBar1->Append(Menu2, _("Help"));
   SetMenuBar(MenuBar1);
   StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
   int __wxStatusBarWidths_1[1] = { -1 };
   int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
   StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
   StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
   SetStatusBar(StatusBar1);
   box->SetSizeHints(this);
   Center();

   Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&sil2Frame::OnQuit);
   Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&sil2Frame::OnAbout);
   Connect(wxID_ANY,wxEVT_LEFT_DOWN,(wxObjectEventFunction)&sil2Frame::OnLeftDClick);
   Connect(wxID_ANY,wxEVT_LEFT_DCLICK,(wxObjectEventFunction)&sil2Frame::OnLeftDClick);
   Connect(wxID_ANY,wxEVT_MOUSEWHEEL,(wxObjectEventFunction)&sil2Frame::OnMouseWheel);
   //*)
   Connect(wxID_ANY,wxEVT_IDLE,(wxObjectEventFunction)&sil2Frame::OnIdle);

   wxPLplotwindow* plotwindow;
   //wxPanel* panel = new wxPanel( this );
	//wxBoxSizer* box = new wxBoxSizer( wxVERTICAL );
	plotwindow = new wxPLplotwindow( this );
	box->Add( plotwindow, 2, wxLEFT | wxEXPAND, 0 );
	//box->Add( settingsDlg, 2, wxALL, 0);
   //panel->SetSizer( box );
	//SetSize( 640, 500 );
	pls = plotwindow->GetStream();
   pls->env(xmin, xmax, ymin, ymax, just, axis );
    //Setup window size
    // - just=0 sets axis so they scale indepedently
    // - axis=0 draw axis box, ticks, and numeric labels
    //   see "man plenv" for details
   pls->lab( "(x)", "(y)", "PlPlot example title");

   settingsDlg = new Settings(this, 0);
	box->Add( settingsDlg, 1, wxRIGHT | wxEXPAND, 0 );

	silTimer *timer = new silTimer();
	timer->Start(50);
}

sil2Frame::~sil2Frame()
{
    //(*Destroy(sil2Frame)
    //*)
}

void sil2Frame::OnQuit(wxCommandEvent& event)
{
    Close();
}

static float frq = 218;
static int amp = 16384;
static bool sel = 0;

void sil2Frame::OnAbout(wxCommandEvent& event)
{
//    wxString msg = wxbuildinfo(long_f);
//    wxMessageBox(msg, _("Welcome to..."));
   unsigned short a = 5;
   unsigned short b = 0xfff8;
   unsigned short c = a - b;

    sel = !sel;
}

void sil2Frame::OnMouseWheel(wxMouseEvent& event)
{
}

void sil2Frame::OnIdle(wxIdleEvent &event)
{
   hw->Tick();
   event.RequestMore(true);
}

void sil2Frame::OnLeftDClick(wxMouseEvent& event)
{
   sel = !sel;
}

void silTimer::Notify()
{
   extern float load;
   static int test = 0;

   test++;

   if (10 == test)
   {
      hw->SetRevTicks(freq+60);
      test = 0;
   }
   else
   {
      hw->SetRevTicks(0.98*freq + 0.5 - load/(300.0*300.0) * (freq*freq));
   }
   sc->Tick();
}
