/***************************************************************
 * Name:      sil2Main.h
 * Purpose:   Defines Application Frame
 * Author:    Johannes ()
 * Created:   2012-01-22
 * Copyright: Johannes ()
 * License:
 **************************************************************/

#ifndef SIL2MAIN_H
#define SIL2MAIN_H
#include "params.hpp"
#include <plplot/plplotP.h>
#include <plplot/wxPLplotwindow.h>

//(*Headers(sil2Frame)
#include <wx/sizer.h>
#include <wx/menu.h>
#include <wx/frame.h>
#include <wx/statusbr.h>
//*)
#include "Settings.h"

class sil2Frame: public wxFrame
{
    public:
      wxPLplotstream* pls;
      Settings *settingsDlg;
      Configurable *controller;

      sil2Frame(wxWindow* parent,wxWindowID id = -1);
      virtual ~sil2Frame();

    private:

        //(*Handlers(sil2Frame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnMouseWheel(wxMouseEvent& event);
        void OnIdle(wxIdleEvent& event);
        void OnLeftDClick(wxMouseEvent& event);
        //*)

        //(*Identifiers(sil2Frame)
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(sil2Frame)
        wxStatusBar* StatusBar1;
        //*)

        DECLARE_EVENT_TABLE()
};

#endif // SIL2MAIN_H
