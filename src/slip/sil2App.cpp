/***************************************************************
 * Name:      sil2App.cpp
 * Purpose:   Code for Application Class
 * Author:    Johannes ()
 * Created:   2012-01-22
 * Copyright: Johannes ()
 * License:
 **************************************************************/

#include "sil2App.h"

//(*AppHeaders
#include "sil2Main.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(sil2App);

bool sil2App::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	sil2Frame* Frame = new sil2Frame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}
