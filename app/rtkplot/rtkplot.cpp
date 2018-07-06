//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop
//---------------------------------------------------------------------------







































USEFORM("..\appcmn\tcpoptdlg.cpp", TcpOptDialog);
USEFORM("..\appcmn\timedlg.cpp", TimeDialog);
USEFORM("..\appcmn\refdlg.cpp", RefDialog);
USEFORM("..\appcmn\serioptdlg.cpp", SerialOptDialog);
USEFORM("..\appcmn\vieweropt.cpp", ViewerOptDialog);
USEFORM("conndlg.cpp", ConnectDialog);
USEFORM("..\appcmn\tspandlg.cpp", SpanDialog);
USEFORM("..\appcmn\viewer.cpp", TextViewer);
USEFORM("..\appcmn\confdlg.cpp", ConfDialog);
USEFORM("..\appcmn\console.cpp", Console);
USEFORM("..\appcmn\aboutdlg.cpp", AboutDialog);
USEFORM("..\appcmn\cmdoptdlg.cpp", CmdOptDialog);
USEFORM("..\appcmn\keydlg.cpp", KeyDialog);
USEFORM("..\appcmn\fileoptdlg.cpp", FileOptDialog);
USEFORM("..\appcmn\ftpoptdlg.cpp", FtpOptDialog);
USEFORM("geview.cpp", GoogleEarthView);
USEFORM("satdlg.cpp", SatDialog);
USEFORM("pntdlg.cpp", PntDialog);
USEFORM("skydlg.cpp", SkyImgDialog);
USEFORM("vmapdlg.cpp", VecMapDialog);
USEFORM("gmview.cpp", GoogleMapView);
USEFORM("mapdlg.cpp", MapAreaDialog);
USEFORM("plotmain.cpp", Plot);
USEFORM("plotopt.cpp", PlotOptDialog);
//---------------------------------------------------------------------------
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
	try
	{
		Application->Initialize();
		Application->Title = "RTKPLOT";
		Application->CreateForm(__classid(TPlot), &Plot);
		Application->CreateForm(__classid(TPlotOptDialog), &PlotOptDialog);
		Application->CreateForm(__classid(TSatDialog), &SatDialog);
		Application->CreateForm(__classid(TRefDialog), &RefDialog);
		Application->CreateForm(__classid(TAboutDialog), &AboutDialog);
		Application->CreateForm(__classid(TSpanDialog), &SpanDialog);
		Application->CreateForm(__classid(TTimeDialog), &TimeDialog);
		Application->CreateForm(__classid(TConnectDialog), &ConnectDialog);
		Application->CreateForm(__classid(TSerialOptDialog), &SerialOptDialog);
		Application->CreateForm(__classid(TTcpOptDialog), &TcpOptDialog);
		Application->CreateForm(__classid(TCmdOptDialog), &CmdOptDialog);
		Application->CreateForm(__classid(TFileOptDialog), &FileOptDialog);
		Application->CreateForm(__classid(TKeyDialog), &KeyDialog);
		Application->CreateForm(__classid(TTextViewer), &TextViewer);
		Application->CreateForm(__classid(TViewerOptDialog), &ViewerOptDialog);
		Application->CreateForm(__classid(TPntDialog), &PntDialog);
		Application->CreateForm(__classid(TConfDialog), &ConfDialog);
		Application->CreateForm(__classid(TGoogleEarthView), &GoogleEarthView);
		Application->CreateForm(__classid(TFtpOptDialog), &FtpOptDialog);
		Application->CreateForm(__classid(TConsole), &Console);
		Application->CreateForm(__classid(TGoogleMapView), &GoogleMapView);
		Application->CreateForm(__classid(TMapAreaDialog), &MapAreaDialog);
		Application->CreateForm(__classid(TSkyImgDialog), &SkyImgDialog);
		Application->CreateForm(__classid(TVecMapDialog), &VecMapDialog);
		Application->Run();
	}
	catch (Exception &exception)
	{
		Application->ShowException(&exception);
	}
	catch (...)
	{
		try
		{
			throw Exception("");
		}
		catch (Exception &exception)
		{
			Application->ShowException(&exception);
		}
	}
	return 0;
}
//---------------------------------------------------------------------------

