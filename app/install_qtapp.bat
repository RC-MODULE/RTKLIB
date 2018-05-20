cd rtkconv_qt
call install.bat
cd ..
cd rtknavi_qt
call install.bat
cd ..
cd rtkplot_qt
call install.bat
cd ..
cd rtkpost_qt
call install.bat
cd ..
cd srctblbrows_qt
call install.bat
cd ..
cd strsvr_qt
call install.bat
cd ..
cd rtkget_qt
call install.bat
cd ..
cd rtklaunch_qt
call install.bat
cd ..

windeployqt.exe  ..\bin\rtkconv_qt.exe  -no-translations
windeployqt.exe  ..\bin\rtknavi_qt.exe  -no-translations
windeployqt.exe  ..\bin\rtkplot_qt.exe  -no-translations
windeployqt.exe  ..\bin\rtkpost_qt.exe  -no-translations
windeployqt.exe  ..\bin\srctblbrows_qt.exe -no-translations
windeployqt.exe  ..\bin\strsvr_qt.exe -no-translations
windeployqt.exe  ..\bin\rtkget_qt.exe -no-translations
windeployqt.exe  ..\bin\rtklaunch_qt.exe -no-translations