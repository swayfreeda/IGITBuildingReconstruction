// Description: This is a softeare develpoed for regular building reconstruction. The input are images and dense points and the output is a
//               textured mesh
// Author:   sway
// Time:  11/05/2014
// Platform:  opencv2.4.9 + libqglviewer2.5.3 + qt4.6
// Organization: Institute of Automation, Chinese Academy of Sciences

// Note:  Most of the original codes are lost unfortunately, whatever, just make a fresh start and never complain !

#include"sw_mainwindow.h".h"
#include <qapplication.h>


using namespace SW;
int main(int argc, char**argv)
{
    QApplication app(argc,argv);

    MainWindow mainwindow;

    mainwindow.resize(400,600);
    mainwindow.setWindowState(mainwindow.windowState()^Qt::WindowMaximized);

    app.setMainWidget(&mainwindow);


    mainwindow.show();

    return app.exec();



}

