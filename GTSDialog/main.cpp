// Description:  Implement an interactive method for 3D reconstruction
// 11/24/2014
// Author: Sway
// Organization: Institute of Automation, Chinese Academy of Sciences


#include"gts_repetitiveStructure.h"

#include"stdlib.h"
#include<iostream>

using namespace std;

int main(int argc, char * argv[])
{

    QApplication app(argc,argv);

    GTSDetectionDialog dialog;

    dialog.resize(600,800);

    dialog.show();

    return app.exec();
}
