#include"sw_mainwindow.h"
#include"sw_plyio.h"


#include<qfiledialog.h>
#include<QListWidget>
#include<QTextStream>
#include<QGridLayout>
#include<QProgressDialog>
#include<QObject>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
SW::MainWindow::MainWindow()
{
    setupUi(this);




    //----------------------SIGNALS  AND SLOTS------------------------------//
    // load dense points from PLY files, a point including positions, colors, and normals
    connect(loadPointsAction, SIGNAL(triggered()), this, SLOT(loadPoints()));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
SW::MainWindow::~MainWindow()
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::MainWindow::loadPoints()
{

    m_points_.clear();
    m_colors_.clear();
    m_normals_.clear();


    QString file_name = QFileDialog::getOpenFileName(this, tr("Open PointCloud File"),".",
                                                     tr("Point Cloud files(*.ply)") );

    statusBar()->showMessage(tr("Loading Dense Points..."));

    PLYIO plyIn(file_name);

    if(plyIn.loadPointsFromPLY(m_points_, m_colors_, m_normals_))
    {


        QString outputText = QString("%1 points").arg(m_points_.size());
        statusBar()->showMessage(outputText);

        viewer->viewAll();
        viewer->updateGL();
    }
    else{

        statusBar()->showMessage("Fail to Load Points From" + file_name);
    }


    //    // enable action to show dense poitns
    //    ui->actionDense_Points->setEnabled(true);

    //    // set bounding box
    //    ui->viewer->viewAll();
    //    ui->viewer->showEntireScene();

    //    QString outputText = QString("%1 points").arg(m_dense_pts_.size());
    //    statusBar()->showMessage(outputText);

    //    // diaplay dense points
    //    emit displayDensePoints(true);
    //    ui->viewer->updateGL();

}
