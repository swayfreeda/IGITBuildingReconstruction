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


    QActionGroup *displayActions = new QActionGroup(this);
    displayActions->addAction(actionDense_Points);
    displayActions->addAction(actionVertices);
    displayActions->addAction(actionWireFrame);
    displayActions->addAction(actionFlat);
    displayActions->addAction(actionTexture);
    actionDense_Points->setChecked(true);


    //--------------------------------SET SHARED POINTER--------------------------------//
    viewer->setDensePointsPtr(&m_points_);
    viewer->setMeshVerticesPtr(&m_vertices_);
    viewer->setMeshFacetsPtr(&m_facets_);
    viewer->setMeshTextureCoordsPtr(&m_texture_coords_);




    //---------------------------------SIGNALS  AND SLOTS------------------------------//
    // load dense points from PLY files, a point including positions, colors, and normals
    connect(loadDataAction, SIGNAL(triggered()), this, SLOT(loadData()));
    connect(actionSave_Points,SIGNAL(triggered()), this, SLOT(savePoints()));
    // display signals and slots
    connect(actionDense_Points, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_points(bool)));
    connect(actionVertices, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_vertices(bool)));
    connect(actionWireFrame, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_wire_frame(bool)));
    connect(actionFlat, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_flat(bool)));
    connect(actionTexture, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_texture(bool)));
    connect(this, SIGNAL(displayDensePoints(bool)), viewer, SLOT(toggle_display_points(bool)));


    //--------------------------------MESSAGE ON STATUS BAR----------------------------//
    // show information from viewer
    connect(viewer, SIGNAL(statusBar(QString)), this, SLOT(viewerMessageToStatusBar(QString)));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
SW::MainWindow::~MainWindow()
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::MainWindow::loadData()
{

    // load images
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open PointCloud File"),".",
                                                     tr("Point Cloud files(*.ply)") );

    statusBar()->showMessage(tr("Loading Dense Points..."));

    PLYIO plyIn(file_name);

    if(plyIn.loadPointsFromPLY(m_points_))
    {
        QString outputText = QString("%1 points").arg(m_points_.size());
        statusBar()->showMessage(outputText);

        actionDense_Points->setEnabled(true);
        emit displayDensePoints(true);
        viewer->updateGL();
    }
    else{

        statusBar()->showMessage("Fail to Load Points From" + file_name);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::MainWindow::savePoints()
{
}
