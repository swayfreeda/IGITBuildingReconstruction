#include"sw_mainwindow.h"
#include"sw_dataIO.h"


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

    m_floorpan_reconstruction_ = false;

    // alloc for the pointers
     m_dataIO_ = new SW::DATAIO();
    m_floorplanRec_ = new SW::FloorPlanDialog(this);


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
    viewer->setCamerasPtr(&m_cameras_);



    //---------------------------------SIGNALS  AND SLOTS------------------------------//
    // load dense points from PLY files, a point including positions, colors, and normals
    connect(loadDataAction, SIGNAL(triggered()), this, SLOT(loadData()), Qt::QueuedConnection);
    m_dataIO_->moveToThread(&m_thread_);
    m_thread_.start();


    connect(actionSave_Points,SIGNAL(triggered()), this, SLOT(savePoints()));
    // display signals and slots
    connect(actionDense_Points, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_points(bool)));
    connect(actionVertices, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_vertices(bool)));
    connect(actionWireFrame, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_wire_frame(bool)));
    connect(actionFlat, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_flat(bool)));
    connect(actionTexture, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_texture(bool)));
    connect(this, SIGNAL(displayDensePoints(bool)), viewer, SLOT(toggle_display_points(bool)));
    connect(actionCameras, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_cameras(bool)));


    //--------------------------------MESSAGE ON STATUS BAR------------------------------------------------//
    // show information from viewer
    connect(viewer, SIGNAL(statusBar(QString)), this, SLOT(viewerMessageToStatusBar(QString)));


    //---------------------------------FloorPlan Reconstruction--------------------------------------------//
    connect(actionFloorPlanReconstuction, SIGNAL(triggered(bool)), this, SLOT(floorPlanReconstruction(bool)));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
SW::MainWindow::~MainWindow()
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::MainWindow::loadData()
{

    // get the folder name
    QString folder_name = QFileDialog::getExistingDirectory(this, tr("Select Folder") );

    statusBar()->showMessage(tr("Loading Data from ")+ folder_name);
    m_dataIO_->setFolderDir(folder_name);

    //------------------------- load points from ply files---------------------------------//
    if(m_dataIO_->loadPointsFromPLY(m_points_))
    {
        QString outputText = QString("%1 points").arg(m_points_.size());
        statusBar()->showMessage(outputText);

        actionDense_Points->setEnabled(true);
        emit displayDensePoints(true);
        viewer->updateGL();
    }
    else{

        statusBar()->showMessage("Fail to Load Points");
    }

    //--------------------------load Images ----------------------------------------------//
    if(m_dataIO_->loadImages(m_images_))
    {
        // set the properties of the imageListWidget
        imageListWidget->setResizeMode(QListView::Adjust);
        imageListWidget->setViewMode(QListView::IconMode);
        imageListWidget->setMovement(QListView::Static);
        imageListWidget->setSpacing(5);

        int index =0;
        foreach(QString key, m_images_.keys())
        {
            foreach(QImage img, m_images_.values(key))
            {
                int height_img = img.height();
                int width_img = img.width();

                // size of the item
                int height_icon = imageListWidget->height() - 25;
                int width_icon = (float)width_img / (float)height_img *height_icon;

                imageListWidget->setIconSize(QSize(width_icon, height_icon));

                QListWidgetItem * pItem = new QListWidgetItem(QIcon(QPixmap::fromImage(img)), key);
                pItem->setSizeHint(QSize(width_icon, height_icon + 20));// +15 make the txt appear
                imageListWidget->insertItem(index, pItem);
                index++;
            }
        }
    }
    else{

        statusBar()->showMessage("Fail to Load Images");
    }

    //--------------------------load cameras----------------------------------------------//
    if(m_dataIO_->loadCameras(m_cameras_))
    {
        actionCameras->setEnabled(true);
        viewer->updateGL();
    }
    else{

        statusBar()->showMessage("Fail to Load Cameras");
    }
#if 0
    //--------------------------load visibilities-----------------------------------------//
    if(m_dataIO_->loadVisiblities(m_points_))
    {
    }
    else{
        statusBar()->showMessage("Fail to Load Visibilities");
    }
#endif

    // acitive the floor plan reconstruction
    actionFloorPlanReconstuction->setEnabled(true);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::MainWindow::savePoints()
{
}
