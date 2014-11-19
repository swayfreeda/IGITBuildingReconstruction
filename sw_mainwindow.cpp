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
    m_floorplanRec_ = new SW::FloorPlanDialog(this, &m_pc_, &m_mesh_, &m_plane3Ds_);
    m_floorplanRec_->setCurrentPlane3DPtr(&m_current_plane3D_);


    QActionGroup *displayActions = new QActionGroup(this);
    displayActions->addAction(actionDense_Points);
    displayActions->addAction(actionVertices);
    displayActions->addAction(actionWireFrame);
    displayActions->addAction(actionFlat);
    displayActions->addAction(actionTexture);
    actionDense_Points->setChecked(true);


    //--------------------------------SET SHARED POINTER--------------------------------//
    viewer->setPointCloudPtr(&m_pc_);
    viewer->setMeshPtr(&m_mesh_);
    viewer->setPlane3DPtr(&m_plane3Ds_);
    viewer->setCurrentPlane3D(&m_current_plane3D_);
    viewer->setFloorPlanDisplay(&m_floorplanRec_->p_floorplan_displays_);



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
    // start floor plan reconstruction
    connect(actionFloorPlanReconstuction, SIGNAL(triggered(bool)), this, SLOT(floorPlanReconstruction(bool)));


    // display inconsistent region
    connect(m_floorplanRec_->checkBox_dispInconsist, SIGNAL(stateChanged(int)), viewer, SLOT(toggle_display_inconsist_pts(int)));

    // display slices
    connect(m_floorplanRec_->checkBox_dispSices, SIGNAL(stateChanged(int)), viewer, SLOT(toggle_display_slices(int)));

    //display modelling process
    connect(m_floorplanRec_->checkBox_dispProcess, SIGNAL(stateChanged(int)), viewer, SLOT(toggle_display_modelling_process(int)) );

    // display modelling results
    connect(m_floorplanRec_->checkBox_dispModel, SIGNAL(stateChanged(int)), viewer, SLOT(toggle_display_modellding_results(int)));

    // display all plane triangulations
    connect(actionAll_Planes_Triangulations, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_all_planes_trians(bool)));

    // add new plane3D
    connect(m_floorplanRec_, SIGNAL(createNewPlane(QString)), this, SLOT(addPlaneListItem(QString)));

    // set current plane3D
    connect(planeListWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(setCurrentPlane3D(QListWidgetItem*)) );

    // update GL
    connect(m_floorplanRec_->getIncinsistDetector(), SIGNAL(updateGLViewer()), viewer, SLOT(updateGLViewer()));
    connect(m_floorplanRec_->getSlicesCalculator(), SIGNAL(updateGLViewer()), viewer,  SLOT(updateGLViewer()));
    connect(m_floorplanRec_->getFloorPlanReconstructor(), SIGNAL(updateGLViewer()), viewer,SLOT(updateGLViewer()));
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
    if(m_dataIO_->loadPointsFromPLY(m_pc_))
    {
        for(uint i=0; i< m_pc_.p_points_.size(); i++)
        {
            m_pc_.p_pt_ids_.append(i);
        }

        QString outputText = QString("%1 points").arg(m_pc_.ptNum());
        statusBar()->showMessage(outputText);

        // compute the bounding box of the point cloud
        m_pc_.compute_bounding_box();

        // compute the center of the point cloud
        m_pc_.compute_center();

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
