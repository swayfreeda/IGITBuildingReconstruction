#include"sw_mainwindow.h"
#include"sw_dataIO.h"
#include"sw_functions.h"


#include<qfiledialog.h>
#include<QListWidget>
#include<QTextStream>
#include<QGridLayout>
#include<QProgressDialog>
#include<QObject>

//-----------------------------------------------------constructor---------------------------------------------//
SW::MainWindow::MainWindow()
{
    setupUi(this);

    m_floorpan_reconstruction_ = false;

    // alloc for the pointers
    m_dataIO_ = new SW::DATAIO();
    m_floorplanRec_ = new SW::FloorPlanDialog(this, &m_pc_, &m_mesh_, &m_plane3Ds_, &m_images_, &m_cameras_);


    QActionGroup *displayActions = new QActionGroup(this);
    displayActions->addAction(actionDense_Points);
    displayActions->addAction(actionVertices);
    displayActions->addAction(actionWireFrame);
    displayActions->addAction(actionFlat);
    displayActions->addAction(actionTexture);
    actionDense_Points->setChecked(true);


    //--------------------------------SET SHARED POINTER--------------------------------//
    viewer->setPointCloudPtr(&m_pc_);
    viewer->setCamerasPtr(&m_cameras_);
    viewer->setMeshPtr(&m_mesh_);
    viewer->setPlane3DPtr(&m_plane3Ds_);
    viewer->setFloorPlanDisplay(&m_floorplanRec_->p_floorplan_displays_);


    paintImageWidget->setImagesPtr(&m_images_);
    paintImageWidget->setCamerasPtr(&m_cameras_);
    paintImageWidget->setPlane3DsPtr(&m_plane3Ds_);



    //---------------------------------SIGNALS  AND SLOTS------------------------------//
    // load dense points from PLY files, a point including positions, colors, and normals
    connect(loadDataAction, SIGNAL(triggered()), this, SLOT(loadData()), Qt::QueuedConnection);
    m_dataIO_->moveToThread(&m_thread_);
    m_thread_.start();

    // load mesh from OFF file
    connect(loadMeshAction, SIGNAL(triggered()), this, SLOT(loadMesh()), Qt::QueuedConnection);
    m_dataIO_->moveToThread(&m_thread_);
    m_thread_.start();

    // save mesh to OFF file
    connect(saveMeshAction, SIGNAL(triggered()), this, SLOT(saveMesh()), Qt::QueuedConnection);
    m_dataIO_->moveToThread(&m_thread_);
    m_thread_.start();

    // start to display mesh
    connect(this, SIGNAL(startDisplayingMesh(int)), viewer, SLOT(toggle_display_modellding_results(int)));


    // save points
    connect(savePointsAction, SIGNAL(triggered()), this, SLOT(savePoints()));

    //------------------------------- display signals and slots----------------------------------------------//
    connect(actionDense_Points, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_points(bool)));
    connect(actionVertices, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_vertices(bool)));
    connect(actionWireFrame, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_wire_frame(bool)));
    connect(actionFlat, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_flat(bool)));
    connect(actionTexture, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_texture(bool)));
    connect(this, SIGNAL(displayDensePoints(bool)), viewer, SLOT(toggle_display_points(bool)));

    // display cameras
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

    // display single plane triangulations
    connect(actionSingle_Plane_Triangulation, SIGNAL(triggered(bool)), viewer, SLOT(toggle_display_single_plane_trians(bool)));

    // add new plane3D to planeListWidget
    connect(m_floorplanRec_, SIGNAL(createNewPlane(QString)), this, SLOT(addPlaneListItem(QString)));

    // set current plane3D in mainwindow
    connect(planeListWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(setCurrentPlane3D(QListWidgetItem*)) );

    //set current plane3D in painImageWidget
    connect(planeListWidget, SIGNAL(itemClicked(QListWidgetItem*)), paintImageWidget, SLOT(setCurrentPlane(QListWidgetItem*)));



    // set current image in painImageWidget
    connect(imageListWidget, SIGNAL(itemClicked(QListWidgetItem*)), paintImageWidget, SLOT(startPaintingImg(QListWidgetItem*)) );
    connect(imageListWidget, SIGNAL(itemActivated(QListWidgetItem*)), paintImageWidget, SLOT(startPaintingImg(QListWidgetItem*)) );

    // set current image in floorreconstruction
    connect(imageListWidget, SIGNAL(itemClicked(QListWidgetItem*)), m_floorplanRec_, SLOT(setCurrentImage(QListWidgetItem*)));
    connect(imageListWidget, SIGNAL(itemActivated(QListWidgetItem*)), m_floorplanRec_, SLOT(setCurrentImage(QListWidgetItem*)));

    // projection mesh onto image
    connect(actionProjection, SIGNAL(triggered(bool)), paintImageWidget, SLOT(startPainitingProjecting(bool)));

    // update GL
    connect(m_floorplanRec_->getIncinsistDetector(), SIGNAL(updateGLViewer()), viewer, SLOT(updateGLViewer()));
    connect(m_floorplanRec_->getSlicesCalculator(), SIGNAL(updateGLViewer()), viewer,  SLOT(updateGLViewer()));
    connect(m_floorplanRec_->getFloorPlanReconstructor(), SIGNAL(updateGLViewer()), viewer,SLOT(updateGLViewer()));
    connect(m_floorplanRec_, SIGNAL(updateGLViewer()), viewer, SLOT(updateGLViewer()));

    // start display back projected quads
    connect(m_floorplanRec_, SIGNAL(startDisplayBackProjQuads()), viewer, SLOT(startDisplayBackPorjQuads()));

    // end display back projected quads
    connect(m_floorplanRec_, SIGNAL(endDisplayBackProjQuads()), viewer, SLOT(endDisplayBackProjQuads()));

    // start display added window planes in viewer
    connect(m_floorplanRec_, SIGNAL(startDisplayAddedWindowPlanes()), viewer, SLOT(startDisplayAddedWindowPlanes()));

    // end display added windwo planes in viewer
    connect(m_floorplanRec_, SIGNAL(endDisplayAddedWindowPlanes()), viewer, SLOT(endDisplayAddedWindowPlanes()));

    // start display modelling results
    connect(m_floorplanRec_, SIGNAL(startDisplayModellingResults(int)), viewer, SLOT(toggle_display_modellding_results(int)));

    // end display sigle plane trians
    connect(m_floorplanRec_, SIGNAL(endDisplaySinglePlaneTrians(bool)), viewer, SLOT(toggle_display_single_plane_trians(bool)));
}


//-----------------------------------------------------desctructor---------------------------------------------//
SW::MainWindow::~MainWindow()
{

}
//-----------------------------------------------------load data-----------------------------------------------//
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
        QImage img_qt;
        foreach(QString key, m_images_.keys())
        {
            foreach(cv::Mat_<cv::Vec3b> img_cv, m_images_.values(key))
            {
                //cout<<"height* width: "<< img_cv.rows<<" * "<<img_cv.cols<<endl;
                img_qt = convertToQImage(img_cv);

                int height_img = img_qt.height();
                int width_img = img_qt.width();

                // size of the item
                int height_icon = imageListWidget->height() - 25;
                int width_icon = (float)width_img / (float)height_img *height_icon;

                imageListWidget->setIconSize(QSize(width_icon, height_icon));

                QListWidgetItem * pItem = new QListWidgetItem(QIcon(QPixmap::fromImage(img_qt)), key);
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



//-----------------------------------------------------load Mesh-----------------------------------------------//
void SW::MainWindow::loadMesh()
{
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open Model File"),".",
                                                     tr("Model files(*.off)") );
    if(!file_name.isEmpty())
    {
        m_mesh_.clear();

        if( m_dataIO_->loadModelFromOFF(m_mesh_, file_name))
        {
            statusBar()->message("OFF File Loaded", 2000);

            viewer->viewAll();

            // state == 2 means true
            emit startDisplayingMesh(2);

            // acitive the floor plan reconstruction
            actionFloorPlanReconstuction->setEnabled(true);

            viewer->updateGL();
        }
    }
    else
    {
        statusBar()->message("OFF File Open Failed", 0);
    }
}


//------------------------------------------------------save mesh-----------------------------------------------//
void SW::MainWindow::saveMesh()
{
    QString file_name = QFileDialog::getSaveFileName(this, tr("Save OFF File"),
                                                     ".", tr("OFF Files(*.off)"));

    if(!file_name.isEmpty())
    {
         m_dataIO_->saveMeshToOFF(m_mesh_, file_name);
    }


}


//-------------------------------------------------------save points--------------------------------------------//
void SW::MainWindow::savePoints()
{
}
