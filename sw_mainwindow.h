#ifndef SW_MAINWINDOW_H
#define SW_MAINWINDOW_H

#include"sw_dataType.h"
#include"sw_glviewer.h"
#include"sw_dataIO.h"
#include"sw_floorplan.h"

#include"ui_mainwindow.h"

#include <QMainWindow>
#include"qlayout.h"
//#include"qpopmenu.h"
#include"qcursor.h"
#include"qevent.h"

#include<QVector>

#include<QWidget>
#include"qnamespace.h"
#include<QFrame>
#include<QSlider>
#include<QCheckBox>
#include<QAction>
#include<QActionGroup>
#include<QToolBar>
#include<QMenu>
#include<QMenuBar>
#include<QStatusBar>
#include<QMessageBox>
#include<QListWidget>
#include<QThread>


#include<opencv2/opencv.hpp>

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem; //
//class QActionGroup;
//class QToolBar;
//class QMenu;
//class QSlider;
//class QFrame;
//class QCheckBox;
class QPushButton;
class QTimer;
class QKeyEvent;
class GLViewer;

typedef PointXYZRGBNormal Point;

namespace SW
{
class MainWindow: public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:

    MainWindow();
    ~MainWindow(); // to be implemented

protected:

    virtual void keyPressEvent(QKeyEvent *){}


public slots:

    //---------------------------------------load points---------------------------//
    // load dense points from *.ply
    // load images from visualize/ folder
    // load visbility from .patch file
    // load camera infomation from txt/ folder
    void loadData();


    // * 11/25/2014 add functions
    //--------------------------------------load Mesh--------------------------------//
    // load meshes including vertices and facets from *.OFF file.
    void loadMesh();


    //---------------------------------------save Mesh-------------------------------//
    // save meshes including vertices and facets to *.OFF file.
    void saveMesh();


    //-------------------------------save points------------------------------------//
    // save dense points to PLY File
    void savePoints();


    //----------------------------------message-------------------------------------//
    // show message from viwer on status bar
    void viewerMessageToStatusBar(QString str)
    {
        statusBar()->showMessage(str);
        update();
    }


    //----------------------------------addPlaneListItem----------------------------//
    // add a plane to the plane list widgets
    inline  void addPlaneListItem(QString name)
    {
        QListWidgetItem * item = new QListWidgetItem(name);

        planeListWidget->addItem(item);

        update();
    }


    //-----------------------------------setCurrentPlane3D---------------------------//
    inline void setCurrentPlane3D(QListWidgetItem* item)
    {

        QString name(item->text());

        if(m_plane3Ds_.contains(name))
        {
            m_floorplanRec_->setCurrentPlane3DPtr(&m_plane3Ds_[name]);
            viewer->setCurrentPlane3D(&m_plane3Ds_[name]);
        }
        else
        {
            QMessageBox::warning(this,tr("Information"), tr("Plane Does not Existed!"));
        }
        update();
        viewer->updateGL();
    }


    //-----------------------------floorplan reconstruction------------------------//
    // begin floorplan reconstruction
    void floorPlanReconstruction(bool flag)
    {
        m_floorpan_reconstruction_ = flag;

        // start to floor plan reconstruction
        if(m_floorpan_reconstruction_ == true)
        {
            m_floorplanRec_->show();
        }

        // terminate floor plan reconstrution
        if(m_floorpan_reconstruction_ == false)
        {
            m_floorplanRec_->hide();
        }
        update();
    }


signals:

    // emit signals to display dense points
    void displayDensePoints(bool flag);

    // load mesh and starting displaying
    void startDisplayingMesh(int state);

private:

    bool m_floorpan_reconstruction_;


    //------------------------------------------class structures-----------------------------------//

    // for load and save points
    SW::DATAIO *m_dataIO_;

    // for floor plan reconstruction
    // Note:  if the floorplanDialog is defined in the form "SW::FloorPlanDialog m_floorplanRec_ "
    // no widgets in the floorplanDialog will be displayed.
    SW::FloorPlanDialog * m_floorplanRec_;




    //------------------------variables for dense points--------------------------------//
    // pointcloud constaines points with normals, rgbs, positions and visibilities
    // also there is a function for computing the center  of the pointcloud
    // and a function for computing the bounding box of the pointcloud
    PointCloud m_pc_;


    //-------------------------varaibles for mesh---------------------------------------//
    // g_mesh containes the vertices, facets and edges of the mesh
    // edges are calculated from vertices and facets
    Mesh m_mesh_;



    //------------------------variables for plane3D--------------------------------------//
    // g_plane3Ds_ containes all the plane structures in the scene, inlcuding:
    // *normal of the plane
    // *parameters of the plane
    // *vertices that are in the plane
    // *texture_coordinates of the verices
    // *facets of the mesh
    // *boundaries of the plane
    // *coordinates system of the plane
    // *window boundaries of the plane
    QMap<QString, Plane3D> m_plane3Ds_;

    // current plane pointer
    //Plane3D  * m_current_plane3D_;



    //-------------------------variables for images---------------------------------------//
    // images of all the scene
    QMap< QString, cv::Mat_<cv::Vec3b> > m_images_;



    //-------------------------variables for cameras--------------------------------------//
    // *projection matrixes
    // *positions
    // *directions
    // *colors
    // *focal
    QMap< QString, Camera> m_cameras_;



    // multi-thread
    QThread m_thread_;

};

}



#endif // SW_MAINWINDOW_H


