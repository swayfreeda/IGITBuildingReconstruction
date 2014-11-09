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


    //---------------------------------------save points---------------------------//
    // save dense points to PLY File
    void savePoints();


    //---------------------------------------message-------------------------------//
    // show message from viwer on status bar
    void viewerMessageToStatusBar(QString str)
    {
        statusBar()->showMessage(str);
        update();
    }


    //----------------------------------------floorplan reconstruction---------------//
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


private:

     bool m_floorpan_reconstruction_;


    //------------------------------------------class structures-----------------------------------//

    // for load and save points
    SW::DATAIO *m_dataIO_;

    // for floor plan reconstruction
    // Note:  if the floorplanDialog is defined in the form "SW::FloorPlanDialog m_floorplanRec_ "
    // no widgets in the floorplanDialog will be displayed.
    SW::FloorPlanDialog * m_floorplanRec_;



    //------------------------------------------variables------------------------------------------//

    // dense points
    QVector<Point> m_points_;

    // vertices
    QVector<Vec3> m_vertices_;

    // facets of the mesh
    QVector<QVector<int> > m_facets_;

    // texture coords of each vertex
    QVector<QVector2D> m_texture_coords_;

    // images of all the scene
    QMap< QString, QImage> m_images_;

    // projection matrixes
    QMap< QString, Camera> m_cameras_;

    // multi-thread
    QThread m_thread_;

};

}



#endif // SW_MAINWINDOW_H


