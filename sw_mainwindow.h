#ifndef SW_MAINWINDOW_H
#define SW_MAINWINDOW_H

#include"sw_dataType.h"
#include"sw_glviewer.h"
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
    // load dense points from PLY File
    void loadPoints();


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

signals:

    // emit signals to display dense points
    void displayDensePoints(bool flag);

private:

    // dense points
    QVector<Point> m_points_;

    // vertices
    QVector<Vec3> m_vertices_;

    // facets of the mesh
    QVector<QVector<int> > m_facets_;

    // texture coords of each vertex
    QVector<QVector2D> m_texture_coords_;

};

}



#endif // SW_MAINWINDOW_H


