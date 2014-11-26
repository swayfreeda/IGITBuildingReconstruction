// Description: a widget is defined for  the diplaying the selected image and can be used to project triangulations
// and other shapes on the image
// Time: 11/07/2014
// Author: Sway
// Organization: Institute of Automation, Chinese Academy of Sciences


#ifndef PAINTIMAGEWIDGET_H
#define PAINTIMAGEWIDGET_H

#include"sw_dataType.h"

#include<QList>
#include <QWidget>
#include <QImage>
#include <QLabel>

#include <QPainter>
#include <QListWidget>
#include <QListWidgetItem>

#include<QKeyEvent>
#include<QMessageBox>

#include"opencv2/opencv.hpp"


namespace SW{
class PaintImageWidget: public QWidget
{
    Q_OBJECT

public:

    PaintImageWidget(QWidget *parent = 0);

    virtual void paintEvent(QPaintEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);


    void  makePolygons();

    /////////////////////////////////////////INTERFACE///////////////////////////////////////////////////
    void setImagesPtr(QMap<QString, cv::Mat_<cv::Vec3b> > * ptr){p_images_ = ptr;}
    void setCamerasPtr(QMap<QString, Camera > * ptr){p_cameras_ = ptr;}
    void setPlane3DsPtr(QMap<QString, Plane3D> * ptr){ p_plane3Ds_ = ptr ;}

public slots:

    // ----------------------------------------CURRENTIMGENAME------------------------------------------//
    // set current image and starting painting image
    void startPaintingImg(QListWidgetItem* item);




    //-----------------------------------------STARTPATINTINGPROJECTIONG--------------------------------//
    void startPainitingProjecting(bool flag);



    //-----------------------------------------SETTINGCURRENTPLANE---------------------------------------//
    void setCurrentPlane(QListWidgetItem* item)
    {
        p_current_plane_ =(*p_plane3Ds_)[item->text()];
        if(p_start_projection_mode_ == true)
        {
            //-------------------------------------------//
            p_seleceted_trians_.clear();
            for(int i=0; i< p_current_plane_.p_facets_.size(); i++)
                p_seleceted_trians_.append(i);

            p_polygons_.clear();
            makePolygons();
        }
        update();
    }

public:



private:


    //--------------------------------------------variables for drawing-------------------------------//
    bool p_start_draw_img_;
    bool p_start_projection_mode_;


    // -------------------------------------------variables for images--------------------------------//
    // shared pointer of images
    QMap<QString, cv::Mat_<cv::Vec3b> > *p_images_;



    //--------------------------------------------variables for cameras-------------------------------//
    //shared pointer of cameras
    QMap<QString, Camera> *p_cameras_;



    //--------------------------------------------variables for planes----------------------------------//
    QMap<QString, Plane3D> *p_plane3Ds_;



    //--------------------------------------------variables for current plane--------------------------//
    // shared pointer of current plane
    Plane3D  p_current_plane_;


    //----------------------------------------------current variables----------------------------------//
    // current image
    cv::Mat_<cv::Vec3b> p_current_image_;

    // current camera
    Camera p_current_camera_;

    // current image name
    QString p_current_img_name_;

    // qimage is used for painting and is usually resized from the original image
    QImage p_img_show_;




    //----------------------------------------------variables for projection---------------------------//
    // indices of selecete triangulations
    QVector<uint> p_seleceted_trians_;

    // polygons to be drawn
    QVector<QPolygon> p_polygons_;

    // scale for the original image and the image to be drawn
    float p_w_scale_;
    float p_h_scale_;



signals:

};
}

#endif // PAINTIMAGEWIDGET_H
