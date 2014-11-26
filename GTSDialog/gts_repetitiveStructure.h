// Description:  This file defines a class for repetitive structure detection
// Time: 11/24/2014
// Author: Sway
// Organizaion: Institute of Automation, Chinese Academy of Sciences.

#ifndef GTS_REPETITIVESTRUCTURE_H
#define GTS_REPETITIVESTRUCTURE_H

#include"ui_RepetitiveStructure.h"
#include"gts_paintWidget.h"

#include<QWidget>
#include<QObject>

#include"opencv2/opencv.hpp"

#include<QWheelEvent>


class GTSDetectionDialog: public QWidget, public Ui::GTSDetectionDialog{

    Q_OBJECT

public:

    ////////////////////////////////////CONSTRUCTOR//////////////////////////////////////////////////////
    GTSDetectionDialog(QWidget *parent =0);



    ////////////////////////////////////GETINITIALABELS/////////////////////////////////////////////////
    // get initia labels through GMM clustering
    QVector<QVector<double> > getInitialLabels();



    ////////////////////////////////////COMPUTEENERGY////////////////////////////////////////////////////
    // compute the enegy for optimizaiont
    double computeEnergy(QVector<QVector<double> > &P, QVector<QVector<double> > &R,
                         QVector<QVector<double> > &Q);



    ////////////////////////////////////OPTIMIZATIONP///////////////////////////////////////////////////
    // optimize P
    QVector<QVector<double> > optimizeP(QVector<QVector<double> > &R,
                                     QVector<QVector<double> > &Q);



    ////////////////////////////////////OPTIMIZATIONQ////////////////////////////////////////////////////
    // optimize Q
    QVector<QVector<double> > optimizeQ(QVector<QVector<double> > &P,
                                     QVector<QVector<double> > &R);




    /////////////////////////////////////GETLABELS//////////////////////////////////////////////////////
    // get labels given P,R and Q
    QVector<QVector<double> > getLabels(QVector<QVector<double> > &P,
                        QVector<QVector<double> > &R,QVector<QVector<double> > &Q);



    ////////////////////////////////////OPTIMIZATION////////////////////////////////////////////////////
    // main process: optimize P and Q
    void optimization(QVector<QVector<double> > &P,
                      QVector<QVector<double> > &R,QVector<QVector<double> > &Q);


    /////////////////////////////////////IMAGESCALE//////////////////////////////////////////////////////
    void imageScale();




    /////////////////////////////////////WHEELEVENT//////////////////////////////////////////////////////
    void wheelEvent(QWheelEvent *event);




    /////////////////////////////////////CLEAR///////////////////////////////////////////////////////////
    void clear(){

        m_bg_samples.clear();
        m_GTS_pos_.clear();
        m_detected_gts_.clear();
        m_detected_gts_retected_.clear();

        m_initial_labels_.clear();
        m_final_labels_.clear();
    }

protected slots:

    void loadImage();
    void rectification();
    void detect();
    void dispInitialResult();
    void dispFinalResult();

signals:

    void dispInitialResultSignal();
    void dispFinalResultSignal();

private:

 QImage m_src_image_;
 QImage m_image_;
 QImage m_rected_image_;

 // pixels for background
 QVector<QPoint> m_bg_samples;

 QVector<QRect> m_GTS_pos_;// 一般只有一个，取一个就好

 QVector<QVector<QPoint> > m_detected_gts_;

 QVector<QVector<QPoint> > m_detected_gts_retected_;

 bool m_without_rectification_;

 int m_rect_h_;
 int m_rect_w_;

 int m_img_h_;
 int m_img_w_;

 float m_scale_;


 QVector<QVector<double> > m_initial_labels_;
 QVector<QVector<double> > m_final_labels_;

 cv::Mat m_H_;
 float m_E_;

};

#endif // REPETITIVESTRUCTURE_H
