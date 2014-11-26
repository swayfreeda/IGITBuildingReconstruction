// description: defines a class for repetitive structure detection
// Time:  11/25/2014
// Author: Sway
// Organization: Institue of Automation, Chinese Academy of Science

#ifndef GTS_REPETITIVESTRUCTURE_H
#define GTS_REPETITIVESTRUCTURE_H

#include"ui_gts_repetitiveStructure.h"
#include"gts_paintWidget.h"

#include<QWidget>
#include<QObject>

#include"opencv2/opencv.hpp"

#include<QWheelEvent>
#include<QPolygon>

class GTSDetectionDialog: public QWidget, public Ui::GTSDetectionDialog{

    Q_OBJECT

public:
    GTSDetectionDialog(QWidget *parent =0);

    // get initia labels through GMM clustering
    QVector<QVector<double> > getInitialLabels();

    // compute the enegy
    double computeEnergy(QVector<QVector<double> > &P, QVector<QVector<double> > &R,
                         QVector<QVector<double> > &Q);

    // optimize P
    QVector<QVector<double> > optimizeP(QVector<QVector<double> > &R,
                                        QVector<QVector<double> > &Q);

    // optimize Q
    QVector<QVector<double> > optimizeQ(QVector<QVector<double> > &P,
                                        QVector<QVector<double> > &R);

    // get labels given P,R and Q
    QVector<QVector<double> > getLabels(QVector<QVector<double> > &P,
                                        QVector<QVector<double> > &R,QVector<QVector<double> > &Q);

    // main process: optimize P and Q
    void optimization(QVector<QVector<double> > &P,
                      QVector<QVector<double> > &R,QVector<QVector<double> > &Q);

    void imageScale();

    void wheelEvent(QWheelEvent *event);

    void clear(){

        m_bg_samples.clear();
        m_GTS_pos_.clear();
       // m_detected_gts_.clear();
       // m_detected_gts_rected_.clear();

        m_initial_labels_.clear();
        m_final_labels_.clear();
    }


    // set image
    void setImage(const QImage & image)
    {
        m_with_rectification_ = false;

        m_detected_gts_.clear();
        m_detected_gts_rected_.clear();


        // clear info
        clear();

        m_src_image_ = image.copy(0, 0, image.width(), image.height());
        m_image_ = m_src_image_.scaled(QSize(m_scale_* m_src_image_.width(),
                                             m_scale_* m_src_image_.height()) );

        m_img_h_ = m_image_.height();
        m_img_w_ = m_image_.width();

        // draw image in the top right grid
        widget_rectified->setImage(m_image_);
        widget_rectified->setDrawImage(true);

        // draw image in the top left grid
        label_input->setPixmap(QPixmap::fromImage(m_image_));

        update();

    }

    // set mask
    void setMask(const QPolygon &mask)
    {
        m_src_mask_.clear();
        m_mask_.clear();
        for(int i=0; i< mask.size(); i++)
         {
            QPoint pt = mask[i];
             m_src_mask_<< pt;
             m_mask_<<m_scale_ * pt ;
         }
        widget_rectified->setMask(m_mask_);
        widget_rectified-> setDrawMask(true);
        update();
    }

   // return results
   QVector<QPolygon>  getGTS(){ return m_detected_gts_; }

protected slots:

    void loadImage();
    void rectification();
    void detect();
    void dispInitialResult();
    void dispFinalResult();

    // abandon the detected gts results
    void abandonResults()
    {
      m_detected_gts_.clear();
      m_detected_gts_rected_.clear();

      m_initial_labels_.clear();
      m_final_labels_.clear();

      widget_rectified->clear();
      update();

    }

    //accept the results
    void acceptResults()
    {
      this->close();
    }

signals:

    void dispInitialResultSignal();
    void dispFinalResultSignal();

private:

    // ooriginal size
    QImage m_src_image_;
    // resized image
    QImage m_image_;
    QImage m_rected_image_;

    // the background should be in the mask
    QPolygon m_src_mask_;
    QPolygon m_mask_;
    QPolygon m_rected_mask_;


    QVector<QPoint> m_bg_samples;
    QVector<QRect> m_GTS_pos_;// 一般只有一个，取一个就好

    QVector<QPolygon > m_detected_gts_;
    QVector<QPolygon > m_detected_gts_rected_;
    bool m_with_rectification_;

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
