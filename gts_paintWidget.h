#ifndef GTS_PAINTWIDGET_H
#define GTS_PAINTWIDGET_H

#include "gts_paintShape.h"

#include <QWidget>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include<QMouseEvent>
#include<QColorDialog>
#include<QRect>

class PaintWidget: public QWidget
{
    Q_OBJECT

public:

    PaintWidget(QWidget *parent = 0);

    // set image
    inline void setImage(QImage & image){
        p_image_ = image.copy(0,0, image.width(), image.height());

        clear();
        update();
    }

    // set mask (a polygon indicates foreground)
    inline void setMask(const QPolygon & mask)
    {
        p_mask_.clear();
        for(int i=0; i< mask.size(); i++)
        {
            p_mask_ << mask[i];
        }
    }

    // set rected mask (a polygon indicates foreground)
    inline void setRectedMask(const QPolygon & polygon)
    {
      p_rected_mask_.clear();
      foreach(QPoint pt, polygon)
      {
        p_rected_mask_.append(pt);
      }
    }

    // begin to draw image
    inline void setDrawImage(bool flag){
        p_draw_image_ = flag;
        update();
    }

    // get fg color
    QColor getFGColor(){return p_fg_color_; }

    // get bg color
    QColor getBGColor(){return p_bg_color_;}

    //get the bg samples
    QVector<QPoint> getBGSamples(){return p_bg_samples_;}

    //get the fg positions
    QVector<QRect> getGTSPos(){return p_gts_pos_;}

    // get the quad
    QVector<QPoint> getQuad(){ return p_quad_; }

    void clear(){

         p_shapeList_.clear();
         p_bg_samples_.clear();
         p_gts_pos_.clear();
         p_quad_.clear();
    }

protected:

    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent * event);

    bool isShapePaintFinished(){return p_shape_paint_finished_; }
    void setShapePaintFinished(bool judge ){ p_shape_paint_finished_ = judge;}


public slots:

    // prepares for drawing bake ground
    inline void drawStrokeOnBackground(){

        p_bg_color_ = QColorDialog::getColor(Qt::red, this);
        p_current_shape_type_ = Shape2D::CURVES;

        p_draw_shape_ = true;
        update();
    }

    // prepares for drawing foreground
    inline void drawRectOnForeground(){

        p_fg_color_ = QColorDialog::getColor(Qt::green, this);
        p_current_shape_type_ = Shape2D::RECT;

        // multi times
        p_gts_pos_.clear();

        p_draw_shape_ = true;
        update();
    }

    // select a quad for rectification
    inline void drawQuad()
    {
        p_current_shape_type_ = Shape2D::POLYGON;
        p_draw_shape_ = true;
        update();
    }

    // draw mask
    inline void setDrawRectedMask(bool flag)
    {
      p_draw_rected_mask_ = flag;
      if(flag == true)
      {
        p_draw_mask_ = false;
      }
      update();
    }

    inline void setDrawMask(bool flag)
    {
      p_draw_mask_ = flag;
      if(flag == true)
      {
         p_draw_rected_mask_ = false;
         update();
      }
    }

private:

    bool p_draw_image_;
    bool p_draw_shape_;
    bool p_shape_paint_finished_;
    bool p_draw_mask_;
    bool p_draw_rected_mask_;


    QImage p_image_;
    QColor p_bg_color_;
    QColor p_fg_color_;

    QPolygon p_mask_;
    QPolygon p_rected_mask_;

    Shape2D::shapeType p_current_shape_type_;
    Shape2D *p_currentShape_;
    QVector< Shape2D*> p_shapeList_;

    QVector<QPoint> p_bg_samples_;
    QVector<QRect> p_gts_pos_;

    QVector<QPoint> p_quad_;

};
#endif // PAINTWIDGET_H
