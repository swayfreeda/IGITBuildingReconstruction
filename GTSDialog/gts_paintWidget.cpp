#include"gts_paintWidget.h"

#include<iostream>


//-------------------------------------------------constructor---------------------------------------------//
PaintWidget::PaintWidget(QWidget *parent)
    :QWidget(parent)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    p_draw_image_ = false;

    setShapePaintFinished(true);
}



//--------------------------------------------------paintEvent--------------------------------------------//
void PaintWidget::paintEvent(QPaintEvent *event)
{
    if(p_draw_image_ == true)
    {
        QPainter painter(this);

        // draw images
        painter.drawImage(0,0,p_image_);

        // draw shapes
        foreach(Shape2D * shape, p_shapeList_)
        {
            shape->paint(painter);
        }
        update();
    }
}




//---------------------------------------------------mousePressEvent-------------------------------------//
void PaintWidget::mousePressEvent(QMouseEvent *event)
{
   // 顺序居然很重要？？？？？？？？？？？？？？
    // cosegment 1
    if((event->button()== Qt::LeftButton)&&(p_draw_image_ ==true)&&
            (isShapePaintFinished() == false)&& p_current_shape_type_== Shape2D::POLYGON)
    {
        p_currentShape_->addPoint(event->pos());

        p_currentShape_->setStartPoint(event->pos());
        p_currentShape_->setEndPoint(event->pos());

        p_quad_.append(event->pos());
    }

    // cosegment 2
    if((event->button()== Qt::RightButton)&&(p_draw_image_ ==true)&&
            (isShapePaintFinished() == false)&& p_current_shape_type_== Shape2D::POLYGON)
    {
        setMouseTracking(false);
        p_currentShape_->setShapeCompleted(true);
        setShapePaintFinished(true);
    }

    // cosegment 3
    if((event->button()== Qt::LeftButton)&&(p_draw_image_ == true)&& (isShapePaintFinished()== true))
    {
        setShapePaintFinished(false);

        switch(p_current_shape_type_)
        {
        case Shape2D::RECT:

            p_currentShape_ = new Rect2D();
            p_currentShape_->setColor(p_fg_color_);
            p_shapeList_<<p_currentShape_;

            p_currentShape_->setStartPoint(event->pos());
            p_currentShape_->setEndPoint(event->pos());
            break;

        case Shape2D::CURVES:

            p_currentShape_ = new Curves2D();
            p_currentShape_->setColor(p_bg_color_);
            p_shapeList_<< p_currentShape_;

            p_currentShape_->addPoint(event->pos());
            break;
        case Shape2D::POLYGON:

            setMouseTracking(true);

            p_currentShape_ = new Polygon2D();
            p_shapeList_ << p_currentShape_;
            p_currentShape_->setShapeCompleted(false);

            p_currentShape_->addPoint(event->pos());
            p_currentShape_->setStartPoint(event->pos());
            p_currentShape_->setEndPoint(event->pos());

            p_quad_.append(event->pos());

            //std::cout<<event->pos().x() <<", " << event->pos().y()<< std::endl;
            break;

        default:
            break;
        }
    }
}



//-------------------------------------------mouseMoveEvent-----------------------------------------------//
void PaintWidget::mouseMoveEvent(QMouseEvent *event)
{
    if((isShapePaintFinished()==false) && (p_draw_image_ == true) )
    {

        switch(p_current_shape_type_)
        {
        case Shape2D::RECT:

            p_currentShape_->setEndPoint(event->pos());
            break;

        case Shape2D::CURVES:
            p_currentShape_->setEndPoint(event->pos());
            p_currentShape_->addPoint(event->pos());

            //the width of the pen
            for(int i=-3; i< 4; i++)
            {
                for(int j=- 3; j< 4; j++)
                {
                    QPoint pt(event->pos().x() + i, event->pos().y()+j);
                    if(p_bg_samples_.contains(pt)) continue;
                    p_bg_samples_<<pt;
                }
            }

            break;
        case Shape2D::POLYGON:
            p_currentShape_->setEndPoint(event->pos());

            break;
        default:
            break;
        }

        update();
    }
}



//--------------------------------------------mouseReleaseEvent-------------------------------------------//
void PaintWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if((event->button()== Qt::LeftButton)&&(p_draw_image_ == true)&&(isShapePaintFinished()== false) )
    {
        switch(p_current_shape_type_)
        {
        case Shape2D::RECT:
        {
            // store the postions of GTS
            QRect rect;
            rect.setTopLeft( p_currentShape_->getStartPoint());
            rect.setBottomRight(  p_currentShape_->getEndPoint() ) ;

            p_gts_pos_<<rect;

            setShapePaintFinished(true);
            break;
        }
        case Shape2D::CURVES:

            setShapePaintFinished(true);
            break;
        case Shape2D::POLYGON:
            break;

        default:
            break;
        }

        update();
    }
}

