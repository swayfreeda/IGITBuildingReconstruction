#include"gts_paintShape.h"
#include<QPen>
#include<QBrush>

#include<math.h>

#include <qgl.h>
#define Pi 3.1416


#include<iostream>


//-------------------------------------------------shape2D------------------------------------------------//
Shape2D::Shape2D()
{
  PaintCompleted_ = false;
}



//--------------------------------------------------Line2D-------------------------------------------------//
Line2D::Line2D()
{}



//--------------------------------------------------Rect2D-------------------------------------------------//
Rect2D::Rect2D()
{}


//--------------------------------------------------Polygon2D---------------------------------------------//
Polygon2D:: Polygon2D()
{
    setShapeCompleted(false);
}


//---------------------------------------------------paint-------------------------------------------------//
void Line2D::paint(QPainter &painter)
{
    painter.drawLine(start_, end_);
}



//---------------------------------------------------paint------------------------------------------------//
void Polygon2D:: paint(QPainter &painter)
{

    painter.setPen( QPen(Qt::blue, 3, Qt::SolidLine, Qt::RoundCap) );

    if(isShapeCompleted() ==false )
    {
        for(int i=0; i< points_.size() -1; i++)
        {
            painter.drawLine(points_[i], points_[i+1]);
        }
        painter.drawLine(start_, end_);
    }
    else{

        for(int i=0; i< points_.size(); i++)
        {
            int id0 = i;
            int id1 = (i+1)% points_.size();
            painter.drawLine(points_[id0], points_[id1]);
        }

    }

    painter.setPen(QPen());

}



//----------------------------------------------addPoint---------------------------------------------------//
void Polygon2D:: addPoint(QPoint point)
{
    points_<<point;
}




//-----------------------------------------------draw-----------------------------------------------------//
void Polygon2D::draw()
{
    // draw boundary
    glColor3f(1.0, 0, 0);
    glLineWidth(2.0);
    glBegin(GL_LINE_STRIP);
      foreach(QPoint point, points_)
       {
          glVertex2i(point.x(), point.y());
       }
    glVertex2i(end_.x(), end_.y());
    glEnd();
    glLineWidth(1.0);

    // draw vertex
    glColor3f(1.0, 1.0, 0.0);

    foreach(QPoint point, points_)
       {
         glBegin(GL_LINE_LOOP);
              for(int i=0; i<7; ++i)
                  glVertex2f(point.x() + 4*cos(2*Pi/7*i), point.y() + 4*sin(2*Pi/7*i));
         glEnd();
       }
}



//------------------------------------------------paint---------------------------------------------------//
void Rect2D::paint(QPainter &painter)
{
    painter.setBrush(QBrush(s_color_, Qt::SolidPattern));
    painter.drawRect(QRect(start_, end_));
    painter.setBrush(QBrush());
}


//-------------------------------------------------constructor-------------------------------------------//
Curves2D::Curves2D()
{

}



//--------------------------------------------------paint-------------------------------------------------//
void Curves2D::paint(QPainter &painter)
{

    painter.setPen(QPen(s_color_, 7, Qt::SolidLine, Qt::RoundCap));
    for(int i=0; i< points_.size()-1; i++)
    {
        painter.drawLine(points_[i], points_[i+1]);
    }
    // 恢复默认设置
    painter.setPen(QPen());
}



//--------------------------------------------------addPoint-----------------------------------------------//
void Curves2D::addPoint(QPoint point)
{
    points_<<point;
}
