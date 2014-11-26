// Descrition: This file defines some shapes for drawing in the paintWidget
// Time: 11/24/2014
// Author: Sway
// Organization:  Institute of Automation, Chinese Academy of Sciences


#ifndef GTS_PAINTSHAPE_H
#define GTS_PAINTSHAPE_H

#include<QPoint>
#include<QPainter>
///////////////////////////////////////////SHAPE2D////////////////////////////////////////////////////////////
class Shape2D
{
  public:
    enum shapeType{LINE, RECT, POLYGON, CURVES};

    Shape2D();

    void setStartPoint(QPoint s){start_ = s; }
    void setEndPoint(QPoint s){ end_ = s;}

    QPoint getStartPoint(){return start_;}
    QPoint getEndPoint(){return end_;}

    bool isShapeCompleted(){return PaintCompleted_;}
    bool setShapeCompleted(bool judge){PaintCompleted_ = judge;}

    int getPointsSize(){return points_.size();}

    QPoint getNthPoint(int n){return points_.at(n);}

    void virtual paint(QPainter &painter)=0;
    void virtual addPoint(QPoint point){}

    void setColor(QColor&color){  s_color_ = color;}

protected:

    QPoint start_;
    QPoint end_;
    QVector<QPoint> points_;

    QColor s_color_;

    bool PaintCompleted_;
};




////////////////////////////////////////////LINE2D//////////////////////////////////////////////////////////
class Line2D: public Shape2D
{
   public:
    Line2D();
    void paint(QPainter &painter);
};




/////////////////////////////////////////////POLYGON2D//////////////////////////////////////////////////////
class Polygon2D: public Shape2D
{
public:
    Polygon2D();

    void paint(QPainter &painter);
    void addPoint(QPoint point);
    void draw();
private:

};




///////////////////////////////////////////////RECT2D///////////////////////////////////////////////////////
class Rect2D: public Shape2D
{
  public:
    Rect2D();
    void paint(QPainter &painter);

};




///////////////////////////////////////////////CURVES2D/////////////////////////////////////////////////////
class Curves2D: public Shape2D{

public:
    Curves2D();
    void paint(QPainter &painter);
    void addPoint(QPoint point);

 private:

};

#endif // PAINTSHAPE_H
