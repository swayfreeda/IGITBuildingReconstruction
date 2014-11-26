#ifndef GTS_PAINTSHAPE_H
#define GTS_PAINTSHAPE_H

#include<QPoint>
#include<QPainter>
/////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////

class Line2D: public Shape2D
{
   public:
    Line2D();
    void paint(QPainter &painter);
};

////////////////////////////////////////////////////////////////////////////////////////////
class Polygon2D: public Shape2D
{
public:
    Polygon2D();

    void paint(QPainter &painter);
    void addPoint(QPoint point);
    void draw();
private:

};
///////////////////////////////////////////////////////////////////////////////////////////


class Rect2D: public Shape2D
{
  public:
    Rect2D();
    void paint(QPainter &painter);

};


///////////////////////////////////////////////////////////////////////////////////////////////
class Curves2D: public Shape2D{

public:
    Curves2D();
    void paint(QPainter &painter);
    void addPoint(QPoint point);

 private:

};

#endif // PAINTSHAPE_H
