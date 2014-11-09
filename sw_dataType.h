// Description: this header defines some data structures, and part of these structures are dependent on Qt
// Time: 11/07/2014
// Author: Sway
// Organization: Institute of Automation, Chinese Academy of Sciences


#ifndef DATATYPE_H
#define DATATYPE_H

#include"stdlib.h"
#include"algorithm"
#include"cmath"
#include<vector>
#include"cmath"
#include"opencv2/opencv.hpp"

#include <iostream>
#include <string>
#include<QVector>
#include <QColor>

using namespace std;



////////////////////////////////////////////////PointXY///////////////////////////////////////////////////////////////////
class PointXY
{
public:

    PointXY(){x = 0; y=0;}
    PointXY(const PointXY&pt)
    {
       this->x = pt.x;
       this->y = pt.y;
    }
    PointXY(float x, float y)
    {
      this->x = x;
      this->y = y;
    }

    PointXY & operator- (const PointXY & pt)
    {
        this->x -= pt.x;
        this->y -= pt.y;

        return *this;
    }

    PointXY &operator = (const PointXY & pt)
    {
      this->x = pt.x;
      this->y = pt.y;
    }

    float x;
    float y;
};



////////////////////////////////////////////////PointXYZ///////////////////////////////////////////////////////////////////
class PointXYZ
{
public:

    PointXYZ(){}
    PointXYZ(const PointXYZ&pt)
    {
      this->x = pt.x;
      this->y = pt.y;
      this->z = pt.z;
    }
    PointXYZ(float x, float y, float z)
    {
       this->x = x;
       this->y = y;
       this->z = z;
    }

    PointXYZ & operator =(const PointXYZ & pt)
    {
        this->x = pt.x;
        this->y = pt.y;
        this->z = pt.z;

        return *this;
    }

    bool operator ==(const PointXYZ &pt)
    {
        // return(x == pt.x && y==pt.y &&z == pt.z);
        return(abs(x -pt.x)<0.01 && abs(y-pt.y)<0.01 &&abs(z - pt.z)<0.01);
    }

public:
    QVector<int> vis;

    float x;
    float y;
    float z;
    bool isProcessed;
    bool inconsist_;

};



////////////////////////////////////////////////PointXYZRGB////////////////////////////////////////////////////////////////
class PointXYZRGB
{
public:

    PointXYZRGB(){}
    PointXYZRGB(const PointXYZRGB&pt)
    {
       this->x = pt.x;
       this->y = pt.y;
       this->z = pt.z;

       this->r = pt.r;
       this->g = pt.g;
       this->b = pt.b;
       
       this->isProcessed = pt.isProcessed;
       this->id = pt.id;

       this-> vis = pt.vis;
       this-> inconsist_ = pt.inconsist_;

    }
    PointXYZRGB(float x, float y, float z, uchar r, uchar g, uchar b)
   {
        this->x = x;
        this->y = y;
        this->z = z;
     
        this->r = (float)r;
        this->g = (float)g;
        this->b = (float)b;
   }

    bool isProcessed;

    int id;

    float x;
    float y;
    float z;

    float r;
    float g;
    float b;

    QVector<int> vis;
    bool inconsist_;
};


////////////////////////////////////////////////PointXYZNormal////////////////////////////////////////////////////////////
class PointXYZNormal{

public:

    PointXYZNormal(){}
    PointXYZNormal(const PointXYZNormal & pt)
    {
        this->x = pt.x;
        this->y = pt.y;
        this->z = pt.z;


        this->normal_x = pt.normal_x;
        this->normal_y = pt.normal_y;
        this->normal_z = pt.normal_z;

        this->inconsist_ = pt.inconsist_;
    }
    PointXYZNormal(float xx, float yy, float zz): x(xx), y(yy), z(zz)
    {
        this->normal_x = 0.0;
        this->normal_y = 0.0;
        this->normal_z = 0.0;

        this->inconsist_= false;
    }
    PointXYZNormal(float xx, float yy, float zz,
                   float nor_x, float nor_y, float nor_z): x(xx), y(yy), z(zz),
        normal_x(nor_x), normal_y(nor_y), normal_z(nor_z)
    {
        this->inconsist_= false;
    }
    PointXYZNormal &operator =(const PointXYZNormal & pt)
    {
        this->x = pt.x;
        this->y = pt.y;
        this->z = pt.z;

        this->normal_x = pt.normal_x;
        this->normal_y = pt.normal_y;
        this->normal_z = pt.normal_z;

        this->inconsist_ = pt.inconsist_;

        return *this;
    }

    bool operator ==( const PointXYZNormal & pt)
    {
        return(abs(x -pt.x)<0.01 && abs(y-pt.y)<0.01 &&abs(z - pt.z)<0.01&&
               abs(normal_x - pt.normal_x)<0.01&&abs(normal_y - pt.normal_y)<0.01&&
               abs(normal_z - pt.normal_z)<0.01);

    }

    float x;
    float y;
    float z;

    float normal_x;
    float normal_y;
    float normal_z;

    bool inconsist_;

};


///////////////////////////////////////////////PointXYZRGBNrmal/////////////////////////////////////////////////////////////
class PointXYZRGBNormal
{
public:

    // constructor
    PointXYZRGBNormal(){

        this->x = 0;
        this->y = 0;
        this->z = 0;

        this->r = 0;
        this->g = 0;
        this->b = 0;

        this->normal_x = 0;
        this->normal_y = 0;
        this->normal_z = 0;

        this->id = 0;
        this->label_ = -1;
        this->vis.clear();
        this->inconsist_ = 0;

    }

    // constructor
    PointXYZRGBNormal(int id){

        this->x = 0;
        this->y = 0;
        this->z = 0;

        this->r = 0;
        this->g = 0;
        this->b = 0;

        this->normal_x = 0;
        this->normal_y = 0;
        this->normal_z = 0;

        this->id = id;
        this->label_ = -1;
        this->vis.clear();
        this->inconsist_ = 0;

    }

    // constructor
    PointXYZRGBNormal(float x, float y, float z){

        this->x = x;
        this->y = y;
        this->z = z;

        this->r = 0;
        this->g = 0;
        this->b = 0;

        this->normal_x = 0;
        this->normal_y = 0;
        this->normal_z = 0;

        this->inconsist_ = 0;

    }

    // copy constructor
    PointXYZRGBNormal(const PointXYZRGBNormal &pt )
    {
        this->x = pt.x;
        this->y = pt.y;
        this->z = pt.z;

        this->r = pt.r;
        this->g = pt.g;
        this->b = pt.b;

        this->normal_x = pt.normal_x;
        this->normal_y = pt.normal_y;
        this->normal_z = pt.normal_z;

        this->id = id;
        this->vis = pt.vis;
        this->inconsist_ = pt.inconsist_;
    }

    // copy constructor from another Type
    PointXYZRGBNormal(const PointXYZNormal &pt )
    {
        this->x = pt.x;
        this->y = pt.y;
        this->z = pt.z;

        this->r = 0;
        this->g = 0;
        this->b = 0;

        this->normal_x = pt.normal_x;
        this->normal_y = pt.normal_y;
        this->normal_z = pt.normal_z;

        this->inconsist_ = pt.inconsist_;
    }

    // overload of operator=
    PointXYZRGBNormal & operator =(const PointXYZRGBNormal & pt)
    {
        this->x = pt.x;
        this->y = pt.y;
        this->z = pt.z;

        this->r = pt.r;
        this->g = pt.g;
        this->b = pt.b;

        this->normal_x = pt.normal_x;
        this->normal_y = pt.normal_y;
        this->normal_z = pt.normal_z;

        this->id = id;
        this->vis = pt.vis;
        this->inconsist_ = pt.inconsist_;

        return *this;
    }


    int id;

    float x;
    float y;
    float z;

    float r;
    float g;
    float b;

    float normal_x;
    float normal_y;
    float normal_z;

    QVector<uint> vis;

    bool inconsist_;

    int label_;
};



////////////////////////////////////////////////Vec3//////////////////////////////////////////////////////////////////////////
class Vec3{

public:
    Vec3(){ x_ = 0; y_ =0; z_ =0;}
    Vec3(float x, float y, float z): x_(x), y_(y), z_(z){}

    // 复制构造函数
    Vec3(const Vec3 & v)
    {
        this->x_ = v.x_;
        this->y_ = v.y_;
        this->z_ = v.z_;
    }

    // 复制操作符
    Vec3 & operator =(const Vec3 & v)
    {
        this->x_ = v.x_;
        this->y_ = v.y_;
        this->z_ = v.z_;

        return *this;
    }

#if 1
    // 下标访问操作符需要重载两个版本const 和 非 const版本 // 返回引用可以进行左右操作符的操作
    float & operator [](const size_t index)
    {
        switch(index)
        {
        case 0:
            return x_;
            break;
        case 1:
            return y_;
            break;
        case 2:
            return z_;
            break;
        default:
            break;
        }
    }

    // 注意 const版本需要const
    const float &operator [](const size_t index) const
    {
        switch(index)
        {
        case 0:
            return x_;
            break;
        case 1:
            return y_;
            break;
        case 2:
            return z_;
            break;
        default:
            break;
        }
    }

#endif
    // norm
    float  &norm()
    {
        float v = sqrt(x_*x_ + y_*y_ + z_*z_);
        return  v;
    }

    // normalization
    void normalize()
    {
        float l = this->norm();
        if(l ==0) std::cerr<<"Divided by 0!"<<std::endl;
        if(l!=0)
        {
            x_ /= l;
            y_ /= l;
            z_ /= l;
        }
    }
   
    // 进行map的排序的操作 如果v1 和 v2 之间相互不存在小于的关系，则认为v1 和 v2 相同
     bool operator < ( const Vec3& v) const 
     {
         bool flag0 = false;
         bool flag1 = false;
         bool flag2 = false;

         if(this->x_ != v.x_)
         {
           flag0 =  this->x_< v.x_ ? true: false;
         }
         if((this->x_== v.x_)&& ( this->y_ != v.y_ ))
         {
           flag1 =  this->y_< v.y_ ? true: false;
         }
         if( (this->x_ == v.x_)&&(this->y_ == v.y_)
                 &&(this->z_ != v.z_ ))
         {
           flag2 =  this->z_ < v.z_? true: false;
         }

        return flag0||flag1||flag2;
     }

public:
    float x_;
    float y_;
    float z_;
};

Vec3  operator +(const Vec3 & vl, const Vec3 & vr); // 注意返回不能是引用
Vec3  operator -(const Vec3 & vl, const Vec3 & vr);
Vec3  operator *(const Vec3 & v, const float f);
Vec3  operator *( const float f, const Vec3 & v);
Vec3  operator /( const Vec3 & v,const float f);
float operator *(const Vec3& vl, const Vec3& vr);
ostream& operator <<(ostream os, const Vec3 v);
Vec3 cross(const Vec3 &vl, const Vec3 &vr);

////////////////////////////////////////////////Line3D//////////////////////////////////////////////////////////////////////
class Line3D
{
public:

    Line3D(){}
    Line3D(PointXYZRGBNormal& pt1, PointXYZRGBNormal& pt2){
        pt1_ = pt1;
        pt2_ = pt2;
    }

public:

     enum LineType{VERTICAL, PARALLEL, COLINEAR};

    PointXYZRGBNormal pt1_;
    PointXYZRGBNormal pt2_;

};



////////////////////////////////////////////////Camera//////////////////////////////////////////////////////////////////////
class Camera{

public:
    Camera()
    {
        rotation_.create(3,3,CV_32FC1);
        trans_.create(3,1,CV_32FC1);
        project_.create(3,4,CV_32FC1);

        pos_.create(3,1,CV_32FC1);
        dir_.create(3,1,CV_32FC1);

        xaxis_.create(3,1,CV_32FC1);
        yaxis_.create(3,1,CV_32FC1);
        zaxis_.create(3,1,CV_32FC1);

        rotation_.setTo(0);
        trans_.setTo(0);
        project_.setTo(0);

        pos_.setTo(0);
        dir_.setTo(0);

        xaxis_.setTo(0);
        yaxis_.setTo(0);
        zaxis_.setTo(0);

        focal_ = 2000.0;  // 默认值

        k0 = 0;
        k1 = 0;
    }

    Camera(const Camera &cam )
    {
       this->focal_ = focal_;
       cam.rotation_.copyTo(this->rotation_);
       cam.trans_.copyTo((this->trans_));
       cam.project_.copyTo(this->project_);

       this->k0 = cam.k0;
       this->k1 = cam.k1;

       cam.pos_.copyTo(this->pos_);
       cam.dir_.copyTo(this->dir_);

       this->img_dir_ = cam.img_dir_;
       this->color_ = cam.color_;


       cam.xaxis_.copyTo(this->xaxis_);
       cam.yaxis_.copyTo(this->yaxis_);
       cam.zaxis_.copyTo(this->zaxis_);

    }

    ~Camera()
    {
        rotation_.release();
        trans_.release();
        project_.release();
        pos_.release();
        dir_.release();

    }


#if 0
    // computet the direction and the position of the camera
    void computePosAndDir();
#endif

    // get all other information from the projection matrix
    void decomposeProjMats();

    // draw camera
    void draw();

#if 0
    // projcet a point from 3D to 2D
    qglviewer::Vec  project(const qglviewer::Vec & coord);
#endif

    PointXY  project(const Vec3 &coord);


public:

    // focal of the camera
    float focal_;
    // the rotation matrix of the camera
    cv::Mat rotation_;
    // the translation vector of the camera
    cv::Mat trans_;
    // the projection matrix of the matrix
    cv::Mat project_;

    float k0;
    float k1;

    // the position of the camera
    cv::Mat pos_;
    // the direction of the camera
    cv::Mat dir_;

    // the image dir of the image corresponding to the camera
    QString img_dir_;
    QColor color_;

    // axis  correspoding to the camera coordinates
    cv::Mat xaxis_;
    cv::Mat yaxis_;
    cv::Mat zaxis_;
};

#endif // DATATYPE_H
