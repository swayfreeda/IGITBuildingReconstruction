#include"sw_dataType.h"
#include<GL/gl.h>

// 重载算数操作符号+
Vec3  operator +(const Vec3 & vl, const Vec3 & vr) //算数和关系操作符一般定义为非成员函数
{
    Vec3 tmp(vl);
    tmp.x_ += vr.x_;
    tmp.y_ += vr.y_;
    tmp.z_ += vr.z_;

    return tmp;
}

// 重载算术操作符号-
Vec3  operator -(const Vec3 & vl, const Vec3 & vr)
{
    Vec3 tmp(vl);
    tmp.x_ -= vr.x_;
    tmp.y_ -= vr.y_;
    tmp.z_ -= vr.z_;

    return tmp;
}

// 重载输出操作符号<<
ostream& operator <<(ostream os, const Vec3 v)
{
    os<<"[ "<< v.x_<<", "<< v.y_<< ", "<< v.z_<<" ]"<<endl;

    return os;
}

// 重载算术操作符*
Vec3  operator *(const Vec3 & v, const float f)
{
    Vec3 tmp(v);
    tmp.x_ *= f;
    tmp.y_ *= f;
    tmp.z_ *= f;
    return tmp;
}


// 重载算术操作符*
Vec3  operator *( const float f, const Vec3 & v)
{
    Vec3 tmp(v);
    tmp.x_ *= f;
    tmp.y_ *= f;
    tmp.z_ *= f;
    return tmp;
}

// 重载算术操作符*
float operator *(const Vec3& vl, const Vec3& vr)
{
    float tmp = vl.x_ * vr.x_ + vl.y_* vr.y_ + vl.z_* vr.z_;
    return tmp;
}

Vec3  operator /( const Vec3 & v,const float f)
{
    Vec3 tmp(v);
    tmp.x_ /= f;
    tmp.y_ /= f;
    tmp.z_ /= f;
    return tmp;
}

//叉乘运算
Vec3 cross(const Vec3 &vl, const Vec3 &vr)
{

    Vec3 tmp;
    tmp.x_ = vl.y_*vr.z_ - vl.z_*vr.y_;
    tmp.y_ = vl.z_*vr.x_ - vl.x_*vr.z_;
    tmp.z_ = vl.x_*vr.y_ - vl.y_*vr.x_;

    return tmp;
}



/********************************************************************************/
/*                      CLASS  CAMERA                                         */
/********************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// computet the direction and the position of the camera
void Camera::computePosAndDir()
{
    // compute the position
    cv::Mat pos = -rotation_.t()* trans_;
    for(int i=0; i<3; i++)
    {
        pos_.at<float>(i) = pos.at<float>(i);
    }
    pos.release();
    // compute the direction
    cv::Mat dir_oir(3,1,CV_32FC1);
    dir_oir.setTo(0);
    dir_oir.at<float>(2) = -1;

    cv::Mat dir = rotation_.t()* dir_oir;
    for(int i=0; i<3; i++)
    {
        dir_.at<float>(i) = dir.at<float>(i);
    }

    pos.release();
    dir_oir.release();
    dir.release();

}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
void Camera::decomposeProjMats()
{

    // 1.0 get direction
    this->dir_.at<float>(0) = this->project_.at<float>(2,0);
    this->dir_.at<float>(1) = this->project_.at<float>(2,1);
    this->dir_.at<float>(2) = this->project_.at<float>(2,2);
    this->dir_ = this->dir_/ norm(this->dir_);

    // 2.0 get position
    cv::Mat KR(3,3,CV_32FC1);
    cv::Mat KT(3,1,CV_32FC1);
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            KR.at<float>(i,j) = this->project_.at<float>(i,j);
        }
    }
    for(int i=0; i<3; i++)
        KT.at<float>(i,0) = this->project_.at<float>(i,3);

    this->pos_ = -KR.inv()* KT;

    // 3.0 compute the focal
    cv::Mat R0(3,1, CV_32FC1);
    cv::Mat R1(3, 1, CV_32FC1);
    cv::Mat R2(3, 1, CV_32FC1);

    for(int i=0; i<3; i++)
    {
        R0.at<float>(i) = KR.at<float>(0, i);
        R1.at<float>(i) = KR.at<float>(1, i);
        R2.at<float>(i) = KR.at<float>(2, i);
    }

    this->focal_ = 0.5*abs(norm(R0.cross(R2)))+ 0.5*abs(norm(R1.cross(R2)));


    // 4.0 axises of the camera
    this->zaxis_ = this->dir_;
    this->yaxis_ = this->zaxis_.cross(R0);
    this->yaxis_ = this->yaxis_/norm(this->yaxis_);

    this->xaxis_ = this->yaxis_.cross(this->zaxis_);
    this->xaxis_ = this->xaxis_/norm(this->xaxis_);

    KR.release();
    KT.release();
    R0.release();
    R1.release();
    R2.release();
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// draw camera
void Camera::draw()
{
    //cv::Mat rotation = rotation_.t();
    //cv::Mat  trans = -rotation_.t()* trans_;
    //qglviewer::Vec dir;
    //float angle = rotationMatrixToAngleAxis(rotation, dir);
    //cout<<"angle axis: "<<angle* dir.x<<", "<<angle* dir.y<<", "<<angle* dir.z<<endl;
    // cout<<"Camera Pos: "<< trans.at<float>(0)<<", "<< trans.at<float>(1)<<", "<< trans.at<float>(2)<<endl;


    glBegin(GL_QUADS);

    glVertex3f(-0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0.4, -0.3, -focal_/(2500.0));
    glVertex3f(-0.4, -0.3, -focal_/(2500.0));
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(-0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(0.4,-0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(-0.4,-0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();
}

#if 0
//////////////////////////////////////////////////////////////////////////////////////////////////////
qglviewer::Vec Camera::project(const qglviewer::Vec& coord)
{
    qglviewer::Vec p2D;
    cv::Mat p3D(4,1,CV_32FC1);
    p3D.at<float>(0) = coord.x;
    p3D.at<float>(1) = coord.y;
    p3D.at<float>(2) = coord.z;
    p3D.at<float>(3) = 1.0;

    cv::Mat ptmp = project_* p3D;
    if(ptmp.at<float>(2)==0)
    {
        p2D.x = -1;
        p2D.y = -1;
        p2D.z = 1;
    }
    else
    {
        p2D.x = ptmp.at<float>(0)/ptmp.at<float>(2);
        p2D.y = ptmp.at<float>(1)/ptmp.at<float>(2);
        p2D.z = 1.0;
    }
    ptmp.release();
    p3D.release();
    return p2D;
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////
PointXY  Camera::project(const Vec3 &coord)
{
    PointXY p2D;
    cv::Mat p3D(4,1,CV_32FC1);
    p3D.at<float>(0) = coord.x_;
    p3D.at<float>(1) = coord.y_;
    p3D.at<float>(2) = coord.z_;
    p3D.at<float>(3) = 1.0;

    cv::Mat ptmp = project_* p3D;
    if(ptmp.at<float>(2)==0)
    {
        p2D.x = -1;
        p2D.y = -1;
    }
    else
    {
        p2D.x = ptmp.at<float>(0)/ptmp.at<float>(2);
        p2D.y = ptmp.at<float>(1)/ptmp.at<float>(2);
    }
    ptmp.release();
    p3D.release();
    return p2D;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////


