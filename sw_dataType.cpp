#include"sw_dataType.h"

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

