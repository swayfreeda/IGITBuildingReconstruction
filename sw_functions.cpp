#include"sw_functions.h"

float angleAxisFromDirections(Vec3 & src, Vec3& dest, Vec3 & axis)
{
    // source direction
    cv::Mat direc1(3, 1, CV_32FC1);
    direc1.at<float>(0) = src.x_; direc1.at<float>(1)= src.y_; direc1.at<float>(2) = src.z_;

    // destinate direction
    cv::Mat direc2(3, 1, CV_32FC1);
    direc2.at<float>(0) = dest.x_; direc2.at<float>(1)= dest.y_; direc2.at<float>(2) = dest.z_;

    // axis
    cv::Mat Axis = direc1.cross(direc2);
    double innerPro = direc1.dot(direc2);
    double angle = acos(innerPro/(norm(direc1)*norm(direc2)));

    Axis = Axis/norm(Axis);

    axis.x_ = Axis.at<float>(0);
    axis.y_ = Axis.at<float>(1);
    axis.z_ = Axis.at<float>(2);

    direc1.release();
    direc2.release();
    Axis.release();

    return angle;


}
