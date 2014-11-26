#include"stdlib.h"
#include"opencv2/opencv.hpp"
#include<vector>


using namespace std;
using namespace cv;


int main(int argc, char **argv)
{

  Point2f src0(0, 0);
  Point2f src1(50, 0);
  Point2f src2(50, 50);
  Point2f src3(0, 50);

  vector<Point2f> srcPts;
  srcPts.push_back(src0);srcPts.push_back(src1);
  srcPts.push_back(src2);srcPts.push_back(src3);

  Point2f dst0(0, 0);
  Point2f dst1(50, 0);
  Point2f dst2(25, 50);
  Point2f dst3(-25, 50);

  vector<Point2f> dstPts;
  dstPts.push_back(dst0);dstPts.push_back(dst1);
  dstPts.push_back(dst2);dstPts.push_back(dst3);

   Mat H = findHomography(srcPts, dstPts, RANSAC, 3);

   cout<<"matrix type: "<< H.type()<<endl;

   Mat HInv = H.inv();

   cout<<"Homography:  "<< H<<endl;
   cout<<"Inv of Homography:  "<< HInv<<endl;


   for(int i=0; i< srcPts.size(); i++)
   {
       cv::Mat pt(3, 1, CV_64FC1);
       pt.at<double>(0) = srcPts[i].x;
       pt.at<double>(1) = srcPts[i].y;
       pt.at<double>(2) = 1.0;
       cout<<"src: "<< pt<<"----> dst: "<< H*pt<<endl;
   }

   for(int i=0; i< dstPts.size(); i++)
   {
      cv::Mat pt(3, 1, CV_64FC1);
      pt.at<double>(0) = dstPts[i].x;
      pt.at<double>(1) = dstPts[i].y;
      pt.at<double>(2) = 1.0;
      cout<<"dst: "<< pt<<"----> src: "<< HInv*pt<<endl;
   }

  return 0;
}
