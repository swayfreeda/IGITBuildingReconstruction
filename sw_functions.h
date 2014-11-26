//Description:  This file defines all the functions that are used in the file.
// Name: Sway
// Time: 11/07/2014
// Organization: Institute of Automation, Chinese Academy of Sciences

//=====================================================================================================================//

#ifndef SW_FUNCTIONS_H
#define SW_FUNCTIONS_H

#include"sw_dataType.h"

#include"opencv2/opencv.hpp"

#include<QVector>
#include<QImage>

#include <iostream>
#include <string>
#include <vector>
#include <utility>
using namespace std;


typedef PointXYZRGBNormal Point;


// relationship between lines
enum LineType{ PARALLEL, INTERSET , COLINEAR};




// 11/07/2014 add functions
////////////////////////////////////ANGle-AXIS METHOD///////////////////////////////////////////////
// get a angle and a axis from two vectors
// **input
// src-- vector
// dst-- vector

// **output
// axis-- the rotation axis
// angle-- the rotation angle
float angleAxisFromDirections(Vec3 & src, Vec3& dest, Vec3 & axis);



// 11/17/2014 : add functions


///////////////////////////////////////////COMPAREPAIRFLOATGREAT//////////////////////////////////////
bool comparePairFloatGreat(pair<int,float> p1, pair<int, float> p2);

///////////////////////////////////////////COMPAREPAIRFLOATLESS///////////////////////////////////////
bool comparePairFloatLess(pair<int,float> p1, pair<int, float> p2);

///////////////////////////////////////////COMPAREPAIRINTGREAT///////////////////////////////////////
bool comparePairIntGreat(pair<int, int>pair1, pair<int,int >pair2);

///////////////////////////////////////////COMPAREPAIRFLOATLESS///////////////////////////////////////
bool comparePairIntLess(pair<int, int>pair1, pair<int,int >pair2);



////////////////////////////////////MEANDISTANCE/////////////////////////////////////////////////////
// compute the mean distances for the points. For each point compute the distances between its
// K neighbours, and then the whole mean distances is obtained by computing the mean of all.
// **input
// points-- input points
float meanDistance(vector<Point> &points);



////////////////////////////////////MEANDISTANCE/////////////////////////////////////////////////////
// compute the mean distances for the points. For each point compute the distances between its
// K neighbours, and then the whole mean distances is obtained by computing the mean of all.
// Another version
// **input
// points-- input points
float meanDistance(QVector<Point> &points);





////////////////////////////////////KNNNEIGHBOURS////////////////////////////////////////////////////
// compute the K nearest neighbours from points for each query
// **input
// Knn-- number of nearest neighbours
// querys-- querys to find neighbours for
// points-- where the neighbours are in
// distances-- the distances betweene each pair of query and its neighbour
vector<vector<int> > knnNeighbours(int Knn, vector<Vec3> &querys,
                                   vector<Vec3> &points, vector<vector<float> > &distances);




/////////////////////////////////////////////////////////////////////////////////////////////////////
// compute the K neareast neighbours of the clusters
// **input
// Knn-- number of the nearest neighbours
// clusters--  input clusters to process
// cluster_distances-- distances between the neareast neigbours
// **return
// indices of K nearest neigbours of each cluster

vector<vector<int> > knnClusterNeighbours(int Knn, vector<vector<Point> >& clusters,
                                          vector<vector<float> >&cluster_distances );





//////////////////////////////////////BILATERALFILTERNOMAL////////////////////////////////////////////
// conduct bilateral filter on normals
// **input
// points-- containes normals
// window-- the window for bilateral filter
// angle-- the minimum angle for filtering
void bilateralFilterNormal(vector<Point> &points, float window, float angle);





/////////////////////////////////////BIALTERALFILTERPOSITION/////////////////////////////////////////
// conduct bilater filter on positions
// **input
// points--input points with normals, rgbs, normals
// window-- the window for bilater filtering
// angle-- the angle thresh for filtering
void bialteralFilterPosition(vector<Point> &points, float window, float angle);





//////////////////////////////////////LEASTSQUAREFILTTINGLINE///////////////////////////
// fitting a line on data by the least square method
// ** input
// pts-- input points to fit a line
vector<float> leastSquareFittingLine(vector<Point> pts);





///////////////////////////////////////EMCLUSTERING///////////////////////////////////////////////////
// conduct an EM algorithm on normals implemented in OPENCV
// **input
// samples-- the input samples
// nCluster-- number of clusters to obtain
// centers--  centers of the clusters
// weights-- weights of the clusters, large clusters are assigned large weights
// labels--  labels of each samples, the label indicated which cluster the sample is belong to
void EMClustering(const vector<Vec3> &samples, int nCluster,
                  vector<Vec3> &centers, vector<float> &weights,vector<int> &labels);




////////////////////////////////////////CENTER MERGING////////////////////////////////////////////////
// merging the clusters obtained from EMClustering, similar clusters will be merged. It is implemented
// through graph
// **input
// input-- the input clusters centers
// input_weight-- the weights for input clusters
// input_labels-- the input labels for each point
// output -- the new clusters centers after merging
// output_labels-- new labels for each point
// angle_thresh-- angle thresh for merging cluters
void centersMerging(const vector<Vec3> &input, const vector< float>&input_weights, vector<int>input_labels,
                       vector<Vec3> &output, vector< float>& output_weights, vector<int> &output_labels, float angle_thresh);





///////////////////////////////////////////LINERELATIONSHAPE///////////////////////////////////////////
// compute the relationshape between two lines(Parallel,  Intersect or Colinear )
// **input
// line0 -- the first line
// line1 -- the second line
// TR    -- the threshold for judging
// ** return
// LineType ( Parallel, Intersect or Colinear)
LineType lineRelationShape(vector<float>&line0,vector<float>&line1, float TR);





////////////////////////////////////////////INTERSECTION//////////////////////////////////////////////
// calculate the intersectio of two lines
// **input
// line0-- the first line
// line1-- the second line
// **return
// Point the intersecting point of two lines
Point intersection(vector<float>&line0, vector<float>&line1);



////////////////////////////////////////////ROTATIONMATRIXFROMANGLEAXIS////////////////////////////////
// get a rotation matrix from an angle and an axis
// **input
// axis-- an axis
// angle-- an angle
// **output
// mat-- a rotation matrix that rotate an object along axis by angle.
void rotationMatrixFromAngleAxis(Vec3& axis, float angle, cv::Mat & mat);


// 11/21/2014 add functions

////////////////////////////////////////////COVERTOQIMAGE////////////////////////////////////////////////
// convert a cv::Mat image to QImage
// **input
// mat-- an image in opencv structure

// **return @
//   -- an image in Qt image structure
QImage convertToQImage(cv::Mat_<cv::Vec3b> &mat);


//////////////////////////////////////////CONVERQIMAGETOMAT//////////////////////////////////////////////
// convert an QImage to cv::Mat
// **input
// img_qt-- an image in qt structure
// **output
// img_cv-- an image in cv::Mat structure
void convertQImageToMat(QImage & img_qt, cv::Mat_<cv::Vec3b> &img_cv);





#endif // SW_FUNCTIONS_H
