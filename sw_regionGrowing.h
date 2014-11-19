#ifndef REGIONGROWING_H
#define REGIONGROWING_H

#include "sw_dataType.h"
#include"stdlib.h"
#include<vector>
#include"cmath"
using namespace std;

namespace SW{


typedef PointXYZRGBNormal Point;


//////////////////////////////////////////////////CLASS REGIONGROWING/////////////////////////////////////
class RegionGrowing
{
public:

    // constructor
    RegionGrowing(){}

    // constructor
    RegionGrowing(vector<Point>& pts): pts_(pts){ }

    // set input points
    void setInput(vector<Point>&pts)
    {
        pts_ = pts;
    }

    // set K nearest neighbours of each point
    void setNeighbours(vector<vector<uint> >&neigh)
    {
        neighbrs_ = neigh;
    }

    // compute K neareast neighbours in 2D sapce
    void neighbours(uint knn);

    // main algorithms
    void regionGrowing();

    // set the angle threh between two points
    void set_angle_thresh(float thresh){ angle_thresh_ = thresh;}

    // set the max distance between two points
    void set_max_dist(float thresh){max_dist_ = thresh; }

    // set the max width of the two points
    void set_max_width(float thresh){max_width_ = thresh;}


public:

    vector<Point>  pts_;

    vector<vector<uint> >neighbrs_;

    vector<vector<uint> >clusters_;

    // the angle thresh between two points
    float angle_thresh_;

    // the maximum distance between two points
    float max_dist_;

    // the maximum width of the line. The width of the line is controlled in a
    // scope of this threshold.
    float max_width_;

};

}


#endif // REGIONGROWING_H
