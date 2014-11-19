
#include"sw_regionGrowing.h"

#include"numeric" // accumulate
#include<opencv2/opencv.hpp>
#include"algorithm"

using namespace cv;
using namespace cv::flann;

using namespace SW;


//--------------------------------------------------neighbours--------------------------------------------//
void RegionGrowing::neighbours(uint knn)
{
    neighbrs_.clear();

    SearchParams params;

    cv::Mat features(pts_.size(), 2, CV_32FC1);
    Mat indices(pts_.size(), 2, CV_32SC1);
    Mat dists;

    // features in 2D space
    for(int i=0; i<pts_.size(); i++)
    {
        features.at<float>(i, 0) = pts_[i].x;
        features.at<float>(i, 1) = pts_[i].z;
    }

    Index FLANN(features, KDTreeIndexParams(4));

    // K neareast neighbours
    knn = knn> pts_.size()? pts_.size()-1: knn;

    // Knn seraching
    FLANN.knnSearch(features,indices, dists, knn,params);

    // results neighbours
    for(int i=0; i< indices.rows; i++)
    {
        vector<uint> neighs;
        for(int j=0; j< indices.cols; j++)
        {
            neighs.push_back(indices.at<int>(i, j));
        }
        neighbrs_.push_back(neighs);
    }

    features.release();
    indices.release();
    dists.release();
}



//---------------------------------------------------regionGrowing----------------------------------------//
void RegionGrowing::regionGrowing()
{
    vector<uint> used;
    vector<uint> labels;

    used.resize(pts_.size(), 0);
    labels.resize(pts_.size(), -1);

    int pt_id = 0;
    int cluster_id =0;
    while(pt_id < pts_.size())
    {
        deque<int> seeds;

        if(used[pt_id]==1)
        {
            pt_id++;
            continue;
        }

        // 1. select a seed point
        seeds.push_back(pt_id);
        used[pt_id] = 1;
        labels[pt_id] = cluster_id;

        pt_id++;

        float max_bound = -numeric_limits<float>::max();
        float min_bound =  numeric_limits<float>::max();
        float pro_tmp;

        // 2. growing
        while(seeds.size()>0)
        {
            int seed_id =  seeds[0];
            seeds.pop_front();

            // normal of the seed point
            Vec3 normal_s(pts_[seed_id].normal_x,
                          pts_[seed_id].normal_y,
                          pts_[seed_id].normal_z);
            normal_s.normalize();

            // position of the seed point
            Vec3 pos_s(pts_[seed_id].x, 0, pts_[seed_id].z);

            pro_tmp = normal_s *  pos_s;
            if(pro_tmp> max_bound) max_bound = pro_tmp;
            if(pro_tmp< min_bound) min_bound = pro_tmp;


            // indices of neighbours
            vector<uint>neighbrs = neighbrs_[seed_id];

            for(int i=0; i<neighbrs.size(); i++)
            {
                int n_id = neighbrs[i];
                if(n_id<0||n_id> pts_.size()) continue;
                if(n_id == seed_id) continue;
                if(used[n_id]==1)continue;

                // normal of neighbouring points
                Vec3 normal_n(pts_[n_id].normal_x,
                              pts_[n_id].normal_y,
                              pts_[n_id].normal_z);
                normal_n.normalize();

                // position of neighbouring points
                Vec3 pos_n(pts_[n_id].x,0, pts_[n_id].z);

                //-------------------------criterion 1: angle between two normals------------------//
                float product  = normal_n * normal_s;
                product = min(product, (float)0.999);
                product = max(product, -(float)0.999);
                float angle = acos(product)*180/3.1415;
                if(angle> angle_thresh_)continue;   // 阈值1 判断角度


                //-------------------------criterion 2: the distance between two points--------------//
                Vec3 diff = pos_n - pos_s;
                if(diff.norm()> 2*max_dist_) continue;  //阈值2 判断近邻的距离

                float max_bound_tmp = max_bound;
                float min_bound_tmp = min_bound;


                //-------------------------criterion 3: the width of the line
                pro_tmp = normal_n *  pos_n;
                if(pro_tmp> max_bound) max_bound =pro_tmp;
                if(pro_tmp< min_bound) min_bound =pro_tmp;

                if(max_bound - min_bound > max_width_)   // 阈值3 判断直线的宽度
                {
                    max_bound = max_bound_tmp;
                    min_bound = min_bound_tmp;

                    continue;
                }

                seeds.push_back(n_id);
                used[n_id] =1;
                labels[n_id] =cluster_id;
            }
        }

        cluster_id ++;
       // if(cluster_id==1) break;

    }

    uint nCluster = cluster_id;
    vector<uint>cluster;
    clusters_.resize(nCluster, cluster);

    for(int i=0; i<nCluster; i++)
    {
        cluster.clear();
        for(int j=0; j<labels.size(); j++)
        {
            if(labels[j]== i)
            {
                cluster.push_back(j);
            }
        }
        clusters_[i].swap(cluster);

    }
}
