#include"sw_functions.h"
#include"unionFindGraph.h"

#include<opencv2/opencv.hpp>
#include "opencv2/legacy/legacy.hpp"

#include"numeric" // std functions such as "accumulate"

//--------------------------------------Gaussian---------------------------------------------------------------//
float Gaussian(float x, float mu, float sigma)
{
    float u = x - mu;

    float v = u*u/(sigma* sigma);

    return exp(-v);
}





//--------------------------------------normalSimlarity--------------------------------------------------------//
float normalSimlaity(const Vec3 & n0, const Vec3 n1, float sigma)
{
    float angle = sigma* 180/3.1416;

    float u = 1 - n0* n1;
    float v =  u/(1 - cos(angle));

    return exp(- v*v);
}





//---------------------------------------angleAxisFromDirections-----------------------------------------------//
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





//----------------------------------------meanDistance--------------------------------------------------------//
float meanDistance(vector<Point> &points)
{
    int Knn = 5 +1;

    cv::Mat samples(points.size(), 3, CV_32FC1);
    cv::Mat query(points.size(), 3, CV_32FC1);

    cv::Mat dists(points.size(), Knn, CV_64FC1);
    cv::Mat indices;

    for(int i=0; i<points.size(); i++)
    {
        samples.at<float>(i, 0) = points[i].x;
        samples.at<float>(i, 1) = points[i].y;
        samples.at<float>(i, 2) = points[i].z;

        query.at<float>(i, 0) = points[i].x;
        query.at<float>(i, 1) = points[i].y;
        query.at<float>(i, 2) = points[i].z;

    }
    cv::flann::KDTreeIndexParams params(4);
    cv::flann::Index neighbours_search(samples, params);
    neighbours_search.knnSearch(query, indices, dists, Knn, cv::flann::SearchParams(128));

    float distance;
    float num = 0;
    for(int i=0; i< dists.rows; i++)
    {
        for(int j=1; j< dists.cols; j++)
        {
            distance += sqrt( dists.at<float>(i, j));
            num ++;
        }
    }

    return distance/num;

}





//----------------------------------------knnNeighbours-------------------------------------------------------//
vector<vector<int> > knnNeighbours(int Knn, vector<Vec3> &querys,
                                   vector<Vec3> &points, vector<vector<float> > &distances)
{
    vector<vector<int> > neighbours;
    vector<int> nhbrs;
    nhbrs.resize(Knn);
    neighbours.resize(querys.size(), nhbrs);

    vector<float> ndists;
    ndists.resize(Knn, numeric_limits<float> ::max());
    distances.resize(querys.size(), ndists);

    cv::Mat samples(points.size(),3,CV_32FC1);
    cv::Mat query(querys.size(),3,CV_32FC1);
    cv::Mat dists;
    cv::Mat indices;

    for(int i=0; i<points.size(); i++)
    {
        samples.at<float>(i, 0) = points[i].x_;
        samples.at<float>(i, 1) = points[i].y_;
        samples.at<float>(i, 2) = points[i].z_;
    }

    for(int i=0; i< querys.size(); i++)
    {
        query.at<float>(i, 0) = querys[i].x_;
        query.at<float>(i, 1) = querys[i].y_;
        query.at<float>(i, 2) = querys[i].z_;
    }


    cv::flann::KDTreeIndexParams params(4);
    cv::flann::Index neighbours_search(samples, params);
    neighbours_search.knnSearch(query, indices, dists, Knn, cv::flann::SearchParams(128));

    for(int i=0; i< indices.rows; i++)
    {
        for(int j=0; j< indices.cols; j++)
        {
            neighbours[i][j] = indices.at<int>(i, j);
        }
    }

    for(int i=0; i< dists.rows; i++)
    {
        for(int j=0; j< dists.cols; j++)
        {
            distances[i][j] = (float)dists.at<float>(i, j);
        }
    }
    samples.release();
    query.release();
    dists.release();
    indices.release();

    return neighbours;
}




//----------------------------------------bilateralFilterNormal----------------------------------------------//
void bilateralFilterNormal(vector<Point> &points, float window, float angle)
{
    vector<Vec3> samples;
    samples.resize(points.size());
    for(int i=0; i< points.size(); i++)
    {
        samples[i].x_ = points[i].x;
        samples[i].y_ = points[i].y;
        samples[i].z_ = points[i].z;
    }

    int Knn =points.size()< 30? points.size(): 30;
    vector<vector<float> > dist;
    vector<vector<int> > neighbours = knnNeighbours(Knn, samples, samples, dist);


    // results normals
    vector<Vec3> filtered_normals;
    filtered_normals.resize(points.size());

    int iter = 0;

    while(iter < 3)
    {
        // for each point
        for(int i=0; i< points.size(); i++)
        {
            Vec3 pos0(points[i].x, points[i].y, points[i].z);
            Vec3 normal0(points[i].normal_x, points[i].normal_y, points[i].normal_z);

            // neighbours--compute the weights of each neighbours
            vector< float > weights;
            weights.resize(neighbours[i].size(), 0);
            for(int j = 0; j< neighbours[i].size(); j++)
            {
                int index = neighbours[i][j];

                if (index == i) continue;
                Vec3 pos1(points[index].x, points[index].y, points[index].z);
                Vec3 normal1(points[index].normal_x,
                             points[index].normal_y,
                             points[index].normal_z);

                Vec3 diff = pos1 - pos0;
                float r = diff.norm();

                // two weights are computed, one is from the distance and the other is
                // from normal
                float weight0 =  Gaussian(r, 0, window);
                float weight1 = normalSimlaity(normal0, normal1, angle);

                weights[j] = weight0 * weight1;
            }


            float Wsum = accumulate(weights.begin(), weights.end(), 0.0);
            if(Wsum == 0)continue;

            // compute the new normals
            filtered_normals[i] = Vec3(0, 0, 0);
            for(int j = 0; j< neighbours[i].size(); j++)
            {
                int index = neighbours[i][j];

                filtered_normals[i] =  filtered_normals[i]
                        + Vec3(points[index].normal_x,
                               points[index].normal_y,
                               points[index].normal_z)* weights[j]/Wsum;
            }
            filtered_normals[i].normalize();
        }

        for(int i=0; i< points.size(); i++)
        {
            points[i].normal_x = filtered_normals[i].x_;
            points[i].normal_y = filtered_normals[i].y_;
            points[i].normal_z = filtered_normals[i].z_;
        }

        iter ++;
    }
}





//----------------------------------------EMClustering--------------------------------------------------------//
void EMClustering(const vector<Vec3> &samples, int nCluster,
                  vector<Vec3> &centers, vector<float> &weights,vector<int> &labels)
{
    labels.clear();
    weights.clear();
    centers.clear();

    labels.resize(samples.size());
    weights.resize(nCluster);
    centers.resize(nCluster);

    cv::Mat samplesM(samples.size(), 3, CV_32FC1);
    cv::Mat labelsM(samples.size(), 1, CV_32SC1);

    for(int i=0; i< samples.size(); i++)
    {
        samplesM.at<float>(i, 0) = samples[i].x_;
        samplesM.at<float>(i, 1) = samples[i].y_;
        samplesM.at<float>(i, 2) = samples[i].z_;
    }
    CvEM em_model;
    CvEMParams params;
    params.covs      = NULL;
    params.means     = NULL;
    params.weights   = NULL;
    params.probs     = NULL;
    params.nclusters = nCluster;
    params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
    params.start_step         = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 300;
    params.term_crit.epsilon  = 0.01;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

    em_model.train( samplesM, cv::Mat(), params, &labelsM);

    cv:: Mat meansM = em_model.getMeans();
    //  cout<<meansM<<endl;

    cv::Mat weightsM = em_model.getWeights();
    //    cout<< weightsM <<endl;

    for(int i=0; i< nCluster; i++)
    {
        weights[i] =  weightsM.at<double>(i);
    }

    for(int i=0; i< centers.size(); i++)
    {
        for(int j=0; j< 3; j++)
        {
            centers[i][j] = meansM.at<double>(i, j);
        }
        centers[i].normalize();
    }

    for(int i=0; i< samples.size(); i++)
    {
        labels[i] = labelsM.at<int>(i);
    }
}




//----------------------------------------centersMerging------------------------------------------------------//
void centersMerging(const vector<Vec3> &input, const vector< float>&input_weights,
                    vector<int>input_labels,  vector<Vec3> &output, vector< float>& output_weights,
                    vector<int> &output_labels, float angle_thresh)
{
    output.clear();
    output_weights.clear();
    output_labels.resize(input_labels.size(), -1);

    UnionFind sg;
    sg.construct(input.size());

    for(int i=0; i< input.size(); i++)
    {

        for(int j=i+1; j< input.size(); j++)
        {
            Vec3 v0 = input[i];
            Vec3 v1 = input[j];

            v0.normalize();
            v1.normalize();

            float u = v0* v1;
            float angle = acos(u)* 180/3.1415;

            if(angle< angle_thresh)
            {
                sg.setUnion(i, j);
            }
        }
    }

    int setNum = sg.getSetNum();
    vector<vector<int> > clusters;
    for(int i =0; i<setNum; i++)
    {
        vector<int> elemts = sg.getKthSetElements(i);
        clusters.push_back(elemts);
    }

    // 输出centers
    for(int i=0; i< clusters.size(); i++)
    {
        // 计算权重
        float Wsum = 0.0;
        for(int j=0; j< clusters[i].size(); j++)
        {
            int id = clusters[i][j];
            Wsum += input_weights[id];
        }

        // 加权合并中心，形成新的中心
        Vec3 center(0, 0, 0);
        for(int j=0; j< clusters[i].size(); j++)
        {
            int id = clusters[i][j];
            center  = center + input[id]*input_weights[id]/Wsum;
        }

        output_weights.push_back(Wsum);
        output.push_back(center);

        for(int j=0; j< clusters[i].size(); j++)
        {
            int id = clusters[i][j];
            // if(i == id) continue;

            for(int k=0; k< input_labels.size(); k++)
            {
                if(input_labels[k] == id)
                    output_labels[k] = i;
            }
        }

    }

}




//----------------------------------------comparePairFloatGreat-----------------------------------------------//
bool comparePairFloatGreat(pair<int,float> p1, pair<int, float> p2)
{
    return p1.second>p2.second;
}




//----------------------------------------comparePairFloatLess------------------------------------------------//
bool comparePairFloatLess(pair<int,float> p1, pair<int, float> p2)
{
    return p1.second<p2.second;
}



//----------------------------------------comparePairIntGreat--------------------------------------------------//
bool comparePairIntGreat(pair<int, int>pair1, pair<int,int >pair2)
{
    return pair1.second> pair2.second;
}



//----------------------------------------comparePairIntLess---------------------------------------------------//
bool comparePairIntLess(pair<int, int>pair1, pair<int,int >pair2)
{
   return pair1.second> pair2.second;
}


//-----------------------------------------rotationMatrixFromAngleAxis------------------------------------------//
void rotationMatrixFromAngleAxis(Vec3& axis, float angle, cv::Mat & mat)
{
    cv::Mat W(3,3,CV_32FC1);
    W.setTo(0);
    W.at<float>(0,1) = -axis.z_;
    W.at<float>(1,0) = axis.z_;
    W.at<float>(0,2) = axis.y_;
    W.at<float>(2,0) = -axis.y_;
    W.at<float>(1,2) = -axis.x_;
    W.at<float>(2,1) = axis.x_;

    cv::Mat I = cv::Mat::eye(3,3,CV_32FC1);

    //构造旋转矩阵
    mat.create(3, 3, CV_32FC1);
    mat = I + W*sin(angle)+ W*W*(1- cos(angle));

    //    cout<<"Axis: "<< Axis<<endl;
    //    cout<<"Angle: "<< angle<<endl;
    //    cout<<"W: "<<W<<endl;
    //    cout<<"result: "<< result<<endl;

    W.release();
    I.release();
}


