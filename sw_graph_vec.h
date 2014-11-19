#ifndef GRAPH_VEC_H
#define GRAPH_VEC_H

#include"sw_alpha_expansion.h"

/*********************************** class alpha expansion vec3 *****************************/
class alpha_expansion_vec: public alpha_expansion_base<Vec3>
{
 public:
alpha_expansion_vec():alpha_expansion_base(){}
alpha_expansion_vec(vector <Vec3> & samples, vector<Vec3>& centers):alpha_expansion_base(samples, centers){
    }

void computeKnnNeighbours(int Knn);

double data_term(int id, int label);

double smooth_term(int xid, int yid, int xlabel, int ylabel);

void setAngleThresh(float angle){angle_thresh_ = angle;}

public:

float angle_thresh_;
};

#endif
