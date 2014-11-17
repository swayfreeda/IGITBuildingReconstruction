#ifndef SW_ALPHAEXPANSION_H
#define SW_ALPHAEXPANSION_H
#include"energy.h"
#include"graph.h"
#include"sw_dataType.h"
#include"sw_functions.h"
#include "stdlib.h" 
#include<vector>
#include<string>
#include<iostream>
using namespace std;

///////////////////////////////////////CLASS ALPHA_EXPANSION_BASE///////////////////////////////////
// alpha expansion is used for optimization for graph cut problem, only if all the terms in the energy
// are regular
// At each step, a label is choosed, and a sub optimization problem is proposed by treating it as
// a two-labels assignment problem.


template<class T>
class alpha_expansion_base
{
public:

    // constructor
    alpha_expansion_base(){ E_ = 0;}
    alpha_expansion_base(vector <T> & samples,
                         vector <T>& centers):input_(samples),centers_(centers),E_(0){
        label_num_ = centers.size();
        sample_num_ = samples.size();
    }


    // set the label to be flipped
    void setAlpha(int alpha){ alpha_ = alpha;}

    // maxmum iter number
    void setMaxIterNum(int num){ max_iter_num_ = num;}

    // set the lambda
    void setLambda(float lambda ){lambda_ = lambda;}

    // set K nearest neighbours
    void setNeighbours(vector<vector<int> > &neighbrs){ neighbrs_ = neighbrs;}

    // compute K nearest neighbours
    virtual void computeKnnNeighbours(int Knn){}

    // compute the data term
    virtual double data_term(int id, int label){}

    // compute the smooth term
    virtual double smooth_term(int xid, int yid, int xlabel, int ylabel){}

    // compute the energy
    void  computeEnergy();

    // alpha expansion at current step, only once
    double expansion();

    // multi- expansions until convergence
    void optimization();// 通过 不同标签的 alpha-expansion 进行优化


public:

    vector<int> labels_;
    vector<vector<int> > neighbrs_;
    vector<T> input_;
    vector<T> centers_;

    int label_num_;
    int sample_num_;
    int max_iter_num_;

    double E_;
    double lambda_;
    int alpha_; // labels to be flipped
};


#endif // ALPHAEXPANSION_H
