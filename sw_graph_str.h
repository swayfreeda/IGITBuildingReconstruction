
#ifndef SW_GRAPH_STR_H
#define SW_GRAPH_STR_H
#include"sw_alpha_expansion.h"
#include"sw_dataType.h"
#include "stdlib.h" 
#include<vector>
#include<string>
#include<iostream>
using namespace std;


/*********************************class alpha expansion string ********************************/
class alpha_expansion_str: public alpha_expansion_base<string>
{
public:
alpha_expansion_str():alpha_expansion_base(){}
alpha_expansion_str(vector <string> & samples, vector<string>& centers):alpha_expansion_base(samples, centers)
{
}


double data_term(int id, int label);

double smooth_term(int xid, int yid, int xlabel, int ylabel);

void setSimThresh(float thresh){sim_thresh_ = thresh;}


public:

float sim_thresh_;

};

#endif
