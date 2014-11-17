#include"sw_graph_str.h"
#include"sw_codingEdit.h"

/********************************* class members ********************************/
//////////////////////////////////////////////////////////////////////////////////
double alpha_expansion_str:: data_term(int id, int label)
{
  string str1 = input_[id];
  string str2 = centers_[label];

  float dist = 1- similarity(str1, str2);
  return (double)dist;
}

/////////////////////////////////////////////////////////////////////////////////
double alpha_expansion_str::smooth_term(int xid, int yid, int xlabel, int ylabel)
{
  double E = 0;

   if(xlabel != ylabel)
   {
       string str1 = input_[xid];
       string str2 = input_[yid];
       
       double dist = (double) similarity(str1, str2);

       if(dist < sim_thresh_)
       {
          E += 3*lambda_;
       }
       else
       {
          E += lambda_;
       }
   }
   return E;
}



