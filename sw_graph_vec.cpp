#include "sw_graph_vec.h"

/*********************************** class members *****************************/
////////////////////////////////////////////////////////////////////////////////////
void  alpha_expansion_vec:: computeKnnNeighbours(int Knn)
{
  neighbrs_.clear();
  vector<vector<float> > dist;
  neighbrs_ =  knnNeighbours(Knn, input_, input_, dist);
}
///////////////////////////////////////////////////////////////////////////////////
double alpha_expansion_vec:: data_term(int id, int label)
{
   Vec3 normal = input_[id];
    Vec3 center = centers_[label];

    normal.normalize();
    center.normalize();

    double u = normal * center;

    u = min(u, 1.0);
    u = max(-1.0, u);  //abs(u)> 1 则返回NAN

    return acos(u)* 180/ 3.1416;
}

///////////////////////////////////////////////////////////////////////////////////
double alpha_expansion_vec:: smooth_term(int xid, int yid, int xlabel, int ylabel)
{
   double E = 0;

   if(xlabel != ylabel)
   {
       Vec3 normal0 = input_[xid];
       Vec3 normal1 = input_[yid];

       normal0.normalize();
       normal1.normalize();

       double u = normal0 * normal1;

       u = min(u, (double)1.0);
       u = max((double)-1.0, u);  //abs(u)> 1 则返回NAN

       double angle = acos(u)* 180/ 3.1416;

       if(angle < angle_thresh_)
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

