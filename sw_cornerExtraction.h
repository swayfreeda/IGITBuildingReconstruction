#include"sw_dataType.h"
#include"sw_functions.h"
#include"sw_alpha_expansion.h"
#include"sw_regionGrowing.h"
#include"stdlib.h"
#include<iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/legacy/legacy.hpp"
using namespace std;
using namespace cv;

typedef PointXYZRGBNormal Point;

///////////////////////////////////////////CLASS EDGE////////////////////////////////////////////////////////
class edge
{
public:

    // comstructor
    edge() { i=0; j=0; }
    edge(int ii, int jj){i = ii; j = jj;}
    edge(edge &e) { i = e.i; j = e.j; }
    edge & operator =(const edge &e)
    {
        if(this!=&e)
        {
            i = e.i;
            j = e.j;
        }
        return *this;
    }
public:

    int i;
    int j;
};

///////////////////////////////////////////CLASS NODE////////////////////////////////////////
class Node{

public:
    Node(){ disabled_ = false; }
    Node(PointXYZRGBNormal & pt, int i){
        pt_ = pt;
        id_ = i;
        disabled_ = false;
    }

    void setDisabled(bool flag){ disabled_ = flag; }
    bool isDisabled(){  return disabled_; }

public:
    int id_; // index in the graph
    PointXYZRGBNormal pt_;
    bool disabled_;

};

////////////////////////////////////////////////////////////////////////////////////////////////
template<class Tp>
class Graph_link
{
public:
    // type definitions
    typedef map<int, Tp, less<int> > EdgeList;
    typedef vector<edge> edgeList;

    Graph_link(){ e.clear(); }

    //add node
    inline void addNode(Node & n)
    {
        nodes_.push_back(n);
    }

    // visit i-th node
    inline Node & node(int i)
    {
        for(vector<Node>::iterator it = nodes_.begin();
            it!= nodes_.end(); it++)
        {
            if(it->id_ == i)
                return(*it);
        }
    }

    // create a graph of nv nodes
    void makeGraph(int nv)
    {
        EdgeList e1;
        e.clear();
        e.resize(nv, e1);
    }

    // clear all the edges
    inline void clearEdges(){e.clear();}

    //clear all the nodes
    inline void clearNodes(){nodes_.clear(); }

    // get the degree of the node i. e.g. the number of edges that are connected to it
    int getNodeDegree(int i)
    {
       return e[i].size();
    }

    // return the value of edge connecte node i and node j
    Tp getEdge(int i, int j)
    {
        typename EdgeList::iterator iter;
         iter =  e[i].find(j);
         if(iter!= e[i].end())
         {
             return iter->second;
         }
         else
         {
             Tp tmp = (Tp)-1;
            return tmp;
         }
    }

    void insertDirectedEdge(int i, int j, Tp w)
    {
        e[i].insert(typename EdgeList::value_type(j, w));
    }

    // insert two edges between node i and node j
    void insertEdge(int i, int j, Tp w)
    {
        insertDirectedEdge(i, j, w);
        insertDirectedEdge(j, i, w);
    }

public:
    vector<EdgeList>e;
    vector<Node> nodes_;
};


//将直线连接成曲线
vector<vector<PointXYZRGBNormal> > linkLinesToCurves(vector<vector<PointXYZRGBNormal> > lines, float TR);

// 将曲线调整到主方向上
void curvesAdjustment(vector<vector<PointXYZRGBNormal> >& input_curves, vector<Vec3> & main_directions, float TR);

// 对曲线进行处理
vector<vector<PointXYZRGBNormal> >curvesProcessing(vector<vector<PointXYZRGBNormal> >& input_curves,float TR);

vector< vector<PointXYZRGBNormal> > cornerExtraction(vector<PointXYZRGBNormal > & input,
                                                float TR, float RG_TR, float Link_TR, vector<Vec3> &main_directions);

