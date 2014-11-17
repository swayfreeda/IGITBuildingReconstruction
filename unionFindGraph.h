#ifndef UNIONFINDGRAPH_H
#define UNIONFINDGRAPH_H
#include"stdlib.h"
#include<vector>
#include<map>
#include<set>

using namespace std;


class UnionFind
{
public:
    UnionFind(){
        eos_ = -1;// 节点为-1 表示该节点为根节点
        nSets_ = 0;
    }

    void construct(int N){ P_.resize(N, -1);
      nSets_ = N;
    }

    // 获取i的父亲节点
    int  getParent(int i) { return P_[i]; }

    // 设置i的父节点为j
    void setParent(int i, int j){ P_[i] = j; }

    // 返回联通区域的个数
    int getSetNum(){return nSets_;}

    // 返回元素的个数
    int getElementNum(){return P_.size(); }

    // 返回第i个元素所在的联通区域的根节点
    int findSetRoot(int i);

    // 将i和j 设置成相同的联通区域
    void setUnion(int i, int j);

    // 获取第k个节点的根节点
    int getKthSetRoot(int k);

    // 返回与i属于同一个联通区域的所有的节点
    vector<int> getSameSetElements(int iter);

    // 返回第k个联通区域的所有的元素
    vector<int> getKthSetElements(int k);



public:

    vector<int> P_;
    int eos_;

    int nSets_;
    int nCurPos_;
};
#endif // UNIONFINDGRAPH_H
