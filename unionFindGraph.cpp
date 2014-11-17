
#include"unionFindGraph.h"
#include"cmath"

// 返回第i个元素所在的联通区域的根节点
int UnionFind::findSetRoot(int i)
{
   int index = i;
   while(getParent(index) != eos_)
   {
     index = getParent(index);
   }

   return index;
}

// 将i和j 设置成相同的联通区域
void UnionFind::setUnion(int i, int j)
{
   int rooti = findSetRoot(i);
   int rootj = findSetRoot(j);

   if(rooti< rootj)
   {
      setParent(rootj, rooti);
      nSets_ --;
   }
   if(rooti> rootj) // 这里不应该存在相等的情况，因为相等表示i,j属于同一个联通区域，再进行设置的话
                   // 将根节点的父节点设置成了本身，而不是-1
   {
     setParent(rooti, rootj);
     nSets_--;
   }

}

// 获取第k个节点的根节点
int UnionFind::getKthSetRoot(int k)
{
   int iter = 0;

   int rootid = -1;

   int elementNum = getElementNum();

   for(int i=0; i< elementNum; i++)
   {
       if(getParent(i) == eos_)
       {
           if(iter == k)
           {
             rootid = i;
             break;
           }

          iter++;
       }
   }
   return rootid;
}

// 返回与i属于同一个联通区域的所有的节点// 第一个元素是根节点
vector<int> UnionFind::getSameSetElements(int iter)
{
   vector<int> elemts;

   int rooti = findSetRoot(iter);

   int elementsNum = getElementNum();

   elemts.push_back(rooti);
   for(int i=0; i< elementsNum; i++)
   {

       if(i!= rooti && findSetRoot(i) == rooti)
       {
          elemts.push_back(i);
       }
   }
   return elemts;
}

// 返回第k个联通区域的所有的元素
vector<int> UnionFind::getKthSetElements(int k)
{
   int rooti = getKthSetRoot(k);

   vector<int> elements = getSameSetElements(rooti);

   return elements;

}
