#include"sw_cgal_mesh_processing.h"

//----------------------------------------getBoundary-------------------------------------------------------------//
// get boundary points from a polygon
template<class Kernel_e, class Container>
vector<PointXY> getBoundary (const CGAL::Polygon_2<Kernel_e, Container>& P)
{
    vector<PointXY> boundary;
    typename CGAL::Polygon_2<Kernel_e, Container>::Vertex_const_iterator  vit;
    for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    {
    ostringstream out;
    out<<vit->x()<<" " <<vit->y()<<endl;// space doesn't affect the results and will be ig by defalut

    istringstream in(out.str());
        PointXY pt;
    in>>pt.x>>pt.y;

 //       cout<<"( "<<vit->x()<<", "<<vit->y()<<" )---->";
 //       cout<<"( "<<pt.x<< ", "<< pt.y<<") "<<endl;

    boundary.push_back(pt);;
    }
  return boundary;
}



//--------------------------------------------getOuterBoundary-----------------------------------------------------//
//get outer boundary of a polygon with holes
template<class Kernel_e, class Container>
vector<PointXY>  getOuterBoundary(const CGAL::Polygon_with_holes_2<Kernel_e, Container> & pwh)
{
    //get outer boundary
    if (! pwh.is_unbounded()) {
        vector<PointXY> outer_boundary = getBoundary (pwh.outer_boundary());
        return outer_boundary;
    }
}


//--------------------------------------------getOuterBoundaryFromTrians-------------------------------------------//
vector< vector<PointXY> >  getOuterBoundaryFromTrians(const vector<vector<PointXY> > & facets)
{
    Polygon_set_2e S;
    {
    if(facets.size()>0)
    {
         Polygon_2e Trian;
         for(int j=0; j< facets[0].size(); j++)
            {
                  Trian.push_back(Point_2e(facets[0][j].x, facets[0][j].y));
            }
             S.insert(Trian);
    }

    }
    for(int i=1; i< facets.size(); i++)
    {
    Polygon_2e Trian;
    for(int j=0; j< facets[i].size(); j++)
    {
            Trian.push_back(Point_2e(facets[i][j].x, facets[i][j].y));
    }
       S.join(Trian);
    }

    // S may contain several compomemts (each component is a polygon with holes)
    std::list<Polygon_with_holes_2e> res;
    std::list<Polygon_with_holes_2e>::const_iterator it;
    S.polygons_with_holes (std::back_inserter (res));

    vector<vector<PointXY> > outerBoundarys;
    // each component
    for (it = res.begin(); it != res.end(); ++it) {
       vector<PointXY>outerBoundary = getOuterBoundary(*it);
       outerBoundarys.push_back(outerBoundary);
    }
    return outerBoundarys;
}
