#ifndef SW_CGAL_MESH_PROCESSING_H
#define SW_CGAL_MESH_PROCESSING_H

#include"sw_dataType.h"


#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include<CGAL/Polygon_with_holes_2.h>
#include<CGAL/Polygon_set_2.h>


typedef CGAL::Exact_predicates_exact_constructions_kernel        Kernel_e;
typedef CGAL::Polygon_2<Kernel_e>                                Polygon_2e;
typedef Kernel_e::Point_2                                        Point_2e;
typedef CGAL::Polygon_with_holes_2<Kernel_e>                     Polygon_with_holes_2e;
typedef CGAL::Polygon_set_2<Kernel_e>                            Polygon_set_2e;


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>


struct FaceInfo2
{
    FaceInfo2(){}
    int nesting_level;
    bool in_domain(){
        return nesting_level%2 == 1;
    }
};
typedef CGAL::Exact_predicates_inexact_constructions_kernel       Kernel;
typedef CGAL::Triangulation_vertex_base_2<Kernel>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,Kernel>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag>  CDT;
//typedef CGAL::Constrained_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Point                                                CDT_Point;
typedef CGAL::Polygon_2<Kernel>                                        Polygon_2;
typedef Kernel::Point_2                                        Point_2;
typedef CDT::Vertex_handle Vertex_handle;


#include<stdlib.h>
#include<iostream>
#include<vector>
#include<map>
#include<list>

using namespace std;

////////////////////////////////////////GETOUTERBOUNDARYFROMTRIANS//////////////////////////////////////////////////////
//get a boundary of a polygon which is the union of a set of triangulations
// operated in 2D space
vector< vector<PointXY> > getOuterBoundaryFromTrians(const vector<vector<PointXY> > & facets);

#endif // SW_CGAL_MESH_PROCESSING_H
