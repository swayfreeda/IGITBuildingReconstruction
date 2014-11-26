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
// get a boundary of a polygon which is the union of a set of triangulations
// operated in 2D space
vector< vector<PointXY> > getOuterBoundaryFromTrians(const vector<vector<PointXY> > & facets);


// 11/25/2014 add functions

////////////////////////////////////////ISCOUNTERCLOCKWISE///////////////////////////////////////////////////////////////
// check whether the boundary is counterclockwise
bool isCounterClockWise(const vector<PointXY> & boundary);



//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains(CDT& cdt);



////////////////////////////////////////INSERT_POLYGON////////////////////////////////////////////////////////////////////
//constrained triangulations used this functions
void insert_polygon(CDT& cdt,const Polygon_2& polygon);




////////////////////////////////////////GETVERTICESANDFACETS//////////////////////////////////////////////////////////////
vector<vector<Vec3> > getVerticesAndFacets(const CDT&  cdt, const vector<vector<Vec3> > & boundary3D);



////////////////////////////////////////CONSTRIANDTRIANGULATIONS//////////////////////////////////////////////////////////
vector<vector<Vec3> > constrained_triangulation(const vector<vector<PointXY > > & boundarys2D,const vector<vector<Vec3> > &boundarys3D);




/////////////////////////////////////////TRIANGULATION_ON_EDIT_PLANE//////////////////////////////////////////////////////
vector<vector<Vec3> > triangulation_on_edit_plane(vector<Vec3> & boundary);

#endif // SW_CGAL_MESH_PROCESSING_H
