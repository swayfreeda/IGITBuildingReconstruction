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



//-----------------------------------------isCounterClockWise---------------------------------------------------------//
bool isCounterClockWise(const vector<PointXY> &boundary)
{
    Polygon_2 polygon;
    for(int i=0; i< boundary.size(); i++)
    {
        polygon.push_back(Point_2(boundary[i].x, boundary[i].y));
    }

    CGAL::Orientation orient = polygon.orientation();

    if(orient == CGAL::COUNTERCLOCKWISE)
    {
        return true;
    }
    else{
        return false;
    }
}





//-------------------------------------------mark_domains-------------------------------------------------------------//
void mark_domains(CDT& ct,
                  CDT::Face_handle start,
                  int index,
                  std::list<CDT::Edge>& border )
{
    if(start->info().nesting_level != -1){
        return;
    }
    std::list<CDT::Face_handle> queue;
    queue.push_back(start);
    while(! queue.empty()){
        CDT::Face_handle fh = queue.front();
        queue.pop_front();
        if(fh->info().nesting_level == -1){
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++){
                CDT::Edge e(fh,i);
                CDT::Face_handle n = fh->neighbor(i);
                if(n->info().nesting_level == -1){
                    if(ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}




//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
//------------------------------------------mark_domains---------------------------------------------------------------//
void mark_domains(CDT& cdt)
{
    for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }
    std::list<CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while(! border.empty()){
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1){
            mark_domains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}





//-----------------------------------------------inset_polygon-----------------------------------------------------------//
void insert_polygon(CDT& cdt,const Polygon_2& polygon){
    if ( polygon.is_empty() ) return;
    CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp11::prev(polygon.vertices_end()));
    for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin();
         vit!=polygon.vertices_end();++vit)
    {
        CDT::Vertex_handle vh=cdt.insert(*vit);
        cdt.insert_constraint(vh,v_prev);
        v_prev=vh;
    }
}



//----------------------------------------------getverticesandfacets------------------------------------------------------//
vector<vector<Vec3> > getVerticesAndFacets(const CDT&  cdt, const vector<vector<Vec3> > & boundarys3D )
{
    vector<Vec3> vertices;

    for(int i=0;i< boundarys3D.size(); i++)
    {
        int pt_num = boundarys3D[i].size();
        Vec3 p = boundarys3D[i][pt_num -1];
        vertices.push_back(p);

        for(int j=0; j< pt_num-1; j++)
        {
            Vec3 p = boundarys3D[i][j];
            vertices.push_back(p);
        }
    }
    // construct a query table
    std::map<Vertex_handle, int> query_table;
    int index =0;
    for(CDT::Finite_vertices_iterator vit = cdt.finite_vertices_begin();
        vit != cdt.finite_vertices_end(); vit++)
    {
        CDT_Point p = vit->point();
        //	cout<<"point: "<< p<<endl;

        query_table.insert(std::make_pair(vit, index));

        index++;
    }

    //for(int i=0; i< vertices.size(); i++)
    //{
    //    cout<<"( "<<vertices[i].x_<< ", "<< vertices[i].y_<<", "<< vertices[i].z_<<") "<<endl;
    //}
    vector<vector<Vec3> > all_facets;
    // write facet information
    for(CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit!= cdt.finite_faces_end(); fit++)
    {
        if( fit->info().in_domain() )
        {
            vector<Vec3> facet;
            for(int i = 0; i<3; i++)
            {
                Vertex_handle tmp = fit->vertex(i);
                if(query_table.count(tmp))
                {
                    int id = query_table[tmp];
                    facet.push_back(Vec3(vertices[id].x_, vertices[id].y_, vertices[id].z_));
                }
            }
            all_facets.push_back(facet);
        }
    }


    return all_facets;
}




//----------------------------------------------constrained_triagulations--------------------------------------------------//
vector<vector<Vec3> > constrained_triangulation(const vector<vector<PointXY> >& boundarys2D,
                                                const vector<vector<Vec3> > & boundarys3D)
{
    CDT cdt;
    for(int i=0; i< boundarys2D.size(); i++)
    {
        Polygon_2 polygon;
        for(int j=0; j< boundarys2D[i].size(); j++)
        {
            polygon.push_back(CDT_Point(boundarys2D[i][j].x, boundarys2D[i][j].y));
        }
        insert_polygon(cdt, polygon);
    }

    mark_domains(cdt);

    vector<vector<Vec3> > all_facets = getVerticesAndFacets(cdt, boundarys3D);
    return all_facets;
}



//-----------------------------------------------triangulation_on_edit_plane--------------------------------------------------//
vector<vector<Vec3> > triangulation_on_edit_plane(vector<Vec3> & boundary)
{
    Polygon_2  cgal_polygon;

    // project the points on to the plane
    for(int i=0;i< boundary.size(); i++)
    {
        Vec3 p = boundary[i];
        cgal_polygon.push_back(CDT_Point(p.x_, p.z_));
    }

    //Insert the polyons into a constrained triangulation
    CDT cdt;
    insert_polygon(cdt, cgal_polygon);
    //Mark facets that are inside the domain bounded by the polygon
    mark_domains(cdt);

    vector<vector<Vec3> > boundarys;
    boundarys.push_back(boundary);
    vector<vector<Vec3> > all_facets = getVerticesAndFacets(cdt, boundarys);
    return all_facets;

}
