#include"sw_dataType.h"
#include"sw_cgal_mesh_processing.h"

#include<GL/gl.h>
#include <limits>
#include<QPair>
#include<QSet>
//----------------------------------------operator + -----------------------------------------------------//
// 重载算数操作符号+
Vec3  operator +(const Vec3 & vl, const Vec3 & vr) //算数和关系操作符一般定义为非成员函数
{
    Vec3 tmp(vl);
    tmp.x_ += vr.x_;
    tmp.y_ += vr.y_;
    tmp.z_ += vr.z_;

    return tmp;
}


//----------------------------------------operator - -----------------------------------------------------//
// 重载算术操作符号-
Vec3  operator -(const Vec3 & vl, const Vec3 & vr)
{
    Vec3 tmp(vl);
    tmp.x_ -= vr.x_;
    tmp.y_ -= vr.y_;
    tmp.z_ -= vr.z_;

    return tmp;
}




//-----------------------------------------operator << ---------------------------------------------------//
// 重载输出操作符号<<
ostream& operator <<(ostream os, const Vec3 v)
{
    os<<"[ "<< v.x_<<", "<< v.y_<< ", "<< v.z_<<" ]"<<endl;

    return os;
}




//------------------------------------------operator * --------------------------------------------------//
// 重载算术操作符*
Vec3  operator *(const Vec3 & v, const float f)
{
    Vec3 tmp(v);
    tmp.x_ *= f;
    tmp.y_ *= f;
    tmp.z_ *= f;
    return tmp;
}


//------------------------------------------operator *---------------------------------------------------//
// 重载算术操作符*
Vec3  operator *( const float f, const Vec3 & v)
{
    Vec3 tmp(v);
    tmp.x_ *= f;
    tmp.y_ *= f;
    tmp.z_ *= f;
    return tmp;
}



//-------------------------------------------operator *---------------------------------------------------//
// 重载算术操作符*
float operator *(const Vec3& vl, const Vec3& vr)
{
    float tmp = vl.x_ * vr.x_ + vl.y_* vr.y_ + vl.z_* vr.z_;
    return tmp;
}




//--------------------------------------------operator /--------------------------------------------------//
Vec3  operator /( const Vec3 & v,const float f)
{
    Vec3 tmp(v);
    tmp.x_ /= f;
    tmp.y_ /= f;
    tmp.z_ /= f;
    return tmp;
}



//--------------------------------------------cross------------------------------------------------------//
//叉乘运算
Vec3 cross(const Vec3 &vl, const Vec3 &vr)
{

    Vec3 tmp;
    tmp.x_ = vl.y_*vr.z_ - vl.z_*vr.y_;
    tmp.y_ = vl.z_*vr.x_ - vl.x_*vr.z_;
    tmp.z_ = vl.x_*vr.y_ - vl.y_*vr.x_;

    return tmp;
}



/********************************************************************************/
/*                      CLASS  CAMERA                                         */
/********************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// computet the direction and the position of the camera
void Camera::computePosAndDir()
{
    // compute the position
    cv::Mat pos = -rotation_.t()* trans_;
    for(int i=0; i<3; i++)
    {
        pos_.at<float>(i) = pos.at<float>(i);
    }
    pos.release();
    // compute the direction
    cv::Mat dir_oir(3,1,CV_32FC1);
    dir_oir.setTo(0);
    dir_oir.at<float>(2) = -1;

    cv::Mat dir = rotation_.t()* dir_oir;
    for(int i=0; i<3; i++)
    {
        dir_.at<float>(i) = dir.at<float>(i);
    }

    pos.release();
    dir_oir.release();
    dir.release();

}
#endif

//--------------------------------------------decomposeProhMats-------------------------------------------//
// through the projection matrix we can get nearly all the information of the cameras
// the position of the camera
// the direction of the camera
// the focal of the camera
// the axises of the camera
void Camera::decomposeProjMats()
{

    // 1.0 get direction
    this->dir_.at<float>(0) = this->project_.at<float>(2,0);
    this->dir_.at<float>(1) = this->project_.at<float>(2,1);
    this->dir_.at<float>(2) = this->project_.at<float>(2,2);
    this->dir_ = this->dir_/ norm(this->dir_);

    // 2.0 get position
    cv::Mat KR(3,3,CV_32FC1);
    cv::Mat KT(3,1,CV_32FC1);
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            KR.at<float>(i,j) = this->project_.at<float>(i,j);
        }
    }
    for(int i=0; i<3; i++)
        KT.at<float>(i,0) = this->project_.at<float>(i,3);

    this->pos_ = -KR.inv()* KT;

    // 3.0 compute the focal
    cv::Mat R0(3,1, CV_32FC1);
    cv::Mat R1(3, 1, CV_32FC1);
    cv::Mat R2(3, 1, CV_32FC1);

    for(int i=0; i<3; i++)
    {
        R0.at<float>(i) = KR.at<float>(0, i);
        R1.at<float>(i) = KR.at<float>(1, i);
        R2.at<float>(i) = KR.at<float>(2, i);
    }

    this->focal_ = 0.5*abs(norm(R0.cross(R2)))+ 0.5*abs(norm(R1.cross(R2)));


    // 4.0 axises of the camera
    this->zaxis_ = this->dir_;
    this->yaxis_ = this->zaxis_.cross(R0);
    this->yaxis_ = this->yaxis_/norm(this->yaxis_);

    this->xaxis_ = this->yaxis_.cross(this->zaxis_);
    this->xaxis_ = this->xaxis_/norm(this->xaxis_);

    KR.release();
    KT.release();
    R0.release();
    R1.release();
    R2.release();
}
//--------------------------------------------------draw camera ----------------------------------------//
// draw camera
void Camera::draw()
{
    //cv::Mat rotation = rotation_.t();
    //cv::Mat  trans = -rotation_.t()* trans_;
    //qglviewer::Vec dir;
    //float angle = rotationMatrixToAngleAxis(rotation, dir);
    //cout<<"angle axis: "<<angle* dir.x<<", "<<angle* dir.y<<", "<<angle* dir.z<<endl;
    // cout<<"Camera Pos: "<< trans.at<float>(0)<<", "<< trans.at<float>(1)<<", "<< trans.at<float>(2)<<endl;


    glBegin(GL_QUADS);

    glVertex3f(-0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0.4, -0.3, -focal_/(2500.0));
    glVertex3f(-0.4, -0.3, -focal_/(2500.0));
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(-0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(0.4, 0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(0.4,-0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(-0.4,-0.3, -focal_/(2500.0));
    glVertex3f(0, 0, 0);
    glEnd();
}

#if 0
//////////////////////////////////////////////////////////////////////////////////////////////////////
qglviewer::Vec Camera::project(const qglviewer::Vec& coord)
{
    qglviewer::Vec p2D;
    cv::Mat p3D(4,1,CV_32FC1);
    p3D.at<float>(0) = coord.x;
    p3D.at<float>(1) = coord.y;
    p3D.at<float>(2) = coord.z;
    p3D.at<float>(3) = 1.0;

    cv::Mat ptmp = project_* p3D;
    if(ptmp.at<float>(2)==0)
    {
        p2D.x = -1;
        p2D.y = -1;
        p2D.z = 1;
    }
    else
    {
        p2D.x = ptmp.at<float>(0)/ptmp.at<float>(2);
        p2D.y = ptmp.at<float>(1)/ptmp.at<float>(2);
        p2D.z = 1.0;
    }
    ptmp.release();
    p3D.release();
    return p2D;
}
#endif

//--------------------------------------------------project-----------------------------------------------//
PointXY  Camera::project(const Vec3 &coord)
{
    PointXY p2D;
    cv::Mat p3D(4,1,CV_32FC1);
    p3D.at<float>(0) = coord.x_;
    p3D.at<float>(1) = coord.y_;
    p3D.at<float>(2) = coord.z_;
    p3D.at<float>(3) = 1.0;

    cv::Mat ptmp = project_* p3D;
    if(ptmp.at<float>(2)==0)
    {
        p2D.x = -1;
        p2D.y = -1;
    }
    else
    {
        p2D.x = ptmp.at<float>(0)/ptmp.at<float>(2);
        p2D.y = ptmp.at<float>(1)/ptmp.at<float>(2);
    }
    ptmp.release();
    p3D.release();
    return p2D;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////




//--------------------------------------------------compute_bounding_box----------------------------------//
// compute the bouding box of the pointcloud
void PointCloud::compute_bounding_box()
{

    p_Xmin_ = p_Ymin_ = p_Zmin_ = std::numeric_limits<double>::max();
    p_Xmax_ = p_Ymax_ = p_Zmax_ = -std::numeric_limits<double>::max();

    foreach(uint id, p_pt_ids_)
    {
        if(p_Xmin_ > p_points_[id].x)      p_Xmin_ = p_points_[id].x;
        if(p_Ymin_ > p_points_[id].y)      p_Ymin_ = p_points_[id].y;
        if(p_Zmin_ > p_points_[id].z)      p_Zmin_ = p_points_[id].z;

        if(p_Xmax_< p_points_[id].x)       p_Xmax_ = p_points_[id].x;
        if(p_Ymax_< p_points_[id].y)       p_Ymax_ = p_points_[id].y;
        if(p_Zmax_< p_points_[id].z)       p_Zmax_ = p_points_[id].z;
    }
}



//--------------------------------------------------compute_center---------------------------------------------------//
//compute the center of the point cloud
void PointCloud::compute_center()
{
    p_center_x_ = p_center_y_ = p_center_z_ = 0;

    foreach(uint id,  p_pt_ids_)
    {
        p_center_x_  += p_points_[id].x;
        p_center_y_  += p_points_[id].y;
        p_center_z_  += p_points_[id].z;
    }

    p_center_x_ /= (float)ptNum();
    p_center_y_ /= (float)ptNum();
    p_center_z_ /= (float)ptNum();

    cout<<"scene center: "<< p_center_x_ << ", " <<  p_center_y_<<", "<<  p_center_z_<<endl;
}



//---------------------------------------------------computeEdges---------------------------------------------------//
void Mesh::computeEdges()
{
    if(m_vertices_.size()==0 || m_facets_.size() ==0)
    {
    }
    else
    {
        // put all the edges into the QSet structure, and each edge is guaranteed
        QSet<QPair<uint, uint> > e;

        for(int i=0; i< m_facets_.size(); i++)
        {
            for(int j=0; j< m_facets_[i].size(); j++)
            {
                uint id0 = (uint)j;
                uint id1 = (uint)(j+1) % m_facets_[i].size();

                if(m_facets_[i][id0]> m_facets_[i][id1])e.insert(qMakePair(m_facets_[i][id1], m_facets_[i][id0]));
                else e.insert(qMakePair(m_facets_[i][id0], m_facets_[i][id1] ));
            }
        }

        // get the all the edges from QSet and push it into m_edges_
        QSet<QPair<uint,uint > > ::const_iterator iter = e.constBegin();
        while(iter!= e.constEnd())
        {
            m_edges_<< *iter;
            iter++;
        }
    }
}



//----------------------------------------------------fittingPlane---------------------------------------------------//
void Plane3D::fittingPlane(vector<Vec3> & points)
{
    int  pts_num = points.size();
    cv::Mat features(pts_num, 3, CV_32FC1);

    for(int i=0; i< points.size(); i++)
    {
        // points
        for(int j=0; j< 3; j++)
        {
            features.at<float>(i, j) = points[i][j];
        }
    }

    // center of the plane
    cv:: Mat mean(3,1,CV_32FC1);
    mean.setTo(0);
    for(int i =0; i< pts_num; i++)
    {
        mean.at<float>(0) += features.at<float>(i,0)/pts_num;
        mean.at<float>(1) += features.at<float>(i,1)/pts_num;
        mean.at<float>(2) += features.at<float>(i,2)/pts_num;
    }

    for(int i=0; i< 3; i++) p_center_[i] = mean.at<float>(i);
    ////////////////////////////////////

    for(int i=0; i<pts_num; i++)
    {
        features.at<float>(i,0) -= mean.at<float>(0);
        features.at<float>(i,1) -= mean.at<float>(1);
        features.at<float>(i,2) -= mean.at<float>(2);
    }

    // compute parameters
    cv::Mat covar_matrix = (features.t()*features) /pts_num;

    cv::SVD svd;
    cv:: Mat U,S,V;
    svd.compute(covar_matrix, S, U, V);


//    p_normal_[0] = U.at<float>(0,2);
//    p_normal_[1] = U.at<float>(1,2);
//    p_normal_[2] = U.at<float>(2,2);

//    cout<<"( "<<p_normal_.x_<<", "<<p_normal_.y_<<", "<< p_normal_.z_<<" )---->";

   // normal 通过平面边界的叉乘来计算
    int id0 = p_facets_[0][0];
    int id1 = p_facets_[0][1];
    int id2 = p_facets_[0][2];
    Vec3 v10 = p_vertices_[id1]  - p_vertices_[id0];
    Vec3 v21 = p_vertices_[id2]  - p_vertices_[id1];
    v10.normalize();
    v21.normalize();
    p_normal_ = cross(v10, v21);
    p_normal_.normalize();

//    cout<<"( "<<p_normal_.x_<<", "<<p_normal_.y_<<", "<<p_normal_.z_<<endl;


    Vec3 frameZ = p_normal_;
    p_frame_y_  = Vec3(U.at<float>(0,0), U.at<float>(1,0),U.at<float>(2,0));
    p_frame_x_ = cross(p_frame_y_, frameZ);

    float d = p_normal_ *p_center_;
    p_d_ = -d;

    features.release();
    mean.release();
    covar_matrix.release();
    U.release();
    V.release();
    S.release();
}



//----------------------------------------------------constraint_triangulation---------------------------------------//
void Plane3D::constraint_triangulation()
{


}


//-----------------------------------------------------updateTriangulations-------------------------------------------//
void Plane3D::updateTriangulations(vector<vector<Vec3> >facets)
{
    p_vertices_.clear();
    p_facets_.clear();


    // create a table
    map<Vec3, int> table;
    for(int i=0; i< facets.size(); i++)
    {
        for(int j=0; j< facets[i].size(); j++)
        {
            Vec3 pt = facets[i][j];
            table.insert(make_pair(pt, 0));
        }
    }

    /////////////////////attach index to each point/////////////////////////////
    int index = 0;
    for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
    {
        iter->second = index;
        index++;
    }

    //get new vertices
    for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
    {
        p_vertices_.push_back(iter->first);
    }

    // get facets
    for(int i=0; i< facets.size(); i++)
    {
        QVector<uint> facetID;
        for(int j=0; j< facets[i].size(); j++)
        {
            Vec3 pt = facets[i][j];
            facetID.append(table[pt]);
        }
        p_facets_.push_back(facetID);
    }


}



//-----------------------------------------------------drawTriangulation----------------------------------------------//
void Plane3D::drawTriangulation()
{
    // draw facets
    glPushMatrix();
    glColor3f((GLfloat)p_color_.red()/255.0,
              (GLfloat)p_color_.green()/255.0,
              (GLfloat)p_color_.blue()/255.0);

    foreach(QVector<uint> facet, p_facets_)
    {
        glBegin(GL_TRIANGLES);
        foreach(uint id, facet)
        {
            glVertex3f(p_vertices_[id].x_, p_vertices_[id].y_, p_vertices_[id].z_);
        }
        glEnd();
    }


    // draw lines
    glColor3f(0.0f, 0.0f, 0.0f);
    glLineWidth(2.0);
    foreach(QVector<uint> facet, p_facets_)
    {
        glBegin(GL_LINE_LOOP);

        foreach(uint id, facet)
        {
            glVertex3f(p_vertices_[id].x_, p_vertices_[id].y_, p_vertices_[id].z_);
        }

        glEnd();
    }
    glLineWidth(1.0);

    // draw vertices
    glColor3f(1.0f, 0.0, 0.0);
    glPointSize(4.0);
    glBegin(GL_POINTS);
    foreach(QVector<uint> facet, p_facets_)
    {
        foreach(uint id, facet)
        {
            glVertex3f(p_vertices_[id].x_, p_vertices_[id].y_, p_vertices_[id].z_);
        }
    }
    glEnd();
    glPointSize(1.0);

    glPopMatrix();

}




//-----------------------------------------------------drawTriangulationWithEdges------------------------------------//
void Plane3D::drawTriangulationWithEdges(int i)
{

    // draw facets
    glBegin(GL_TRIANGLES);
    foreach(int id, p_facets_[i])
      {
          glVertex3f(p_vertices_[id].x_, p_vertices_[id].y_, p_vertices_[id].z_);
      }
    glEnd();


    // draw edges
    glLineWidth(2.0);
    glColor3f(0.0, 0.0, 0.0);

    glBegin(GL_LINE_LOOP);

    foreach(int id, p_facets_[i])
      {
          glVertex3f(p_vertices_[id].x_, p_vertices_[id].y_, p_vertices_[id].z_);
      }
    glEnd();

    glLineWidth(1.0);
    glPopMatrix();



}



//-----------------------------------------------------drawTriangulation---------------------------------------------//
void Plane3D::drawTriangulation(int i)
{

    // draw facets
    glPushMatrix();

      glBegin(GL_TRIANGLES);
      foreach(int id, p_facets_[i])
        {
            glVertex3f(p_vertices_[id].x_, p_vertices_[id].y_, p_vertices_[id].z_);
        }
      glEnd();

     glPopMatrix();

}




//-------------------------------------------------------cvt3Dto2D----------------------------------------------------//
PointXY Plane3D::cvt3Dto2D(const Vec3 &pt3D)
{
    // translate 3D to 2D
    PointXY pt2D;
    Vec3 p(pt3D.x_, pt3D.y_, pt3D.z_);
    pt2D.x = (p - p_center_)* p_frame_x_;
    pt2D.y = (p - p_center_)* p_frame_y_;

    return pt2D;
}



//-------------------------------------------------------cvt2Dto3D---------------------------------------------------//
Vec3 Plane3D::cvt2Dto3D(const PointXY &pt2D)
{
    Vec3 pt3D = pt2D.x*  p_frame_x_ + pt2D.y * p_frame_y_ + p_center_;
    return pt3D;
}



//-----------------------------------------------------getBoundaryFromTriangulations---------------------------------//
void Plane3D::getBoundaryFromTriangulations()
{
    p_boundary3Ds_.clear();

    // convert 3D points to 2D points
    vector<PointXY> vertice2Ds;

    foreach(Vec3 pt3D, p_vertices_)
    {
       PointXY  pt2D = this->cvt3Dto2D(pt3D);
       vertice2Ds.push_back(pt2D);
    }

    // convert 3D facets to 2D facets
    vector<vector<PointXY> > facets2D;
    foreach(QVector<uint> facet_ids, p_facets_)
    {
        vector<PointXY> facet;
        foreach(uint pt_id, facet_ids)
        {
            facet.push_back(vertice2Ds[pt_id]);
        }
        facets2D.push_back(facet);
    }

    // get boundarys from triangulations
    vector<vector<PointXY> >  boundarys2D = getOuterBoundaryFromTrians(facets2D);

   // bool flag = isCounterClockWise(boundarys2D[0]);

    // convert 2D facets to 3D facets
    for(int i=0; i< boundarys2D.size(); i++)
    {
        QVector<Vec3> boudary3D;
        for(int j=0; j< boundarys2D[i].size(); j++)
        {
            PointXY pt2D = boundarys2D[i][j];

            Vec3 pt3D = this->cvt2Dto3D(pt2D);

            boudary3D.push_back(pt3D);
        }

        p_boundary3Ds_.push_back(boudary3D);
    }

}




//-----------------------------------------------------boundaryProcessing---------------------------------------------//
// eliminate redunctant vertices
// make sure that no three points are on the same line
void Plane3D::boundaryProcessing()
{
    int pt_num = (int)p_boundary3Ds_[0].size();
    QVector<Vec3> new_outer_boundary;

    for(int i=0; i< p_boundary3Ds_[0].size(); i++)
    {
      int id0 = i;
      int id1 = (i+1)%pt_num;
      int id2 = (i+2)%pt_num;

      Vec3 v10 = p_boundary3Ds_[0][id1] - p_boundary3Ds_[0][id0];
      Vec3 v21 = p_boundary3Ds_[0][id2] - p_boundary3Ds_[0][id1];
      v10.normalize();
      v21.normalize();

      float pro = v10 * v21;

      pro = min(pro, (float)0.99999);
      pro = max((float)-0.9999, pro);

      float angle = acos(abs(pro))*180/3.1415;

      if(angle >75){ new_outer_boundary.append(p_boundary3Ds_[0][id1]);}
    }

    p_boundary3Ds_[0].swap(new_outer_boundary);
}



//----------------------------------------------adjustNormalDirection-------------------------------------------//
void Plane3D::adjustNormalDirection()
{
    // according direction of boundarys to adjust nomals
    // the 0-th boundary is the outer boundary
    if(p_facets_.size()>0)
    {
        int id0 = p_facets_[0][0];
        int id1 = p_facets_[0][1];
        int id2 = p_facets_[0][2];
        Vec3 v10 = p_vertices_[id1]  - p_vertices_[id0];
        Vec3 v21 = p_vertices_[id2]  - p_vertices_[id1];
        v10.normalize();
        v21.normalize();
        Vec3 n = cross(v10, v21);
        n.normalize();

//        cout<<"( "<<n.x_<<", "<<n.y_<<", "<< n.z_<<" )---->";
//        cout<<"( "<<p_normal_.x_<<", "<<p_normal_.y_<<", "<<p_normal_.z_<<" ): "<<n*p_normal_<<endl;



        if(n* p_normal_< 0)
        {
          p_normal_ = -1* p_normal_;
          p_d_ = -p_d_;

          Vec3 tmp = p_frame_x_;
          p_frame_x_ = p_frame_y_;
          p_frame_y_ = tmp;
        }
    }
}




//----------------------------------------------makePlaneFacetsCCW--------------------------------------------------//
void Plane3D::makePlaneFacetsCCW()
{
     QVector<QVector<uint> > new_facets;

       foreach(QVector<uint> facet,  p_facets_)
        {
            Polygon_2 polygon;
            foreach(int id, facet)
            {
                Vec3 pt3D = p_vertices_[id];
                PointXY pt2D = this->cvt3Dto2D(pt3D);
                polygon.push_back(Point_2(pt2D.x, pt2D.y));
            }

            // if orientation is clock wise
            if(polygon.orientation() == CGAL::CLOCKWISE)
            {
              QVector<uint> facet_tmp;
              facet_tmp.append(facet[0]);
              for(int i=facet.size() -1; i> 0; i--)
              {
                  facet_tmp.append(facet[i]);
              }
              new_facets.append(facet_tmp);
            }

            // if orientation is counterclockwise
             if(polygon.orientation() == CGAL::COUNTERCLOCKWISE)
             {
               new_facets.append(facet);
             }
        }
       p_facets_.swap(new_facets);
}
