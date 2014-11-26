
//Note: glew provides functions related to vertices arrays and buffers
//Qt provides functions related to buffers and undefined the functions related to buffers from glew, however,
// Qt doesn't provide any functions relates to vertex array
//And hence we include GL/glew.h for vertex array functions and meanwhile we include QGLFunctions for
// buffer functions
#include<GL/glew.h> // must be put in the front of gl.h or glu.h
//#include"QGLFunctions"

#include"sw_glviewer.h"
#include"sw_functions.h"

#include <string>
#include<stdlib.h>
#include<fstream>
#include<sstream>

#include<QFileDialog>
#include<qcolordialog.h>
#include<QMessageBox>
#include<QWidget>
#include<qfiledialog.h>
#include<QProgressDialog>
#include<QObject> // tr was not declare
#include<QApplication> // qaApp
#include<QTextStream>

#include<GL/gl.h>
#include<GL/glu.h>
#include<GL/freeglut.h>
#include<time.h>
#include<sstream>
#include<cassert>


using namespace SW;

#ifdef VERTEX_ARRAY
#define BUFFER_OFFSET(offset) ((GLubyte*) NULL + offset)
#endif


//-----------------------------------------constructor--------------------------------------------------//
SW::GLViewer::GLViewer(QWidget *parent0,
                       const QGLWidget *parent1, Qt::WFlags f):
                       QGLViewer(parent0, parent1, f)
{
    setAutoFillBackground(true);

    g_display_dense_pts_ = false;
    g_display_vertices_ = false;
    g_display_wire_frame_ = false;
    g_display_flat_ = false;
    g_display_texture_ = false;
    g_display_cameras_ = false;
    g_display_inconsist_=false;
    g_display_slices_ = false;
    g_display_modelling_process_ = false;
    g_display_modelling_results_ = false;
    g_display_all_planes_triangulations_= false;
    g_display_single_plane_triangulations_ = false;
    g_display_back_projected_quads_ = false;
    g_display_added_widow_planes_  = false;

}


//----------------------------------------destructor----------------------------------------------------//
SW::GLViewer::~GLViewer()
{

}


//----------------------------------------helpString----------------------------------------------------//
QString SW::GLViewer::helpString()
{
    QString text("<h2> MeshLive 1.0 [2006.10.18.1]<p></h2>");
    text += "An easy and extensible mesh interaction C++ program for real-time applications..<p> ";
    text += "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Designed by hywu, jpan, xlvector. Since 2006.<p> ";
    text += "Based on:<p> ";
    text += "QT (http://www.trolltech.com/)<p> ";
    text += "libQGLViewer (http://artis.imag.fr/~Gilles.Debunne/QGLViewer/)<p> ";
    text += "CGAL (http://www.cgal.org/, http://www-sop.inria.fr/geometrica/team/Pierre.Alliez/)<p> ";
    text += "OpenMesh (http://www.openmesh.org/)<p> ";
    text += "Boost (http://www.boost.orgwww/)<p> ";
    text += "OpenCV (http://sourceforge.net/projects/opencvlibrary/)<p> ";
    text += "Python (http://www.python.org/)<p> ";
    text += "etc.<p> ";

    return text;
}


//-------------------------------------------init-------------------------------------------------------//
void SW::GLViewer::init()
{

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    // glDisable(GL_DITHER);
    // glShadeModel(GL_FLAT);
    // glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    //------Note very important, or error occurs-------//
#ifdef VERTEX_ARRAY
    glewInit();
    glGenVertexArrays(NumVAOs, g_VAOs_);
#endif

}


//--------------------------------------------draw------------------------------------------------------//
void SW::GLViewer::draw()
{
    //drawAxises(0.1, 0.1);
    //-------------------- draw mesh vertices------------------------------------------//
    if(g_display_vertices_ == true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(g_VAOs_[MESH]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        glDrawElements(GL_TRIANGLES, g_mesh_->m_facets_.size()*3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else

        for(int i=0; i< g_mesh_->m_vertices_.size(); i++)
        {
            glColor3f(0.5, 0.5, 0.5);
            glBegin(GL_POINTS);
            glVertex3f(g_mesh_->m_vertices_[i].x_,
                       g_mesh_->m_vertices_[i].y_,
                       g_mesh_->m_vertices_[i].z_);
            glEnd();
        }
#endif
    }


    //--------------------draw mesh wire frame-----------------------------------------//
    if(g_display_wire_frame_ == true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(g_VAOs_[MESH]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, g_mesh_->m_facets_.size()*3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
        // must get edges......
#endif
    }

    //------------------- draw mesh flat  -----------------------------------------------//
    if(g_display_flat_ == true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(g_VAOs_[MESH]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, g_mesh_->m_facets_.size()*3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
        foreach(QVector<uint> facet, g_mesh_->m_facets_)
        {
            glColor3f(0.5, 0.5, 0.5);
            glBegin(GL_TRIANGLES);
            foreach(uint id, facet)
            {
                glVertex3f(g_mesh_->m_vertices_[id].x_,
                           g_mesh_->m_vertices_[id].y_,
                           g_mesh_->m_vertices_[id].z_);
            }
            glEnd();
        }
#endif
    }

    //-------------------------------- draw texture  --------------------------------------------------//
    if(g_display_texture_ == true)
    {
#if 0
        glEnable(GL_TEXTURE_2D); // ¼ÓÕâ¸ö£¬ÃüÁî¾Í¿ÉÒÔÁË
        glBindVertexArray(g_VAOs_[MESH]);
        glBindTexture(GL_TEXTURE_2D, g_texture_id_);
        glDisableClientState(GL_COLOR_ARRAY);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, NumElements, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

        glEnableClientState(GL_COLOR_ARRAY);
        glDisable(GL_TEXTURE_2D);
#endif
    }

    //---------------------------------draw cameras ---------------------------------------------------//
    if(g_display_cameras_== true)
    {
#ifdef VERTEX_ARRAY

#else
        foreach(QString key, g_cameras_->keys())
        {
            foreach(Camera cam, g_cameras_->values(key))
            {
                Vec3 axis;
                Vec3 src(0,0,-1);
                Vec3 dirst(cam.dir_.at<float>(0),
                           cam.dir_.at<float>(1),
                           cam.dir_.at<float>(2));

                // 计算旋转轴和旋转角度
                float angle = angleAxisFromDirections(src, dirst, axis);

                glDisable(GL_LIGHTING);

                glPushMatrix();

                glTranslatef( cam.pos_.at<float>(0),
                              cam.pos_.at<float>(1),
                              cam.pos_.at<float>(2) );

                glRotatef(angle*180/3.1415, axis.x_, axis.y_, axis.z_);
                glColor3f( (GLfloat) (cam.color_.red())/255.0,
                           (GLfloat) (cam.color_.green())/255.0,
                           (GLfloat) (cam.color_.blue())/255.0);
                cam.draw();

                glPopMatrix();
            }
        }

#endif
    }


    //----------------------------------draw inconsistent regions--------------------------------------//
    if(g_display_inconsist_ ==  true)
    {
        glPushMatrix();
        glBegin(GL_POINTS);
        foreach(uint it,  g_pc_->p_pt_ids_)
        {
            if(g_pc_->p_points_[it].inconsist_ == true)
            {
                glColor3f(1.0, 0.0, 0.0 );
                glVertex3f(g_pc_->p_points_[it].x, g_pc_->p_points_[it].y, g_pc_->p_points_[it].z );
            }
        }
        glEnd();
        glPopMatrix();

    }


    //---------------------------------draw slices----------------------------------------------------//
    if(g_display_slices_ == true)
    {
        QVector<QVector<uint > > pts_ids = g_floorplan_displays_->f_slice_pt_ids_;
        QVector<float> yCoords = g_floorplan_displays_->f_Y_coords_;

        assert(yCoords.size() == pts_ids.size());

        glPushMatrix();
        glBegin(GL_POINTS);

        for(int i=0; i< pts_ids.size() ; i++)
        {
            float y = yCoords[i];
            for(int j=0; j< pts_ids[i].size(); j++)
            {
                int id = pts_ids[i][j];

                glColor3f(0.0, 1.0, 0.0 );
                glVertex3f(g_pc_->p_points_[id].x, y, g_pc_->p_points_[id].z);
            }
        }
        glEnd();
        glPopMatrix();
    }


    //----------------------------------draw modelling process-----------------------------------------//
    if(g_display_modelling_process_ ==  true)
    {
       //draw starting layyer
        drawStartingLayer();

       //draw ending layer
        drawEndingLayer();

       //draw semi-planes
        drawSemiPlanes();
    }


    //------------------------------------draw all plane triangulations---------------------------------//
    if(g_display_all_planes_triangulations_ == true)
    {
        foreach(QString key, g_plane3Ds_->keys())
            foreach(Plane3D value, g_plane3Ds_->values(key))
            {
                if(value.p_facets_.size()>0&&value.p_vertices_.size()>0)
                {
                    value.drawTriangulation();
                }
            }
    }


    //------------------------------------draw single plane triangulations------------------------------//
    if(g_display_single_plane_triangulations_ == true)
    {
        if(g_current_plane3D_ptr_->p_facets_.size()>0&&g_current_plane3D_ptr_->p_vertices_.size()>0)
        {
           // glColor3f(0.5, 1.0, 0.5);
            g_current_plane3D_ptr_->drawTriangulation();
        }
    }


    //-------------------------------------draw back projected quads-------------------------------------//
    if(g_display_back_projected_quads_== true)
    {
        glColor4f(1.0, 0.0, 1.0, 0.75);
        foreach(QVector<Vec3> quad, g_current_plane3D_ptr_->p_window_boundary3Ds_)
        {
            glPushMatrix();

            int pt_num = quad.size();
            for(int i =0;i< quad.size(); i++)
            {
                int id0 = i;
                int id1 = (i+1) % pt_num;
                glBegin(GL_LINE);

                glVertex3f(quad[id0].x_, quad[id0].y_, quad[id0].z_);
                glVertex3f(quad[id1].x_, quad[id1].y_, quad[id1].z_);

                glEnd();
            }

            glPopMatrix();
        }

    }


    //--------------------------------------draw added window planes------------------------------------//
    if(g_display_added_widow_planes_ == true)
    {
        glEnable(GL_BLEND);
        glColor4f(1.0, 1.0, 0.0, 0.75);
        foreach(QVector<Vec3> quad, g_current_plane3D_ptr_->p_added_window_planes_)
        {
            glPushMatrix();

            glBegin(GL_QUADS);

            foreach(Vec3 pt, quad)
            {
                glVertex3f(pt.x_, pt.y_, pt.z_);
            }

            glEnd();

            glPopMatrix();
        }
        glDisable(GL_BLEND);

        // draw lines
        // draw lines
        glColor3f(0.0f, 0.0f, 0.0f);
        glLineWidth(2.0);
        foreach(QVector<Vec3> quad, g_current_plane3D_ptr_->p_added_window_planes_)
        {
            glBegin(GL_LINE_LOOP);

            foreach(Vec3 pt, quad)
            {
                glVertex3f(pt.x_, pt.y_, pt.z_);
            }
            glEnd();
        }
        glLineWidth(1.0);

    }


    //-----------------------------------draw modelling results----------------------------------------//
    if(g_display_modelling_results_== true)
    {
        glPushMatrix();
        glColor4f(0.5f, 1.0f, 0.5f, 1.0f);

        // draw facets
        foreach(QVector<uint> facet, g_mesh_->m_facets_)
        {
            glBegin(GL_TRIANGLES);
            foreach(uint id, facet)
            {
                glVertex3f(g_mesh_->m_vertices_[id].x_,
                           g_mesh_->m_vertices_[id].y_,
                           g_mesh_->m_vertices_[id].z_);
            }
            glEnd();
        }
        // draw edges
        glColor3f(0.0f, 0.0f, 0.0f);
        glLineWidth(2.0);

        QVector<QPair<uint, uint> > ::const_iterator iter = g_mesh_->m_edges_.constBegin();
        while(iter!= g_mesh_->m_edges_.constEnd())
        {
            uint id0 = iter->first;
            uint id1 = iter->second;

            glBegin(GL_LINES);

            glVertex3f(g_mesh_->m_vertices_[id0].x_,
                       g_mesh_->m_vertices_[id0].y_,
                       g_mesh_->m_vertices_[id0].z_);
            glVertex3f(g_mesh_->m_vertices_[id1].x_,
                       g_mesh_->m_vertices_[id1].y_,
                       g_mesh_->m_vertices_[id1].z_);

            glEnd();

            iter++;
        }

        glLineWidth(1.0);

        // draw vertices
        glColor3f(1.0f, 0.0, 0.0);
        glPointSize(4.0);
        glBegin(GL_POINTS);

        foreach(Vec3 pt, g_mesh_->m_vertices_)
        {
          glVertex3f(pt.x_, pt.y_, pt.z_);
        }

        glEnd();
        glPointSize(1.0);

        glPopMatrix();
    }


    //---------------------------------- draw dense points---------------------------------------------//
    if(g_display_dense_pts_== true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(POINTS);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
        glDrawElements(GL_POINTS, g_pc_->ptNum(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
        glBegin(GL_POINTS);
        foreach(uint id, g_pc_->p_pt_ids_)
        {
            glColor3f(g_pc_->p_points_[id].r/255.0,
                      g_pc_->p_points_[id].g/255.0,
                      g_pc_->p_points_[id].b/255.0);
            glVertex3f(g_pc_->p_points_[id].x,
                       g_pc_->p_points_[id].y,
                       g_pc_->p_points_[id].z);
        }
        glEnd();
#endif
    }


    glFlush();
}


//---------------------------------------------viewall--------------------------------------------------//
void SW::GLViewer::viewAll()
{
    // compute the bounding box of all the objects in the scene
    computeSceneBoundingBox();
#if 0
    setSceneBoundingBox(qglviewer::Vec(g_bounding_left_top_.x(),
                                       g_bounding_left_top_.y(),
                                       g_bounding_left_top_.z()),
                        qglviewer::Vec(g_bounding_right_bottom_.x(),
                                       g_bounding_right_bottom_.y(),
                                       g_bounding_right_bottom_.z()));
#endif

    setSceneBoundingBox(qglviewer::Vec(-2.0, -2.0, -2.0),
                        qglviewer::Vec( 2.0, 2.0,  2.0));

    // cout<<"Bounding Box: "<<g_bounding_left_top_.x()<<" ";
    // cout<<g_bounding_left_top_.y()<<" ";
    // cout<<g_bounding_left_top_.z()<<endl;

    // cout<<"Bouding Box: "<<g_bounding_right_bottom_.x()<<" ";
    // cout<< g_bounding_right_bottom_.y()<<" ";
    // cout<< g_bounding_right_bottom_.z() <<endl;

    showEntireScene();
    updateGL();
}


//----------------------------------------------computeSceneBoundingBox---------------------------------//
void SW::GLViewer::computeSceneBoundingBox()
{ 
#if 0
    g_bounding_left_top_.setX(100000);
    g_bounding_left_top_.setY(100000);
    g_bounding_left_top_.setZ(100000);
    g_bounding_right_bottom_.setX(-100000);
    g_bounding_right_bottom_.setY(-100000);
    g_bounding_right_bottom_.setZ(-100000);

    // bounding box of the dense points
    foreach(Point pt, g_pc_->p_points_)
    {
        if (g_bounding_left_top_.x() > pt.x)g_bounding_left_top_.setX(pt.x);
        if (g_bounding_left_top_.y() > pt.y)g_bounding_left_top_.setY(pt.y);
        if (g_bounding_left_top_.z() > pt.z)g_bounding_left_top_.setZ(pt.z);

        if (g_bounding_right_bottom_.x() < pt.x) g_bounding_right_bottom_.setX(pt.x);
        if (g_bounding_right_bottom_.y() < pt.y) g_bounding_right_bottom_.setY(pt.y);
        if (g_bounding_right_bottom_.z() < pt.z) g_bounding_right_bottom_.setZ(pt.z);
    }
#endif

    g_bounding_left_top_.setX(g_pc_->xmin());
    g_bounding_left_top_.setY(g_pc_->ymin());
    g_bounding_left_top_.setZ(g_pc_->zmin());
    g_bounding_right_bottom_.setX(g_pc_->xmax());
    g_bounding_right_bottom_.setY(g_pc_->ymax());
    g_bounding_right_bottom_.setZ(g_pc_->zmax());


    // bounding box of the meshes
    foreach(Vec3 pt, g_mesh_->m_vertices_)
    {
        if (g_bounding_left_top_.x() > pt.x_)g_bounding_left_top_.setX(pt.x_);
        if (g_bounding_left_top_.y() > pt.y_)g_bounding_left_top_.setY(pt.y_);
        if (g_bounding_left_top_.z() > pt.z_)g_bounding_left_top_.setZ(pt.z_);

        if (g_bounding_right_bottom_.x() <  pt.x_) g_bounding_right_bottom_.setX(pt.x_);
        if (g_bounding_right_bottom_.y() <  pt.y_) g_bounding_right_bottom_.setY(pt.y_);
        if (g_bounding_right_bottom_.z() <  pt.z_) g_bounding_right_bottom_.setZ(pt.z_);
    }
}


//--------------------------------------------makeDensePoints-------------------------------------------//
void SW::GLViewer::makeDensePoints()
{

#ifdef VERTEX_ARRAY
    enum{Vertices, Color, Elements, NumVBOs};
    GLuint buffers[NumVBOs];

    // active a vertex array
    glBindVertexArray(POINTS);
    glGenBuffers(NumVBOs, buffers);

    GLfloat * vertices = new GLfloat [g_pc_->ptNum() * 3];

    for(int i=0; i< g_pc_->ptNum(); i++)
    {
        int id = g_pc_->p_pt_ids_[i];
        vertices[i* 3 + 0] =  (GLfloat)(g_pc_->p_points_)[id].x;
        vertices[i* 3 + 1] =  (GLfloat)(g_pc_->p_points_)[id].y;
        vertices[i* 3 + 2] =  (GLfloat)(g_pc_->p_points_)[id].z;

    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Vertices]);
    glBufferData(GL_ARRAY_BUFFER, g_pc_->ptNum()* sizeof(GLfloat), vertices, GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);


    // COLOR
    GLfloat * colors = new GLfloat [ g_pc_->ptNum() * 3];
    for(int i=0; i< g_pc_->ptNum(); i++)
    {
        int id = g_pc_->p_pt_ids_[i];
        colors[i* 3 + 0] =  (GLfloat)(g_pc_->p_points_)[id].r/(GLfloat)255;
        colors[i* 3 + 1] =  (GLfloat)(g_pc_->p_points_)[id].g/(GLfloat)255;
        colors[i* 3 + 2] =  (GLfloat)(g_pc_->p_points_)[id].b/(GLfloat)255;
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Color]);
    glBufferData(GL_ARRAY_BUFFER, g_pc_->ptNum()*sizeof(GLfloat), colors, GL_STATIC_DRAW);
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);


#if 0  // There will be problems if normals are used
    //NORMAL
    GLfloat * normals = new GLfloat [ g_points_->size() * 3];
    for(int i=0; i< g_points_->size(); i++)
    {
        normals[i* 3 + 0] =  (GLfloat)(*g_points_)[i].normal_x;
        normals[i* 3 + 1] =  (GLfloat)(*g_points_)[i].normal_y;
        normals[i* 3 + 2] =  (GLfloat)(*g_points_)[i].normal_z;

    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[NORMALS]);
    glBufferData(GL_ARRAY_BUFFER, g_points_->size() * 3 * sizeof(GLfloat), normals, GL_STATIC_DRAW);
    glNormalPointer(3, GL_FLOAT,  BUFFER_OFFSET(0));
    glEnableClientState(GL_NORMAL_ARRAY);
#endif

    // FACETS ELEMENTS
    GLuint * elements = new GLuint [g_pc_->ptNum()];
    for(int i=0; i< g_pc_->ptNum(); i++)
    {
        elements[i] = (GLuint)i;
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[Elements]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, g_pc_->ptNum()*sizeof(GLuint), elements, GL_STATIC_DRAW);
#endif
}


//---------------------------------------------makeMeshes------------------------------------------------//
void SW::GLViewer::makeMeshes()
{
#ifdef VERTEX_ARRAY
    enum {VERTICES, COLORS, TEXTURE_COORDS, ELEMENTS, NUMVBOS};
    GLuint buffers[NUMVBOS];

    glBindVertexArray(g_VAOs_[MESH]);
    glGenBuffers(NUMVBOS, buffers);

    // VERTICES
    GLfloat * vertices = new GLfloat [g_mesh_->m_vertices_.size() * 3];
    for(int i=0; i< g_mesh_->m_vertices_.size(); i++)
    {
        vertices[i* 3 + 0] = g_mesh_->m_vertices_[i].x_;
        vertices[i* 3 + 1] = g_mesh_->m_vertices_[i].y_;
        vertices[i* 3 + 2] = g_mesh_->m_vertices_[i].z_;
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[VERTICES]);
    glBufferData(GL_ARRAY_BUFFER, g_mesh_->m_vertices_.size() * 3 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

    // COLOR
    GLfloat * colors = new GLfloat [g_mesh_->m_vertices_.size()* 3];
    for(int i=0; i< g_mesh_->m_vertices_.size(); i++)
    {
        colors[i * 3 + 0] = (GLfloat)0.5;
        colors[i * 3 + 1] = (GLfloat)0.5;
        colors[i * 3 + 2] = (GLfloat)0.5;
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[COLORS]);
    glBufferData(GL_ARRAY_BUFFER, g_mesh_->m_vertices_.size() * 3 * sizeof(GLfloat), colors, GL_STATIC_DRAW);
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

#if 0
    // NORMAL
    GLfloat * colors = new GLfloat [g_vertices_->size()* 3];
    glBindBuffer(GL_ARRAY_BUFFER, buffers[NORMALS]);
    glBufferData(GL_ARRAY_BUFFER, g_ver_num_ * 3 * sizeof(GLfloat), g_ver_normals_, GL_STATIC_DRAW);
    glNormalPointer(3, GL_FLOAT,  BUFFER_OFFSET(0));
    glEnableClientState(GL_NORMAL_ARRAY);
#endif

    //TEXTURE COORD
    GLfloat *new_tex = new GLfloat [g_mesh_->m_vertices_.size() * 2]; // the y coordinate shoule by 1-y , may a bug in qt
    for (int i = 0; i < g_mesh_->m_vertices_.size(); i++)
    {
        new_tex[i * 2 + 0] = g_mesh_->m_texture_coords_[i].x();
        new_tex[i * 2 + 1] = 1 - g_mesh_->m_texture_coords_[i].y();
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[TEXTURE_COORDS]);
    glBufferData(GL_ARRAY_BUFFER, g_mesh_->m_vertices_.size() * 2 * sizeof(GLfloat), new_tex, GL_STATIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    //ELEMENTS
    GLuint * elements = new GLuint [g_mesh_->m_facets_.size() *3];
    for(int i=0; i< g_mesh_->m_facets_.size(); i++)
    {
        elements[i * 3  + 0] = g_mesh_->m_facets_[i][0];
        elements[i * 3  + 1] = g_mesh_->m_facets_[i][1];
        elements[i * 3  + 2] = g_mesh_->m_facets_[i][2];

    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[ELEMENTS]);
    glBufferData(GL_ARRAY_BUFFER, g_mesh_->m_facets_.size()*3*sizeof(GLuint), elements, GL_STATIC_DRAW);
#endif
}


//---------------------------------------------makeTexutures---------------------------------------------//
void SW::GLViewer::makeTextures()
{
}

//----------------------------------------------drawAxises------------------------------------------------//
void SW::GLViewer::drawAxises(double width, double length)
{
    glEnable(GL_LINE_SMOOTH);

    double axisLength = length;

    glLineWidth(width);

    glBegin(GL_LINES);
    {
        // qglColor(Qt::red);

        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(axisLength, 0.0, 0.0);
    }
    glEnd();

    glBegin(GL_LINES);
    {
        //qglColor(Qt::green);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, axisLength, 0.0);
    }
    glEnd();

    glBegin(GL_LINES);
    {
        //qglColor(Qt::blue);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, axisLength);
    }
    glEnd();

    glLineWidth(1.0);;
    glDisable(GL_LINE_SMOOTH);

    //qglColor(Qt::red);
    glColor3f(1.0, 0.0, 0.0);
    //renderText(axisLength, 0.0, 0.0, "X", QFont("helvetica", 12, QFont::Bold, TRUE));

    //qglColor(Qt::green);
    glColor3f(0.0, 1.0, 0.0);
    //renderText(0.0, axisLength, 0.0, "Y", QFont("helvetica", 12, QFont::Bold, TRUE));

    //qglColor(Qt::blue);
    glColor3f(0.0, 0.0, 1.0);
    //renderText(0.0, 0.0, axisLength, "Z", QFont("helvetica", 12, QFont::Bold, TRUE));

}


//--------------------------------------------mousePressEvent---------------------------------------------//
void SW::GLViewer::mousePressEvent(QMouseEvent *e)
{
    QGLViewer::mousePressEvent(e);
}


//---------------------------------------------mouseReleaseEvent-----------------------------------------//
void SW::GLViewer::mouseReleaseEvent(QMouseEvent *e)
{
    QGLViewer::mouseReleaseEvent(e);
}


//---------------------------------------------mouseMoveEvent---------------------------------------------//
void SW::GLViewer::mouseMoveEvent(QMouseEvent *e)
{
    QGLViewer::mouseMoveEvent(e);
}


//----------------------------------------------wheelEvent-----------------------------------------------//
void SW::GLViewer::wheelEvent(QWheelEvent *e)
{
    QGLViewer::wheelEvent(e);
}


//-----------------------------------------------keyPressEvent-------------------------------------------//
void SW::GLViewer::keyPressEvent(QKeyEvent *e)
{
    QGLViewer::keyPressEvent(e);
}


//-------------------------------------------------drawStartingLayer-------------------------------------//
void SW::GLViewer::drawStartingLayer()
{
    glEnable(GL_BLEND);

    // draw quad
    glColor4f(0.0, 0.5, 0.0f, 0.8f);
    glBegin(GL_QUADS);
    foreach(Vec3 pt, g_floorplan_displays_->f_starting_layer_boundary_ )
    {
        glVertex3f(pt.x_, pt.y_, pt.z_);
    }
    glEnd();

    //draw boundary
    glLineWidth(2.0);
    glColor4f(0.4f, 0.9f, 0.1f, 0.5f);
    glBegin(GL_LINE_LOOP);

    QVector<Vec3> boundary = g_floorplan_displays_->f_starting_layer_boundary_;
    for(int i=0; i< boundary.size(); i++)
    {
        int id0 = i;
        int id1 = (i+1)% (int)boundary.size();

        glVertex3f(boundary[id0].x_,  boundary[id0].y_,  boundary[id0].z_);
        glVertex3f(boundary[id1].x_,  boundary[id1].y_,  boundary[id1].z_);
    }
    glEnd();

    glDisable(GL_BLEND);

}


//--------------------------------------------------drawEndingLayer---------------------------------------//
void SW::GLViewer::drawEndingLayer()
{
    // draw quad
    glEnable(GL_BLEND);

    glColor4f(0.5, 0.0, 0.5f, 0.8f);
    glBegin(GL_QUADS);
    foreach(Vec3 pt, g_floorplan_displays_->f_ending_layer_boundary_)
    {
        glVertex3f(pt.x_, pt.y_, pt.z_);
    }
    glEnd();

    // draw boundary
    glLineWidth(2.0);
    glColor4f(0.4f, 0.9f, 0.1f, 0.5f);
    glBegin(GL_LINE_LOOP);

    QVector<Vec3> boundary = g_floorplan_displays_->f_ending_layer_boundary_;
    for(int i=0; i< boundary.size(); i++)
    {
        int id0 = i;
        int id1 = (i+1)% (int)boundary.size();

        glVertex3f(boundary[id0].x_,  boundary[id0].y_,  boundary[id0].z_);
        glVertex3f(boundary[id1].x_,  boundary[id1].y_,  boundary[id1].z_);
    }
    glEnd();

    glDisable(GL_BLEND);
}


//---------------------------------------------------drawSemiplanes---------------------------------------//
// draw semi planed int  the floor plan reconstruction
void SW::GLViewer:: drawSemiPlanes()
{
    glEnable(GL_BLEND);

   // quad facets
    glPushMatrix();
    glColor4f(0.0f, 0.5f, 0.5f, 0.75f);

    foreach (QVector<Vec3> facet, g_floorplan_displays_->f_semi_planes_)
    {
        glBegin(GL_QUADS);
        foreach (Vec3 pt, facet)
        {
            glVertex3f(pt.x_, pt.y_, pt.z_);
        }
        glEnd();

    }


    // draw vertices
    glColor4f(1.0f, 0.0f, 0.0f, 0.75f);
    glPointSize(4);

    glBegin(GL_POINTS);
    foreach (QVector<Vec3> facet, g_floorplan_displays_->f_semi_planes_)
    {
        foreach(Vec3 pt, facet)
        {
            glVertex3f(pt.x_, pt.y_, pt.z_);
        }
    }
    glEnd();

    glPointSize(1);
    glPopMatrix();

    glDisable(GL_BLEND);
}
