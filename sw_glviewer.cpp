
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

SW::GLViewer::GLViewer(QWidget *parent0, const QGLWidget *parent1, Qt::WFlags f): QGLViewer(parent0, parent1, f)
{
    setAutoFillBackground(true);

    g_display_dense_pts_ = false;
    g_display_vertices_ = false;
    g_display_wire_frame_ = false;
    g_display_flat_ = false;
    g_display_texture_ = false;
    g_display_cameras_ = false;

}
/////////////////////////////////////////////////////////////////////////////////////////////////
SW::GLViewer::~GLViewer()
{

}

////////////////////////////////////NON CLASS METHOD///////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::GLViewer::draw()
{


    //drawAxises(0.1, 0.1);

    // draw dense points
    if(g_display_dense_pts_== true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(POINTS);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
        glDrawElements(GL_POINTS, g_points_->size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
        foreach(Point pt, * g_points_)
        {
            glColor3f(pt.r/255.0, pt.g/255.0, pt.b/255.0);
            glBegin(GL_POINTS);
            glVertex3f(pt.x, pt.y, pt.z);
            glEnd();
        }
#endif

    }

    // draw mesh vertices
    if(g_display_vertices_ == true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(g_VAOs_[MESH]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        glDrawElements(GL_TRIANGLES, g_facets_->size()*3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
        for(int i=0; i< g_vertices_->size(); i++)
        {
           glColor3f(0.5, 0.5, 0.5);
           glBegin(GL_POINTS);
           glVertex3f((*g_vertices_)[i].x_, (*g_vertices_)[i].y_, (*g_vertices_)[i].z_);
           glEnd();
        }
#endif
    }

    // draw mesh wire frame
    if(g_display_wire_frame_ == true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(g_VAOs_[MESH]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, g_facets_->size()*3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
       // must get edges......
#endif
    }

    // draw mesh flat
    if(g_display_flat_ == true)
    {
#ifdef VERTEX_ARRAY
        glBindVertexArray(g_VAOs_[MESH]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, g_facets_->size()*3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
#else
        foreach(QVector<int> facet, *g_facets_)
        {
            glColor3f(0.5, 0.5, 0.5);
            glBegin(GL_TRIANGLES);
            foreach(int id, facet)
            {
                glVertex3f((*g_vertices_)[id].x_, (*g_vertices_)[id].y_, (*g_vertices_)[id].z_);
            }
            glEnd();
        }
#endif
    }

    // draw texture
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
    glFlush();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::GLViewer::computeSceneBoundingBox()
{
    g_bounding_left_top_.setX(100000);
    g_bounding_left_top_.setY(100000);
    g_bounding_left_top_.setZ(100000);
    g_bounding_right_bottom_.setX(-100000);
    g_bounding_right_bottom_.setY(-100000);
    g_bounding_right_bottom_.setZ(-100000);

    // bounding box of the dense points
    foreach(Point pt, *g_points_)
    {
        if (g_bounding_left_top_.x() > pt.x)g_bounding_left_top_.setX(pt.x);
        if (g_bounding_left_top_.y() > pt.y)g_bounding_left_top_.setY(pt.y);
        if (g_bounding_left_top_.z() > pt.z)g_bounding_left_top_.setZ(pt.z);

        if (g_bounding_right_bottom_.x() < pt.x) g_bounding_right_bottom_.setX(pt.x);
        if (g_bounding_right_bottom_.y() < pt.y) g_bounding_right_bottom_.setY(pt.y);
        if (g_bounding_right_bottom_.z() < pt.z) g_bounding_right_bottom_.setZ(pt.z);
    }
    // bounding box of the meshes
    foreach(Vec3 pt, *g_vertices_)
    {
        if (g_bounding_left_top_.x() > pt.x_)g_bounding_left_top_.setX(pt.x_);
        if (g_bounding_left_top_.y() > pt.y_)g_bounding_left_top_.setY(pt.y_);
        if (g_bounding_left_top_.z() > pt.z_)g_bounding_left_top_.setZ(pt.z_);

        if (g_bounding_right_bottom_.x() <  pt.x_) g_bounding_right_bottom_.setX(pt.x_);
        if (g_bounding_right_bottom_.y() <  pt.y_) g_bounding_right_bottom_.setY(pt.y_);
        if (g_bounding_right_bottom_.z() <  pt.z_) g_bounding_right_bottom_.setZ(pt.z_);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::GLViewer::makeDensePoints()
{

#ifdef VERTEX_ARRAY
    enum{Vertices, Color, Elements, NumVBOs};
    GLuint buffers[NumVBOs];

    // active a vertex array
    glBindVertexArray(POINTS);
    glGenBuffers(NumVBOs, buffers);

    GLfloat * vertices = new GLfloat [ g_points_->size() * 3];
    for(int i=0; i< g_points_->size(); i++)
    {
        vertices[i* 3 + 0] =  (GLfloat)(*g_points_)[i].x;
        vertices[i* 3 + 1] =  (GLfloat)(*g_points_)[i].y;
        vertices[i* 3 + 2] =  (GLfloat)(*g_points_)[i].z;

    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Vertices]);
    glBufferData(GL_ARRAY_BUFFER, g_points_->size()* sizeof(GLfloat), vertices, GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);


    // COLOR
    GLfloat * colors = new GLfloat [ g_points_->size() * 3];
    for(int i=0; i< g_points_->size(); i++)
    {
        colors[i* 3 + 0] =  (GLfloat)(*g_points_)[i].r/(GLfloat)255;
        colors[i* 3 + 1] =  (GLfloat)(*g_points_)[i].g/(GLfloat)255;
        colors[i* 3 + 2] =  (GLfloat)(*g_points_)[i].b/(GLfloat)255;
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Color]);
    glBufferData(GL_ARRAY_BUFFER, g_points_->size()*sizeof(GLfloat), colors, GL_STATIC_DRAW);
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
    GLuint * elements = new GLuint [g_points_->size()];
    for(int i=0; i< g_points_->size(); i++)
    {
        elements[i] = (GLuint)i;
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[Elements]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, g_points_->size()*sizeof(GLuint), elements, GL_STATIC_DRAW);
#endif
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::GLViewer::makeMeshes()
{
#ifdef VERTEX_ARRAY
    enum {VERTICES, COLORS, TEXTURE_COORDS, ELEMENTS, NUMVBOS};
    GLuint buffers[NUMVBOS];

    glBindVertexArray(g_VAOs_[MESH]);
    glGenBuffers(NUMVBOS, buffers);

    // VERTICES
    GLfloat * vertices = new GLfloat [g_vertices_->size() * 3];
    for(int i=0; i< g_vertices_->size(); i++)
    {
        vertices[i* 3 + 0] = (*g_vertices_)[i].x_;
        vertices[i* 3 + 1] = (*g_vertices_)[i].y_;
        vertices[i* 3 + 2] = (*g_vertices_)[i].z_;
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[VERTICES]);
    glBufferData(GL_ARRAY_BUFFER, g_vertices_->size() * 3 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

    // COLOR
    GLfloat * colors = new GLfloat [g_vertices_->size()* 3];
    for(int i=0; i< g_vertices_->size(); i++)
    {
        colors[i * 3 + 0] = (GLfloat)0.5;
        colors[i * 3 + 1] = (GLfloat)0.5;
        colors[i * 3 + 2] = (GLfloat)0.5;
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[COLORS]);
    glBufferData(GL_ARRAY_BUFFER, g_vertices_->size() * 3 * sizeof(GLfloat), colors, GL_STATIC_DRAW);
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
    GLfloat *new_tex = new GLfloat [g_vertices_->size() * 2]; // the y coordinate shoule by 1-y , may a bug in qt
    for (int i = 0; i < g_vertices_->size(); i++)
    {
        new_tex[i * 2 + 0] = (*g_texture_coords_)[i].x();
        new_tex[i * 2 + 1] = 1 - (*g_texture_coords_)[i].y();
    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[TEXTURE_COORDS]);
    glBufferData(GL_ARRAY_BUFFER, g_vertices_->size() * 2 * sizeof(GLfloat), new_tex, GL_STATIC_DRAW);
    glTexCoordPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    //ELEMENTS
    GLuint * elements = new GLuint [g_facets_->size() *3];
    for(int i=0; i< g_facets_->size(); i++)
    {
        elements[i * 3  + 0] = (*g_facets_)[i][0];
        elements[i * 3  + 1] = (*g_facets_)[i][1];
        elements[i * 3  + 2] = (*g_facets_)[i][2];

    }
    glBindBuffer(GL_ARRAY_BUFFER, buffers[ELEMENTS]);
    glBufferData(GL_ARRAY_BUFFER, g_facets_->size()*3*sizeof(GLuint), elements, GL_STATIC_DRAW);
#endif
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::GLViewer::makeTextures()
{
}
////////////////////////////////////////////////////////////////////////////////////////////////////
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

void SW::GLViewer::mousePressEvent(QMouseEvent *e)
{
    QGLViewer::mousePressEvent(e);
}
void SW::GLViewer::mouseReleaseEvent(QMouseEvent *e)
{
    QGLViewer::mouseReleaseEvent(e);
}
void SW::GLViewer::mouseMoveEvent(QMouseEvent *e)
{
    QGLViewer::mouseMoveEvent(e);
}
void SW::GLViewer::wheelEvent(QWheelEvent *e)
{
    QGLViewer::wheelEvent(e);
}
void SW::GLViewer::keyPressEvent(QKeyEvent *e)
{
    QGLViewer::keyPressEvent(e);
}
