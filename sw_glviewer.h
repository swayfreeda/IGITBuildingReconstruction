#ifndef SW_GLVIEWER_H
#define SW_GLVIEWER_H

//#define VERTEX_ARRAY //  change ot VERTEX_ARRAY display Mode

#include"sw_dataType.h"
#include"sw_floorplan.h"

#include"QGLViewer/qglviewer.h"

#include<list>
#include<vector>
#include<QObject>
#include<QMouseEvent>
#include<QMenu>
#include<QAction>
# include <QMenu>
#include <qcursor.h>
#include <qmap.h>
#include <math.h>
#include<QColor>
#include<QListWidgetItem>
#include <QImage>
#include<QKeyEvent>

#include<QVector2D>
#include<QVector3D>
#include"opencv2/opencv.hpp"
#include<QThread>


typedef PointXYZRGBNormal Point;

namespace SW{

//----------------------------------------------CLASS QGLViewer------------------------------------------------//
class  GLViewer : public QGLViewer
{
    Q_OBJECT

public:

    // Points are the original dense points generated from SFM and CMVS
    // Mesh are the vertices and facets generated from poisson surface reconstruction
    enum {POINTS, MESH, NumVAOs};

    //constructor
    //NOTE: Different versions of Qts have different versions of Constructors
    GLViewer(QWidget *parent0=0, const QGLWidget *parent1=0, Qt::WFlags f = 0);
    ~GLViewer();

    virtual void init();
    virtual void draw();
    virtual QString helpString();
    virtual void transform(){}

    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void wheelEvent(QWheelEvent *e);
    virtual void keyPressEvent(QKeyEvent *e);


    void viewAll();
    // compute the bounding box of the whole scene
    void computeSceneBoundingBox();


    //------------------------------DISPLAY FUNCTIONS------------------------------------//
    //draw world coordinates
    void drawAxises(double width, double length);

    // draw cameras
    void drawCameras();

    // create vertex arrays and buffers for dense points
    void makeDensePoints();

    // create vertex arrays and buffers for meshes
    void makeMeshes();

    // create textures
    void makeTextures();

    //------------------------------INTERFACES-------------------------------------------//
    void setPointCloudPtr(PointCloud * ptr){g_pc_ = ptr;}
    void setMeshPtr(Mesh * ptr){ g_mesh_ = ptr; }
    void setCamerasPtr(QMap<QString, Camera> * ptr){ g_cameras_ = ptr; }
    void setPlane3DPtr(QMap<QString, Plane3D> * ptr){ g_plane3Ds_ = ptr;}
    void setFloorPlanDisplay(FloorPlanDisplay * ptr){ g_floorplan_displays_ = ptr;}
    void setCurrentPlane3D(Plane3D * ptr){  g_current_plane3D_ptr_ =  ptr;}

protected:

    // draw starting layer in the floorplan reconstruction
    void drawStartingLayer();

    // draw endding layer in the floorplan reconstruction
    void drawEndingLayer();

    // draw semi planed int  the floor plan reconstruction
    void  drawSemiPlanes();

public slots:

    virtual void drawText(){}

    //-------------------------------DISPLAY FUNCTIONS-----------------------------------//
    // display dense points
    inline void toggle_display_points(bool flag)
    {
        viewAll();
        showEntireScene();

        makeDensePoints();

        g_display_dense_pts_ = flag;

        QString outputText = QString("%1 points").arg(g_pc_->ptNum());
        emit statusBar(outputText);

        if (flag == true)
        {
            g_display_vertices_ = false;
            g_display_wire_frame_ = false;
            g_display_flat_ = false;
            g_display_texture_ = false;
        }
        updateGL();
    }

    // display vertices
    inline void toggle_display_vertices(bool flag)
    {
        viewAll();
        showEntireScene();

        g_display_vertices_ = flag;

        QString outputText = QString("%1 vertices").arg(g_mesh_->m_vertices_.size());
        emit statusBar(outputText);

        if (flag == true)
        {
            g_display_dense_pts_ = false;
            g_display_wire_frame_ = false;
            g_display_flat_ = false;
            g_display_texture_ = false;
        }
        updateGL();
    }

    // display wire frame
    inline void toggle_display_wire_frame(bool flag)
    {

        viewAll();
        showEntireScene();
        g_display_wire_frame_ = flag;

        QString outputText = QString("%1 vertices %1 facets").arg(g_mesh_->m_vertices_.size()).arg(g_mesh_->m_facets_.size());
        emit statusBar(outputText);

        if (flag == true)
        {
            g_display_dense_pts_ = false;
            g_display_vertices_ = false;
            g_display_flat_ = false;
            g_display_texture_ = false;
        }
        updateGL();
    }

    // display flat
    inline void toggle_display_flat(bool flag)
    {
        viewAll();
        showEntireScene();
        g_display_flat_ = flag;

        QString outputText = QString("%1 vertices %1 facets").arg(g_mesh_->m_vertices_.size()).arg(g_mesh_->m_facets_.size());
        emit statusBar(outputText);

        if (flag == true)
        {
            g_display_dense_pts_ = false;
            g_display_vertices_ = false;
            g_display_wire_frame_ = false;
            g_display_texture_ =false;
        }
        updateGL();
    }

    // display texture
    inline void toggle_display_texture(bool flag)
    {
        viewAll();
        showEntireScene();
        g_display_texture_ = flag;

        QString outputText = QString("%1 vertices %1 facets").arg(g_mesh_->m_vertices_.size()).arg(g_mesh_->m_facets_.size());
        emit statusBar(outputText);

        if (flag == true)
        {
            if (g_texture_id_ == -1)
            {
                makeTextures();
            }
            g_display_dense_pts_ = false;
            g_display_vertices_ = false;
            g_display_wire_frame_ =false;
            g_display_flat_ =false;
        }
        updateGL();
    }

    // display cameras
    inline void toggle_display_cameras(bool flag)
    {
        viewAll();
        showEntireScene();
        g_display_cameras_ = flag;

        QString outputText = QString("%1 vertices cameras").arg(g_cameras_->size());
        emit statusBar(outputText);
        updateGL();
    }

    // display the inconsistent regions, under the floor plan reconstrucion framework
    inline void toggle_display_inconsist_pts(int state)
    {
        bool flag ;
        if(state ==0)flag = false;// unchecked
        if(state ==2) flag = true;// checked
        g_display_inconsist_ = flag;
        g_display_dense_pts_ = !flag;
        updateGL();
    }

    // display the slices data of each layer, under the floor plan reconstruction framework
    inline void toggle_display_slices(int state)
    {
        bool flag ;
        if(state ==0) flag = false;// unchecked
        if(state ==2) flag = true;// checked
        g_display_slices_ = flag;
        updateGL();
    }

    // display the modelling process, under the floor plan reconstruction framework,
    inline void toggle_display_modelling_process(int state)
    {
        bool flag ;
        if(state ==0) flag = false;// unchecked
        if(state ==2) flag = true;// checked
        g_display_modelling_process_ = flag;

        updateGL();
    }

    // display the modelling results, under the floor plan reconstruction framework
    inline void toggle_display_modellding_results(int state)
    {
        bool flag ;
        if(state ==0) flag = false;// unchecked
        if(state ==2) flag = true;// checked
        g_display_modelling_results_ = flag;

        if(g_display_modelling_results_ == true)
        {
           g_display_dense_pts_ = !flag;
        }

        updateGL();
    }

    // display the triangulations on all planes
    inline void toggle_display_all_planes_trians(bool flag)
    {
        //viewAll();
        //showEntireScene();
        g_display_all_planes_triangulations_ = flag;

        if(g_display_all_planes_triangulations_== true)
        {
            g_display_modelling_results_  = false;
            g_display_single_plane_triangulations_ = false;
        }
        QString outputText = QString("%1 planes").arg(g_plane3Ds_->size());
        emit statusBar(outputText);

        updateGL();
    }


    // display the triangulations on all planes
    inline void toggle_display_single_plane_trians(bool flag)
    {
        g_display_single_plane_triangulations_ =flag;

        QString outputText = QString("%1 planes").arg(g_plane3Ds_->size());
        emit statusBar(outputText);


        if(g_display_single_plane_triangulations_== true)
        {
          g_display_all_planes_triangulations_ = false;
          g_display_modelling_results_ = true;
        }

        updateGL();
    }


    // update GLViewer
    inline void updateGLViewer()
    {
      updateGL();
    }


    // start display back projected quads
    inline void startDisplayBackPorjQuads()
    {
       g_display_back_projected_quads_ =true;
       updateGL();
    }

    // end display back projectd quads
    inline void endDisplayBackProjQuads()
    {
      g_display_back_projected_quads_ = false;
      updateGL();
    }

    // start display  added window planes
    inline void startDisplayAddedWindowPlanes()
    {
      g_display_added_widow_planes_ = true;
      updateGL();
    }

    // end display added window planes
    inline void endDisplayAddedWindowPlanes()
    {
      g_display_added_widow_planes_  =false;
      updateGL();
    }


signals:

    void statusBar(QString info);

private:

    //------------------------display related bool variables----------------------------//
    bool g_display_dense_pts_;
    bool g_display_vertices_;
    bool g_display_wire_frame_;
    bool g_display_flat_;
    bool g_display_texture_;
    bool g_display_cameras_;

    bool g_display_inconsist_;
    bool g_display_slices_;
    bool g_display_modelling_process_;
    bool g_display_modelling_results_;
    bool g_display_all_planes_triangulations_;
    bool g_display_single_plane_triangulations_;
    bool g_display_back_projected_quads_;
    bool g_display_added_widow_planes_ ;


    //------------------------variables for dense points--------------------------------//
    // pointcloud constaines points with normals, rgbs, positions and visibilities
    // also there is a function for computing the center  of the pointcloud
    // and a function for computing the bounding box of the pointcloud
    PointCloud *g_pc_;

    //-------------------------varaibles for mesh---------------------------------------//
    // g_mesh containes the vertices, facets and edges of the mesh
    // edges are calculated from vertices and facets
     Mesh *g_mesh_;



    //------------------------variables for plane3D--------------------------------------//
    // g_plane3Ds_ containes all the plane structures in the scene, inlcuding:
    // *normal of the plane
    // *parameters of the plane
    // *vertices that are in the plane
    // *texture_coordinates of the verices
    // *facets of the mesh
    // *boundaries of the plane
    // *coordinates system of the plane
    // *window boundaries of the plane
    QMap<QString, Plane3D > *g_plane3Ds_;


    // current plane3D
    Plane3D * g_current_plane3D_ptr_;


    //-------------------------variables for vertex array and buffers---------------------//
    // We set two basic vertex array, the first is for the dense points and the the second
    // is for the mesh
#ifdef VERTEX_ARRAY
    GLuint g_VAOs_[NumVAOs];
#endif

    QVector3D g_bounding_left_top_;
    QVector3D g_bounding_right_bottom_;


    //---------------------------variables for texture -------------------------------------//
    //texture index
    GLuint g_texture_id_;


    //---------------------------variable for cameras----------------------------------------//
    QMap<QString, Camera> *g_cameras_;

    //---------------------------variable for display-----------------------------------------//
    FloorPlanDisplay *g_floorplan_displays_;
};

}
#endif // SW_GLVIEWER_H
