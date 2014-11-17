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
    void setDensePointsPtr(QVector<Point>* ptr){ g_points_ = ptr;}
    void setMeshVerticesPtr(QVector<Vec3> * ptr) { g_vertices_ = ptr;}
    void setMeshFacetsPtr(QVector<QVector<uint> > * ptr){g_facets_ = ptr;}
    void setMeshTextureCoordsPtr(QVector<QVector2D> * ptr){ g_texture_coords_ = ptr;}
    void setCamerasPtr(QMap<QString, Camera> * ptr){ g_cameras_ = ptr; }
    void setPtIdsPtr(QVector<uint> * ptr){g_pt_ids_ = ptr;}
    void setFloorPlanDisplay(FloorPlanDisplay * ptr){ g_floorplan_displays_ = ptr;}

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

        QString outputText = QString("%1 points").arg(g_points_->size());
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

        QString outputText = QString("%1 vertices").arg(g_vertices_->size());
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

        QString outputText = QString("%1 vertices %1 facets").arg(g_vertices_->size()).arg(g_facets_->size());
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

        QString outputText = QString("%1 vertices %1 facets").arg(g_vertices_->size()).arg(g_facets_->size());
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

        QString outputText = QString("%1 vertices %1 facets").arg(g_vertices_->size()).arg(g_facets_->size());
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


    // under the floor plan reconstrucion framework
    inline void toggle_display_inconsist_pts(int state)
    {
        bool flag ;
        if(state ==0)flag = false;// unchecked
        if(state ==2) flag = true;// checked
        g_display_inconsist_ = flag;
        g_display_dense_pts_ = !flag;
        updateGL();
    }
    // under the floor plan reconstruction framework
    inline void toggle_display_slices(int state)
    {
        bool flag ;
        if(state ==0) flag = false;// unchecked
        if(state ==2) flag = true;// checked
        g_display_slices_ = flag;
        updateGL();
    }
    // update GLViewer
    inline void updateGLViewer()
    {
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

    //------------------------variables for dense points--------------------------------//
    QVector<Point> *g_points_;            // dense points containes world coordinates, colors,
    // normals, vis data and so on.
    QVector<uint> *g_pt_ids_;             // remaining points ids after deleting points by mannul


    //-------------------------varaibles for mesh---------------------------------------//
    QVector<Vec3> *g_vertices_;            // world coordinates of the vertices in the mesh
    QVector<QVector<uint> > *g_facets_;     // facets of the mesh
    QVector<QVector2D> *g_texture_coords_; // texture coords of each vertex



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
