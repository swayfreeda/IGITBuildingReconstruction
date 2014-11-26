// Description: member functions in PaintImageWidget are implemented in this file
#include"sw_paintImageWidget.h"
#include"sw_functions.h"

#include<iostream>

using namespace SW;


//-------------------------------------------------CLASS MEMBERS--------------------------------------------//




//-------------------------------------------------constructor----------------------------------------------//
PaintImageWidget::PaintImageWidget(QWidget *parent)
    :QWidget(parent)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    setBackgroundColor(Qt::gray);

    p_start_draw_img_ = false;
    p_start_projection_mode_ = false;
    setFocusPolicy(Qt::StrongFocus);

}




//--------------------------------------------------paintevent----------------------------------------------//
void PaintImageWidget::paintEvent(QPaintEvent *event)
{
    // cout<<"x_offset: "<<*p_x_offset_<<endl;

    QPainter painter(this);
    if(p_start_draw_img_ == true)
    {

        // paint the current image which is resize from the original image
        if(p_current_image_.rows!= 0&&  p_current_image_.cols!=0)
        {
            painter.drawImage(0,0, p_img_show_);
        }
    }

    if(p_start_projection_mode_ == true)
    {
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(Qt::red, 2));

        // draw the boundary of the polygon
        foreach(QPolygon polygon, p_polygons_)
        {
            painter.drawPolygon(polygon);
        }

        // draw the vertices of the polygon
        painter.setPen(QPen(Qt::yellow, 2));
        foreach(QPolygon polygon, p_polygons_)
        {
            foreach(QPoint p, polygon)
            {
                painter.drawEllipse(p, 3,3);
            }
        }
    }
#if 0
    if(p_texture_mapping_mode_ ==true)
    {

        // draw selected triangulations
        painter.setPen(QPen(Qt::green, 2));
        painter.setBrush(QBrush(Qt::red, Qt::SolidPattern));

        // draw the projected triangulations on the current image
        if(p_2Dtriangulation_vertices_.size()>0)
        {
            // the selected triangulations
            foreach(int id, p_selected_triangulation_)
            {
                // for each selected triangulations
                QPolygon polygon;
                if(id >= p_current_plane_->p_facets_.size())
                    continue;

                // number of vertices in the triagulations
                int num = p_current_plane_->p_facets_[id].size();
                for(int i=0; i<num; i++)
                {
                    int ver_index = p_current_plane_->p_facets_[id][i];
                    polygon.push_back(p_2Dtriangulation_vertices_[ver_index]);
                }
                painter.drawPolygon(polygon);
            }
        }
    }
}
#endif
//    }
//}
}




//-----------------------------------------------keypressevent--------------------------------------------------//
void PaintImageWidget::keyPressEvent(QKeyEvent * event)
{
    // in the texture mode press the SPACE Key to select a texture image and a start point
    // if( (event->key()== Qt::Key_Space) && p_texture_mapping_mode_ == true)
    // {
    // QMessageBox::information(this, "Selection", tr(current_image_name_+ QString(" is selceted as texture!")));

    // translate the texure coordinates
    // offsetTextureCoordiantes()
    //  }

    //  else{
    //      QMessageBox::information(this,"No Selection", "Click image first and then press Space Key");
    QWidget::keyPressEvent(event);
    //   }
}





//---------------------------------------------startPaintingImg-------------------------------------------------//
void PaintImageWidget::startPaintingImg(QListWidgetItem* item)
{

    // when image changed the factors are to be changed is :

    //*********1.0 current image name ***********//
    p_current_img_name_ = item->text();


    //*********2.0 current camera **************//
    p_current_camera_ = (*p_cameras_)[item->text()];
    //cout<<"proj: "<< p_current_camera_.project_ <<endl;


    //*********3.0 current image **************//
    p_current_image_ = (*p_images_)[item->text()];


    //*********4.0 image for showing **********//
    p_img_show_ = convertToQImage(p_current_image_);
    p_img_show_= p_img_show_.scaled(this->height(), this->width(), Qt::KeepAspectRatio);


    //*********5.0 scale factor **************//
    p_w_scale_  = (float)  p_img_show_.width()/  (float)(*p_images_)[item->text()].cols;
    p_h_scale_ = (float)  p_img_show_.height()/ (float)(*p_images_)[item->text()].rows;


    // if image changes the projected polygons change either
    if( p_start_projection_mode_ == true)
    {
        //---------------------------------------------//
        p_seleceted_trians_.clear();
        for(int i=0; i< p_current_plane_.p_facets_.size(); i++)
        {
            p_seleceted_trians_.append(i);
        }

        //----------------------------------------------//
        p_polygons_.clear();
        makePolygons();
    }

    p_start_draw_img_ = true;
    update();

}



//------------------------------------------startPaintingProjecting---------------------------------------------//
void PaintImageWidget::startPainitingProjecting(bool flag)
{
    p_start_projection_mode_ = flag;

    if(flag == true)
    {

        // if no image or no plane happens
        if(p_current_image_.cols==0 || p_current_image_.rows==0|| p_current_plane_.p_vertices_.size()==0)
        {
            QMessageBox::warning(this, tr("Warning"), tr("Select An Image And A Plane First!"));
        }
        else{

            p_start_draw_img_ ==  true;
            //---------------------------------------------//
            p_seleceted_trians_.clear();
            for(int i=0; i< p_current_plane_.p_facets_.size(); i++)
            {
                p_seleceted_trians_.append(i);
            }
            //----------------------------------------------//
            p_polygons_.clear();
            makePolygons();

            update();
        }
    }
    else {  // flag == false

        p_seleceted_trians_.clear();
        p_polygons_.clear();
    }

    update();
}




//----------------------------------------------makePolygons---------------------------------------------//
void PaintImageWidget::makePolygons()
{
    // if No Camreas
    if(p_current_camera_.project_.cols==0 || p_current_camera_.project_.rows==0)
    {
        QMessageBox::warning(this, tr("Warning"), tr("No Cameras! Select An Image"));
    }
    // if No images
    else if(p_current_image_.rows ==0 || p_current_image_.cols ==0 )
    {
        QMessageBox::warning(this, tr("Warning"), tr("No Images! Select An Image!"));
    }
    else
    {
        // compute the porjected coordinates of all the vertices in plane
        QVector<QPoint> vertices2D;
        foreach(Vec3 pt, p_current_plane_.p_vertices_)
        {

            PointXY pt2D =  p_current_camera_.project(pt);
            vertices2D.append(QPoint(pt2D.x * p_w_scale_, pt2D.y * p_h_scale_));
        }

        // get all the triangles of the mesh
        foreach(uint t_id, p_seleceted_trians_)
        {
            if(t_id> p_seleceted_trians_.size()) continue;

            QPolygon polygon;
            foreach(uint p_id, p_current_plane_.p_facets_[t_id])
            {
                polygon.append(vertices2D[p_id]);
            }
            p_polygons_.append(polygon);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//compute the projection of all the vertices in the plane
// only need to compute when plane or images change
#if 0
void PaintImageWidget::computeTextureProjection()
{
    // projection matrix
    // cv::Mat proj = (*p_projs_)[item->text()];
    // compute projection of triangulation vertices

    p_2Dtriangulation_vertices_.clear();
    p_current_plane_->p_texture_coordinate_.clear();

    if(p_current_proj_.rows==0)
    {
        QMessageBox::warning(this, tr("Warning"), tr("No Projection Matrix"));
    }
    else {
        foreach(Vec3 pt, p_current_plane_->p_vertices_)
        {
            QPoint p;
            projectionFrom3DTo2D(pt, p_current_proj_, p );
            float x = (float)p.x();
            float y = (float)p.y();

            p_current_plane_->p_texture_coordinate_.push_back(PointXY(x,y));

            //scale for display
            p.setX(p.x()* width_scale_);
            p.setY(p.y() * height_scale_);
            p_2Dtriangulation_vertices_.push_back(p);
        }
    }
}

#endif
