#include"sw_floorplan.h"
#include"sw_functions.h"
#include"sw_cornerExtraction.h"
#include"sw_codingEdit.h"

#include<QtGui>

//float Letter::TR_ = 0.01;

//--------------------------------------computeCurvatures-----------------------------------------------//
// compute the curvatures of each point in the point cloud
void SW::InconsistenRegionDetector::computeCurvatures()
{

    p_curvatures_.clear();

    QVector<uint>  pts_ids = p_floor_plan_->p_pc_->p_pt_ids_;
    QVector<Point> pts = p_floor_plan_->p_pc_->p_points_;

    cv::Mat points(pts_ids.size(),3,CV_32FC1);
    cv::Mat query(pts_ids.size(),3,CV_32FC1);
    cv::Mat dists;
    cv::Mat indices;

    int index = 0;
    foreach(uint it, pts_ids)
    {
        points.at<float>(index, 0) = pts[it].x;
        points.at<float>(index, 1) = pts[it].y;
        points.at<float>(index, 2) = pts[it].z;

        query.at<float>(index, 0) = pts[it].x;
        query.at<float>(index, 1) = pts[it].y;
        query.at<float>(index, 2) = pts[it].z;

        index++;
    }

    cv::flann::KDTreeIndexParams params(4);
    cv::flann::Index neighbours_search(points, params);
    neighbours_search.knnSearch(query, indices, dists, p_Knn_, cv::flann::SearchParams(128));

    ////////////////////////////////////////////////////////////////////////////
    vector<vector<float> > Singulars;
    vector<float> Singular;
    Singular.resize(3);
    Singulars.resize(pts_ids.size(), Singular);

    cv::Mat Covar_matrix(3,3,CV_32FC1);
    cv::Mat Sample(3,1,CV_32FC1);
    cv::Mat Mean(3,1,CV_32FC1);
    cv::Mat S,U,V;


    for(int i=0; i< indices.rows; i++)
    {
        Mean.setTo(0);
        for(int j=0; j<indices.cols; j++)
        {
            int point_index = indices.at<int>(i,j);//////////////////////type////////////////////
            Mean.at<float>(0)  += pts[point_index].x/p_Knn_;
            Mean.at<float>(1)  += pts[point_index].y/p_Knn_;
            Mean.at<float>(2)  += pts[point_index].z/p_Knn_;
        }

        Covar_matrix.setTo(0);
        for(int j=0; j<indices.cols; j++)
        {
            int point_index = indices.at<int>(i,j);

            Sample.at<float>(0) =  pts[point_index].x;
            Sample.at<float>(1) =  pts[point_index].y;
            Sample.at<float>(2) =  pts[point_index].z;

            Sample -= Mean;

            Covar_matrix += Sample*Sample.t()/p_Knn_;
            // cout<<Covar_matrix<<endl;
        }

        //SVD decmposition
        cv::SVD::compute(Covar_matrix,S,U,V);

        // Singular values
        for(int k=0; k<3; k++)
        {
            Singular[k] = S.at<float>(k);
        }

        //cout<<"S: "<<S<<endl;
        Singulars[i].swap(Singular);
    }

    for(int i=0; i<Singulars.size(); i++)
    {
        p_curvatures_.push_back(Singulars[i][1]/Singulars[i][2]);
    }

    Covar_matrix.release();
    Sample.release();
    Mean.release();
    S.release();
    U.release();
    V.release();

    points.release();
    query.release();
    dists.release();
    indices.release();

    Singulars.clear();



}


//---------------------------------------getInconsistentRegion------------------------------------------//
// compute the inconsistent region in the point cloud
void SW::InconsistenRegionDetector::getInconsistentRegion()
{
    const QVector<uint>  *pts_ids = &p_floor_plan_->p_pc_->p_pt_ids_;
    QVector<Point> *pts = &p_floor_plan_->p_pc_->p_points_;


    int index = 0;
    foreach(uint it, *pts_ids)
    {
        (*pts)[it].inconsist_ = false;

        if(p_curvatures_[index] < p_threshold_)
        {
            (*pts)[it].inconsist_ = true;
        }
        index ++;
    }

}


//---------------------------------------detect---------------------------------------------------------//
void SW::InconsistenRegionDetector::detect( bool is_knn_changed, bool is_thresh_changed)
{

    if(p_floor_plan_->p_pc_->p_points_.size()==0)
    {
        QMessageBox::warning(0, tr("Warning!"), tr("No Points!"));
    }

    // k values
    p_Knn_ = p_floor_plan_->spinBox_knn->value();

    // k valuse
    p_threshold_ = p_floor_plan_->doubleSpinBox_threshold->value();

    if(is_knn_changed ==true)
    {
        computeCurvatures();
        p_floor_plan_->setKnnChanged(false);
    }

    if(is_thresh_changed == true)
    {
        p_floor_plan_->setThreshChanged(false);
    }

    getInconsistentRegion();
    //delete [] pts_ids;
    //delete [] pts;
    emit updateGLViewer();
    emit enableGettingSlices();
}



//---------------------------------------divideToSlices-------------------------------------------------//
void SW::SlicesDataCaculator::divideToSlices()
{
    QVector<uint>  pts_ids = p_floor_plan_->p_pc_->p_pt_ids_;
    QVector<Point> pts = p_floor_plan_->p_pc_->p_points_;

    double step  = p_floor_plan_->doubleSpinBox_stepValue->value();
    if(step  ==0)
    {
        QMessageBox::warning(0, tr("Warning!"), tr("Step cannot be 0!"));
    }

    else{
        p_floor_plan_->p_slice_pts_.clear();
        p_floor_plan_->p_ycoordinates_.clear();

        p_floor_plan_->p_floorplan_displays_.f_slice_pt_ids_.clear();
        p_floor_plan_->p_floorplan_displays_.f_Y_coords_.clear();

        //------------------------- get the max height of the scene ---------------------//
        double max_height = 0;
        foreach (uint it, pts_ids)
        {
            if(max_height < pts[it].y)
            {
                max_height = pts[it].y;
            }
        }
        //-------------------------get the slices data----------------------------------//
        int layer_id = 0;
        while(1)
        {
            double lower =  layer_id * step;
            double upper = (layer_id +1) *step;

            if(lower >= max_height) break;

            QVector<uint> temp_layer;
            ////float mean_num = 0;

            foreach(uint it, pts_ids)
            {
                qglviewer::Vec normal( pts[it].normal_x,
                                       pts[it].normal_y,
                                       pts[it].normal_z);

                qglviewer::Vec direction(0.0, 1.0, 0.0);

                if(pts[it].inconsist_==false&&abs(normal*direction)< 0.2&&
                        pts[it].y <upper && pts[it].y >lower)
                {
                    temp_layer.push_back(it);
                }
            }
            //// mean_num = (mean_num* p_floor_plan_->p_ycoordinates_.size() +
            ////              temp_layer.size())/(float)(p_floor_plan_->p_ycoordinates_.size()+1);

            ////  if(temp_layer.size()< 0.5* mean_num) break;

            p_floor_plan_->p_slice_pts_.push_back(temp_layer);
            p_floor_plan_->p_ycoordinates_.push_back(lower);

            layer_id ++;
        }

        //-------------------copy the variables to the FloorplanDisplay struct------------------//
        // these variables are used for displaying in glviewers
        foreach(QVector<uint> slice, p_floor_plan_->p_slice_pts_)
        {
            p_floor_plan_->p_floorplan_displays_.f_slice_pt_ids_.append(slice);
        }
        foreach(float y_coord, p_floor_plan_->p_ycoordinates_)
        {
            p_floor_plan_->p_floorplan_displays_.f_Y_coords_.append(y_coord);
        }

        // display the number of slices
        p_floor_plan_->label_displayLayerNumber->setNum((int) p_floor_plan_->p_slice_pts_.size());
        p_floor_plan_->label_displayLayerNumber->setBackgroundColor(Qt::green);

        p_floor_plan_->setIsGettingSlicesFinished(true);
        if(p_floor_plan_->isGettingSlicesFinished()==true&& p_floor_plan_->isComputeMainDirectsFinished()==true)
        {
            emit enableFloorPlanReconstruction();
        }

        emit  updateGLViewer();
        //  pts_ids= 0;
        //  pc = 0;
    }
}



//----------------------------------------computMainDirections------------------------------------------//
void SW::MainDirectionExtractor::computeMainDirections()
{
    m_floor_plan_->p_main_directions_.clear();

    QVector<uint>  pts_ids = m_floor_plan_->p_pc_->p_pt_ids_;
    QVector<Point> pts = m_floor_plan_->p_pc_->p_points_;


    float angle = m_floor_plan_->doubleSpinBox_minAngle->value();

    //------------------------------ collect all the points -----------------------------------------//
    vector<Point> all_points;
    foreach (uint it, pts_ids) {
        if(pts[it].inconsist_ == false)
        {
            all_points.push_back(pts[it]);
            all_points[all_points.size() -1].y = 0;
        }
    }

    p_mean_dist_ =  meanDistance(all_points);
    //********  对法向量进行双边滤波 ****************************************************************/
    bilateralFilterNormal(all_points,6*p_mean_dist_, 15);

    //*******  EM 算法对法向量进行聚类 ************************************************************/
    int nCluster = 10;
    vector<Vec3> normals;
    normals.resize(all_points.size());
    for(int i=0; i<all_points.size(); i++)
    {
        normals[i].x_ = all_points[i].normal_x;
        normals[i].y_ = all_points[i].normal_y;
        normals[i].z_ = all_points[i].normal_z;
    }

    vector<Vec3> centers;
    vector<float> weights;
    vector<int> labels;
    EMClustering(normals, nCluster, centers, weights, labels);

    //    cout<<"Centers Num: " << centers.size()<<endl;
    //    cout<<"Weights: ";
    //    for(int i=0; i< weights.size(); i++)
    //        cout<<weights[i]<<", ";

    //cout<<endl;

    //********  7.0 对聚类后的中心进行合并  ********************************************************/
    vector<Vec3> centers_after_merging;
    vector<float> weights_after_merging;
    vector<int> labels_after_merging;

    centersMerging(centers, weights, labels, centers_after_merging,
                   weights_after_merging, labels_after_merging, angle);
#if 0
    cout<<"Centers Num after Merging: " << centers_after_merging.size()<<endl;
    cout<<"Weights: ";
    for(int i=0; i< weights_after_merging.size(); i++)
        cout<<weights_after_merging[i]<<", ";
    cout<<endl;
#endif

    for(int i=0; i< weights_after_merging.size(); i++)
    {
        if(weights_after_merging[i]> 0.05)  //经验值，保存阈值大于0.05的 聚类中心
            m_floor_plan_->p_main_directions_.push_back(centers_after_merging[i]);
    }

    m_floor_plan_->label_dispMainDirectionNumbers->setNum((int)m_floor_plan_->p_main_directions_.size());
    m_floor_plan_->label_dispMainDirectionNumbers->setBackgroundColor(Qt::green);

    m_floor_plan_->setIsComputeMainDirectsFinsihed (true);
    if(m_floor_plan_->isGettingSlicesFinished()==true&& m_floor_plan_->isComputeMainDirectsFinished()==true)
    {
        emit  enableFloorPlanReconstruction();
    }

    // pts_ids = 0;
    // pc = 0;
}



//----------------------------------------reconstruction------------------------------------------------//
void SW::FloorPlanReconstructor:: reconstruction( )
{
    emit updateGLViewer();

    // initialize all the variables used in the reconstruction
    float step  = m_floor_plan_->doubleSpinBox_stepValue->value();

    QVector<uint>  pts_ids = m_floor_plan_->p_pc_->p_pt_ids_;
    QVector<Point> pts = m_floor_plan_->p_pc_->p_points_;

    QVector<QVector<uint> > * slices_pts = &m_floor_plan_->p_slice_pts_;
    QVector<float> *ycoordinates = &m_floor_plan_->p_ycoordinates_;
    QVector<Vec3> *main_directions = &m_floor_plan_->p_main_directions_;


    QVector<QVector<uint> > *facets = &m_floor_plan_->p_mesh_->m_facets_;
    QVector<Vec3> *vertices = &m_floor_plan_->p_mesh_->m_vertices_;
    QVector<QPair<uint, uint> > *edges = &m_floor_plan_->p_mesh_->m_edges_;
    QVector<Block3> *blocks = &m_floor_plan_->p_blocks_;

    QVector<QVector<Vec3> > *semi_planes=  &m_floor_plan_->p_floorplan_displays_.f_semi_planes_;
    QVector<Vec3>* ending_boundary =   &m_floor_plan_->p_floorplan_displays_.f_ending_layer_boundary_;
    QVector<Vec3>* starting_boundary = &m_floor_plan_->p_floorplan_displays_.f_starting_layer_boundary_;

    facets->clear();
    vertices->clear();
    blocks->clear();

    starting_boundary->clear();
    ending_boundary->clear();
    semi_planes->clear();


    double y_start = (double)getStartingLayer()* step;
    double y_end =   (double)getEndingLayer()* step;


    // starting_boundary and endding_boundary are used for drawing
    starting_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmin(), (float)y_start,
                                      m_floor_plan_->p_pc_->zmin()));
    starting_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmax(), (float)y_start,
                                      m_floor_plan_->p_pc_->zmin()));
    starting_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmax(), (float)y_start,
                                      m_floor_plan_->p_pc_->zmax()));
    starting_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmin(), (float)y_start,
                                      m_floor_plan_->p_pc_->zmax()));

    ending_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmin(), (float)y_end + step,
                                    m_floor_plan_->p_pc_->zmin()));
    ending_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmax(), (float)y_end + step,
                                    m_floor_plan_->p_pc_->zmin()));
    ending_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmax(), (float)y_end + step,
                                    m_floor_plan_->p_pc_->zmax()));
    ending_boundary->push_back(Vec3(m_floor_plan_->p_pc_->xmin(), (float)y_end + step,
                                    m_floor_plan_->p_pc_->zmax()));



    //-----------------------------------------------1.0 Get Corners of all Layers--------------------------------------//
    // corners of each slice layers
    vector<vector<vector< PointXYZRGBNormal> > > all_corners;

    // starting points of all curvers in all layers
    vector<vector<PointXYZRGBNormal> > start_pts;
    float all_mean_dist = 0;
    for(int i_s = p_starting_layer_; i_s<p_ending_layer_ + 1; i_s++)
    {
        // -------------------Slice Points---------------------//
        QVector<uint> indices =  (*slices_pts)[i_s];
        QVector<Point> slice_data;
        slice_data.resize(indices.size());
        for(int i=0; i< slice_data.size(); i++)
        {
            int id = indices[i];
            slice_data[i] = pts[id];
            slice_data[i].y = 0;
        }



        //--------------------Mean Distance--------------------//
        float mean_dist = meanDistance(slice_data);
        all_mean_dist += mean_dist;



        // -------------------Extracting Corners---------------//
        vector<Point> slice_data_tmp;
        vector<Vec3> main_directions_tmp;
        foreach(Point pt, slice_data) {
            slice_data_tmp.push_back(pt);
        }
        foreach(Vec3 v, *main_directions){
            main_directions_tmp.push_back(v);
        }

        p_linking_curves_margin_ = m_floor_plan_->doubleSpinBox_curveMargin->value();
        p_RG_line_width_ = m_floor_plan_->doubleSpinBox_lineWidth->value();
        vector<vector<Point> > corners =  cornerExtraction(slice_data_tmp,
                                                           mean_dist,
                                                           p_RG_line_width_,
                                                           p_linking_curves_margin_,
                                                           main_directions_tmp);



        //---------------------Semi-planes-----------------------//
        for(int i=0; i<corners.size(); i++ )
        {
            vector<PointXYZRGBNormal> sub_corners = corners[i];
            if(sub_corners.size()==0) continue;

            for(int j=0; j< sub_corners.size(); j++)
            {

                float id0 = j%sub_corners.size();
                float id1 = (j+1)%sub_corners.size();

                QVector<Vec3> boundary;
                Vec3 pt;
                pt.x_ = sub_corners[id0].x;
                pt.y_ = (*ycoordinates)[i_s];
                pt.z_ =  sub_corners[id0].z;
                boundary.append(pt);

                pt.x_ = sub_corners[id1].x;
                pt.y_ = (*ycoordinates)[i_s];
                pt.z_ = sub_corners[id1].z;
                boundary.append(pt);

                pt.x_ = sub_corners[id1].x;
                pt.y_ = (*ycoordinates)[i_s] + step;
                pt.z_ = sub_corners[id1].z;
                boundary.append(pt);

                pt.x_ = sub_corners[id0].x;
                pt.y_ = (*ycoordinates)[i_s] + step;
                pt.z_ = sub_corners[id0].z;
                boundary.append(pt);

                semi_planes->append(boundary);
            }
        }




        //---------------------Sort sub-curves--------------------//
        // 对每一层的多个链接部分进行排序
        Vec3 ref_pt(0,0,0);
        if(corners.size()>1)
        {
            subCurvesPositionAdjustment(corners, ref_pt);
        }


        //----------------------Starting Point Adjustment----------//
        //对起始点进行调整
        startPosAdjustment(corners, ref_pt);
        startDirectionAdjustment(corners);


        //----------------------Recovering Y Coordinates------------//
        for(int i=0; i<corners.size(); i++)
        {
            for(int j=0; j< corners[i].size(); j++)
            {
                corners[i][j].y = (*ycoordinates)[i_s];
            }
        }
        all_corners.push_back(corners);


        //----------------------Starting Points of each curves-------//
        //存储每个获取每个轮廓的起始点
        vector<PointXYZRGBNormal> pts;
        for(int i=0; i< corners.size(); i++)
        {
            // 每一个起始点的y坐标都是最低层的y坐标
            corners[i][0].y = (*ycoordinates)[i_s];
            pts.push_back(corners[i][0]);
        }
        start_pts.push_back(pts);

        //emit signals to update QGLViewer
        emit updateGLViewer();
    }

    // mean distance between all layers of slices
    all_mean_dist /= (p_ending_layer_ + 1 -  p_starting_layer_);

    //Margin for letter
    p_Letter_margin_ =  m_floor_plan_->doubleSpinBox_letterMargin->value();

    // Letters
    Letter::TR( p_Letter_margin_ );
    cout<<"p_letter_matgin: "<< p_Letter_margin_<<endl;


    //------------------------------------------------- 2.0 Make Dictionary---------------------------------------------//
    // each letter is a segment, we statistics all the letters and find the representive ones as the dictionary
    vector<vector< PointXYZRGBNormal> > corners_tmp;
    for(int i=0; i< all_corners.size(); i++)
    {
        for(int j=0; j< all_corners[i].size();j++)
        {
            corners_tmp.push_back(all_corners[i][j]);
        }
    }
    map<Letter, string> table = make_dictionary( corners_tmp);
    for(map< Letter, string>::iterator iter = table.begin();
        iter!= table.end(); iter++)
    {
        cout<< "Key: "<< iter->first;
        cout<< "----->";
        cout<< "Value: " << iter-> second<< endl;
    }



    //------------------------------------------------- 3.0 Coding------------------------------------------------------//
    // assign a string to each layer
    vector<string> words = convertCurvesToWords(all_corners, table);
    for(int i=0; i< words.size(); i++)
    {
        cout<<"num: "<<words[i].size();
        cout<<" Code: ";
        cout<<words[i];
        cout<<endl;
    }


    //------------------------------------------------- 4.0 Clustering---------------------------------------------------//
    // clustering for all the layers
    //4.0 对编码进行聚类，获取标签个数，用于多目标的分配
    vector<float> weights;
    vector<vector<PointXYZRGBNormal> > centers_pos;
    vector<string> centers = codeCentersClustering(words, weights, start_pts, centers_pos );
    cout<<"Before Optimization: "<<endl;
    for(int i=0; i< centers.size(); i++)
    {
        cout<<centers[i]<<",  weight: "<<weights[i]<<endl;
    }


    //------------------------------------------------- 5.0 MRF Optimization---------------------------------------------//
    float sim_thresh = 0.5;
    p_MRF_lambda_ = m_floor_plan_->doubleSpinBox_MRFLambda->value();
    vector<string>results = multiLabelAssignment( words, centers, weights,start_pts,
                                                  centers_pos ,sim_thresh, p_MRF_lambda_);
    for(int i=0; i< results.size(); i++)
    {
        cout<<i<< "the layer: " <<results[i]<<endl;
    }



    //------------------------------------------------- 6.0 Creating Blocks---------------------------------------------//
    //6.0 沿着垂直方向进行搜索，连续相同的轮廓聚类成BLOCK3
    vector<Block3> blocks_temp = getBlocks(results, start_pts, step);

    //reconstruction from the ground
    if(blocks_temp.size()> 0){  blocks_temp[0].start_h_ = 0;}

    for(int i=0; i< blocks_temp.size(); i++)
    {
        cout<<"[" <<endl;
        cout<<blocks_temp[i].word_<<endl;
        cout<<blocks_temp[i].start_h_<< ", "<< blocks_temp[i].end_h_<<endl;
        cout<<endl;
    }


    //------------------------------------------------- 6.0 Creating Blocks---------------------------------------------//
    //6.0 对Blocks 进行解码重建
    map<string, Letter> table_inv;
    for(map<Letter, string> ::iterator iter = table.begin(); iter!= table.end(); iter++)
    {
        table_inv.insert(make_pair(iter->second, iter->first));
    }
    blockReconstruction(blocks_temp, table_inv);


    //------------------------------------------------- 7.0 posProcessing ---------------------------------------------//
    //7.0 对blocks进行后处理
    posProcessing(blocks_temp, 20*all_mean_dist);



    //-------------------------------------------------- 8.0 Processing results ---------------------------------------//
    // 8.1 get results before MRF optimization
    vector<Vec3>mid_vertices;
    vector<vector<int> > mid_facets;
    gettingTriangulationsFromAllCorners(all_corners, mid_vertices, mid_facets);
    writeOFFFiles(mid_vertices, mid_facets, "model/mid_model.off");

    // 8.2 get results after MRF optimization
    vector<Vec3>final_vertices;
    vector<vector<int> > final_facets;
    gettingTriangulationsFromBlocks(blocks_temp, final_vertices, final_facets);
    writeOFFFiles(final_vertices, final_facets, "model/final_model.off");


    //---------------------------------------------------9.0 get final results----------------------------------------//
    // 9.0 get vertices
    for(int i=0; i<final_vertices.size(); i++ )
    {
        vertices->append(final_vertices[i]);
    }

    // 9.1 get facets
    for(int i=0; i< final_facets.size(); i++)
    {
        QVector<uint> facet;
        for(int j=0; j<final_facets[i].size(); j++)
        {
            facet.append(final_facets[i][j]);
        }
        facets->append(facet);
    }

    // 9.2 get edges
    m_floor_plan_->p_mesh_->m_edges_.clear();
    m_floor_plan_->p_mesh_->computeEdges();


    // 9.3 get blocks
    for(int i=0; i< blocks_temp.size(); i++)
    {
        blocks->append(blocks_temp[i]);
    }

    emit enableModelingCeiling();

    //// slices_pts = 0;
    //// ycoordinates = 0;
    //// main_directions = 0;
    //// facets = 0;
    //// vertices = 0;
    //// blocks = 0;
    //// progress = 0;
    //// semi_planes=  0;
    //// ending_boundary= 0;
    //// starting_boundary= 0;
}



//----------------------------------------constructor----------------------------------------------------//
SW::FloorPlanDialog::FloorPlanDialog(QWidget *parent,
                                     PointCloud *pc, Mesh *mesh, QMap<QString, Plane3D> *planes) :p_pc_(pc), p_mesh_(mesh), p_plane3Ds_(planes)
{
    setupUi(this);


    // threshold for Knn
    spinBox_knn->setRange ( 1, 100);
    spinBox_knn->setSingleStep(1);
    spinBox_knn->setValue(0);

    // threshold for inconsistent region
    doubleSpinBox_threshold->setRange(1, 500);
    doubleSpinBox_threshold->setSingleStep(1);
    doubleSpinBox_threshold->setValue(10);


    doubleSpinBox_letterMargin->setValue(0.1);
    doubleSpinBox_letterMargin->setSingleStep(0.01);


    /*****************image number need to set********/
    //spinBox_imageID->setRange(0, 200);
    /***********************************************/

    label_displayLayerNumber->setNum(0);
    label_displayLayerNumber->setBackgroundColor(Qt::yellow);

    label_dispMainDirectionNumbers->setNum(0);
    label_dispMainDirectionNumbers->setBackgroundColor(Qt::yellow);


    p_inconsistent_detector_ = new InconsistenRegionDetector(this);
    p_slices_acculator_ = new SlicesDataCaculator(this);
    p_main_directions_extractor_ = new MainDirectionExtractor(this);
    p_floorplan_constructor_ = new FloorPlanReconstructor(this);


    is_knn_changed_ = false;
    is_thresh_changed_ = false;
    is_gettingSlices_finished_ = false;
    is_computeMainDirects_finished_ = false;

    ////doubleSpinBox_windowDepth->setValue(0.05);
    ////doubleSpinBox_windowDepth->setRange(0,1);
    ////doubleSpinBox_windowDepth->setSingleStep(0.01);

    connect(spinBox_knn, SIGNAL(valueChanged(int)), this, SLOT(KnnChanged()) );
    connect(doubleSpinBox_threshold, SIGNAL(valueChanged(double)), this, SLOT(ThreshChanged()));

    // enable group box
    connect(p_inconsistent_detector_, SIGNAL(enableGettingSlices()), this, SLOT(enableGroupBoxGettingSlices()));
    connect(p_slices_acculator_, SIGNAL(enableFloorPlanReconstruction()), this, SLOT(enableGroupBoxFloorPlanReconstrucion()));
    connect(p_main_directions_extractor_, SIGNAL(enableFloorPlanReconstruction()), this, SLOT(enableGroupBoxFloorPlanReconstrucion()));
    connect(p_floorplan_constructor_, SIGNAL(enableModelingCeiling()), this, SLOT(enableGroupBoxModelingCeiling()));

    // compute inconsistent region
    // if button id pressed, signal to detect will be emitted
    connect(pushButton_insistRegionDetection, SIGNAL(clicked()), this, SLOT(emitDetectionSignal()));
    connect(this, SIGNAL(startToDetect(bool,bool)), p_inconsistent_detector_, SLOT(detect(bool,bool)), Qt::QueuedConnection);
    p_inconsistent_detector_->moveToThread(&p_thread_);
    p_thread_.start();


    // getting slices
    connect(pushButton_divide_slices, SIGNAL(clicked()), p_slices_acculator_, SLOT(divideToSlices()), Qt::QueuedConnection);
    p_slices_acculator_->moveToThread(&p_thread_);
    p_thread_.start();

    // compute main directions
    connect(pushButton_mainDireciongs, SIGNAL(clicked()),p_main_directions_extractor_, SLOT(computeMainDirections()), Qt::QueuedConnection);
    p_main_directions_extractor_->moveToThread(&p_thread_);// thread
    p_thread_.start();


    // floor plan reconstrution
    connect(horizontalSlider_Slayer, SIGNAL(valueChanged(int)), p_floorplan_constructor_, SLOT(starttingLayerChanged(int)));
    connect(horizontalSlider_Elayer,SIGNAL(valueChanged(int)), p_floorplan_constructor_, SLOT(endingLayerChanged(int)));
    connect(doubleSpinBox_lineWidth, SIGNAL(valueChanged(double)), p_floorplan_constructor_, SLOT(lineWidthChanged(double )));
    connect(doubleSpinBox_curveMargin, SIGNAL(valueChanged(double)), p_floorplan_constructor_, SLOT(curveMarginChanged(double)));
    connect(doubleSpinBox_MRFLambda, SIGNAL(valueChanged(double)), p_floorplan_constructor_, SLOT(MRFLambdaChanged(double)));
    connect(doubleSpinBox_letterMargin, SIGNAL(valueChanged(double)), p_floorplan_constructor_, SLOT(LetterMarginChanged(double)));

    connect(pushButton_floorplanRec, SIGNAL(clicked()), p_floorplan_constructor_, SLOT(reconstruction()), Qt::QueuedConnection);
    p_floorplan_constructor_->moveToThread(&p_thread_);
    p_thread_.start();


    // get planes from triangulations
    connect(pushButton_PolygonPlanes, SIGNAL(clicked()), this, SLOT(createPlanesFromTriangulations()));


    //gts detection
    ////connect(pushButton_beginGTSDetection_, SIGNAL(clicked()), this, SLOT(startGTSDetection()));

}




//----------------------------------------createPlanesFromTriangulations---------------------------------//
void SW::FloorPlanDialog::createPlanesFromTriangulations()
{

    //--------------------------------------------1.0  neighbours--------------------------------------------------//
    // 1 neighbours of each facet,
    // facets share a common edge are treated as neighbours
    QVector<QVector<int> > neighbours;
    neighbours.resize(p_mesh_->m_facets_.size());
    for(int i=0; i< p_mesh_->m_facets_.size(); i++)
    {
        // for each facet, accumulate how many common vertices they have
        for(int j=0; j< p_mesh_->m_facets_.size(); j++)
        {
            if(i==j) continue;

            int calc = 0;
            for(int k=0; k< p_mesh_->m_facets_[i].size(); k++)
            {
                // a pair of neighbours shares at least two vertices
                if(p_mesh_->m_facets_[j].contains(p_mesh_->m_facets_[i][k]))
                {
                    calc ++;
                }
            }
            if(calc >=1) neighbours[i].append(j);
        }
    }


    //--------------------------------------------2.0  plane params-------------------------------------------------//
    // 2 compute parameters of each plane, just a line parameters fitting
    QVector<QVector<float> > plane_params;
    plane_params.resize(p_mesh_->m_facets_.size());
    for(int i=0; i<p_mesh_->m_facets_.size(); i++)
    {

        int id0= p_mesh_->m_facets_[i][0];
        int id1= p_mesh_->m_facets_[i][1];
        int id2= p_mesh_->m_facets_[i][2];

        Vec3 pt0 = p_mesh_->m_vertices_[id0];
        Vec3 pt1 = p_mesh_->m_vertices_[id1];
        Vec3 pt2 = p_mesh_->m_vertices_[id2];


        Vec3 v10 = pt1 - pt0;
        Vec3 v21 = pt2 - pt1;

        Vec3 normal = cross(v10, v21);
        normal.normalize();;

        float d = normal* pt0;
        d = -d;

        plane_params[i].append(normal[0]);
        plane_params[i].append(normal[1]);
        plane_params[i].append(normal[2]);
        plane_params[i].append(d);

    }


    //------------------------------------------3.0 region growing algorithm---------------------------------------//
    // 3 region growing algorithm
    QVector<int> used(p_mesh_->m_facets_.size(), 0);
    QVector<int> labels(p_mesh_->m_facets_.size(), -1);
    int cluster_id =0;
    int facet_id = 0;

    while(facet_id< p_mesh_->m_facets_.size())
    {
        QList<int> seeds;

        if(used[facet_id]==1)
        {
            facet_id++;
            continue;
        }

        // 1 select a seed point
        seeds.append(facet_id);
        used[facet_id] =1;
        labels[facet_id] = cluster_id;

        facet_id ++;

        while(seeds.size()> 0)
        {
            int seed_id = seeds[0];
            seeds.pop_front();

            Vec3 normal_s(plane_params[seed_id][0],
                          plane_params[seed_id][1],
                          plane_params[seed_id][2]);

            QVector<int> neigbrs = neighbours[seed_id];

            foreach (int n_id, neigbrs)
            {
                if(used[n_id] ==1 )continue;

                Vec3 normal_n(plane_params[n_id][0],
                              plane_params[n_id][1],
                              plane_params[n_id][2]);

                float pro = normal_n* normal_s;
                pro = min(pro, (float)0.999);
                pro = max(pro, -(float)0.999);
                float angle = acos(pro)*180/3.1415;

                if(angle > 10) continue;

                seeds.push_back(n_id);
                used[n_id] = 1;
                labels[n_id] = cluster_id;

            }
        }
        cluster_id ++;
    }


    //--------------------------------------------4.0 Create Planes-------------------------------------------------//
    // 4 create planes
    // planes are created and corresponding triangulations are added to each plane
    int nCluster = cluster_id;  //number of planes

    // collect facets
    QVector<QVector<int> > clusters;
    clusters.resize(nCluster);
    for(int i=0; i< nCluster; i++)
    {
        for(int j=0; j< labels.size(); j++)
        {
            if(labels[j] ==i)
            {
                clusters[i].append(j);
            }
        }
    }

    for(int i=0; i< clusters.size(); i++)
    {
        Plane3D plane;

        /* name */
        QString name;
        name.sprintf("Plane%d", p_plane3Ds_->size());

        /* color*/
        plane.p_color_ = QColor(rand()&255, rand()&255, rand()&255);


        /* facets and vertices */
        vector<vector< Vec3> > facets;
        for(int j=0; j< clusters[i].size(); j++)
        {
            int facet_id = clusters[i][j];
            vector<Vec3> facet;
            for(int k=0; k< p_mesh_->m_facets_[facet_id].size(); k++)
            {
                int pt_id = p_mesh_->m_facets_[facet_id][k];
                facet.push_back(p_mesh_->m_vertices_[pt_id]);
            }
            facets.push_back(facet);
        }

        /* make sure that point index starts from 0 */
        plane.updateTriangulations(facets);


        /* plane params */
        QSet<int> pt_ids;
        for(int j=0; j< clusters[i].size(); j++)
        {
            int facet_id = clusters[i][j];

            foreach(int id, p_mesh_->m_facets_[facet_id])
            {
                pt_ids.insert(id);
            }
        }

        vector<Vec3> points;
        foreach(int id, pt_ids)
        {
            points.push_back(p_mesh_->m_vertices_[id]);
        }

        // fittingplane from points
        plane.fittingPlane(points);

        //current_plane_->makePlaneFacetsCCW();

        /* boundarys */
        plane.getBoundaryFromTriangulations();

        // eliminate reductant vertices
        // no three points are on the same lines
        plane.boundaryProcessing();

        p_plane3Ds_->insert(name, plane);

        emit createNewPlane(name);
    }
}
