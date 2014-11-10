#include"sw_floorplan.h"
//#include"functions.h"
//#include"cornerExtraction.h"
//#include"codingEdit.h"

#include<QtGui>

//float Letter::TR_ = 0.01;
// compute the curvatures of each point in the point cloud
void SW::InconsistenRegionDetector::computeCurvatures()
{

    p_curvatures_.clear();

    QVector<int> * pts_ids = p_floor_plan_->f_pt_ids_;
    QVector<Point> * pts = p_floor_plan_->f_points_;

    cv::Mat points(pts_ids->size(),3,CV_32FC1);
    cv::Mat query(pts_ids->size(),3,CV_32FC1);
    cv::Mat dists;
    cv::Mat indices;

    int index = 0;
    foreach(int it, *pts_ids)
    {
        points.at<float>(index, 0) = (*pts)[it].x;
        points.at<float>(index, 1) = (*pts)[it].y;
        points.at<float>(index, 2) = (*pts)[it].z;

        query.at<float>(index, 0) = (*pts)[it].x;
        query.at<float>(index, 1) = (*pts)[it].y;
        query.at<float>(index, 2) = (*pts)[it].z;

        index++;
    }

    cv::flann::KDTreeIndexParams params(4);
    cv::flann::Index neighbours_search(points, params);
    neighbours_search.knnSearch(query, indices, dists, p_Knn_, cv::flann::SearchParams(128));

    ////////////////////////////////////////////////////////////////////////////
    vector<vector<float> > Singulars;
    vector<float> Singular;
    Singular.resize(3);
    Singulars.resize(pts_ids->size(), Singular);

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
            Mean.at<float>(0)  += (*pts)[point_index].x/p_Knn_;
            Mean.at<float>(1)  += (*pts)[point_index].y/p_Knn_;
            Mean.at<float>(2)  += (*pts)[point_index].z/p_Knn_;
        }

        Covar_matrix.setTo(0);
        for(int j=0; j<indices.cols; j++)
        {
            int point_index = indices.at<int>(i,j);

            Sample.at<float>(0) =  (*pts)[point_index].x;
            Sample.at<float>(1) =  (*pts)[point_index].y;
            Sample.at<float>(2) =  (*pts)[point_index].z;

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


    delete [] pts_ids;
    delete [] pts;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////
// compute the inconsistent region in the point cloud
void SW::InconsistenRegionDetector::getInconsistentRegion()
{
    QVector<int> * pts_ids = p_floor_plan_->f_pt_ids_;
    QVector<Point>  * pts = p_floor_plan_->f_points_;


    int index = 0;
    foreach(int it, *pts_ids)
    {
        (*pts)[it].inconsist_ = false;

        if(p_curvatures_[index] < p_threshold_)
        {
            (*pts)[it].inconsist_ = true;
        }

        index ++;
    }

    delete [] pts_ids;
    delete [] pts;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void SW::InconsistenRegionDetector::detect( bool is_knn_changed, bool is_thresh_changed)
{

    QVector<int> * pts_ids = p_floor_plan_->f_pt_ids_;
    QVector<Point> * pts = p_floor_plan_->f_points_;

    if(pts->size()==0)
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

    delete [] pts_ids;
    delete [] pts;

    emit enableGettingSlices();
}

#if 0
////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlicesDataCaculator::divideToSlices()
{
    QSet<int> * pts_ids = p_floor_plan_->p_pts_ids_;
    PointCloud * pc = p_floor_plan_->p_pc_;
    double step  = p_floor_plan_->doubleSpinBox_stepValue->value();
    if(step  ==0)
    {
        QMessageBox::warning(0, tr("Warning!"), tr("Step cannot be 0!"));
    }


    p_floor_plan_->p_slice_pts_.clear();
    p_floor_plan_->p_ycoordinates_.clear();

    double max_height = 0;
    foreach (int it, *pts_ids)
    {
        if(max_height<(*pc->Points)[it].y)
        {
            max_height = (*pc->Points)[it].y;
        }
    }
    /*****************获取切片数据*****************/
    int layer_id = 0;
    while(1)
    {
        double lower =  layer_id * step;
        double upper = (layer_id +1) *step;

        if(lower >= max_height) break;

        vector<int> temp_layer;
        float mean_num = 0;

        foreach(int it, *pts_ids)
        {
            qglviewer::Vec normal( (*(pc->Points))[it].normal_x,
                                   (*(pc->Points))[it].normal_y,
                                   (*(pc->Points))[it].normal_z);

            qglviewer::Vec direction(0.0, 1.0, 0.0);

            if((*pc->Points)[it].inconsist_==false&&abs(normal*direction)< 0.2&&
                    (*pc->Points)[it].y <upper && (*pc->Points)[it].y >lower)
            {
                temp_layer.push_back(it);
            }
        }
        mean_num = (mean_num* p_floor_plan_->p_ycoordinates_.size() +
                    temp_layer.size())/(float)(p_floor_plan_->p_ycoordinates_.size()+1);

        if(temp_layer.size()< 0.5* mean_num) break;

        p_floor_plan_->p_slice_pts_.push_back(temp_layer);
        p_floor_plan_->p_ycoordinates_.push_back(lower);


        layer_id ++;
    }

    // display the number of slices
    p_floor_plan_->label_displayLayerNumber->setNum((int) p_floor_plan_->p_slice_pts_.size());
    p_floor_plan_->label_displayLayerNumber->setBackgroundColor(Qt::green);

    p_floor_plan_->setIsGettingSlicesFinished(true);
    if(p_floor_plan_->isGettingSlicesFinished()==true&& p_floor_plan_->isComputeMainDirectsFinished()==true)
    {
        emit enableFloorPlanReconstruction();
    }

    pts_ids= 0;
    pc = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainDirectionExtractor::computeMainDirections()
{
    m_floor_plan_->p_main_directions_.clear();

    QSet<int> * pts_ids = m_floor_plan_->p_pts_ids_;
    PointCloud * pc = m_floor_plan_->p_pc_;

    double angle = m_floor_plan_->doubleSpinBox_minAngle->value();

    /******************计算主方向********************/
    vector<PointXYZRGBNormal> all_points;
    foreach (int it, *pts_ids) {
        if((*pc->Points)[it].inconsist_ == false)
        {
            all_points.push_back((*pc->Points)[it]);
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

    cout<<endl;

    //********  7.0 对聚类后的中心进行合并  ********************************************************/
    vector<Vec3> centers_after_merging;
    vector<float> weights_after_merging;
    vector<int> labels_after_merging;

    centersMerging(centers, weights, labels, centers_after_merging,
                   weights_after_merging, labels_after_merging, angle);

    //    cout<<"Centers Num after Merging: " << centers_after_merging.size()<<endl;
    //    cout<<"Weights: ";
    //    for(int i=0; i< weights_after_merging.size(); i++)
    //        cout<<weights_after_merging[i]<<", ";
    //    cout<<endl;

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

    pts_ids = 0;
    pc = 0;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////
void  FloorPlanReconstructor:: reconstruction( )
{

    double step  = m_floor_plan_->doubleSpinBox_stepValue->value();

    QSet<int> * pts_ids = m_floor_plan_->p_pts_ids_;
    PointCloud * pc = m_floor_plan_->p_pc_;
    vector<vector<int> > * slices_pts = &m_floor_plan_->p_slice_pts_;
    vector<double> *ycoordinates = &m_floor_plan_->p_ycoordinates_;
    vector<Vec3> *main_directions = &m_floor_plan_->p_main_directions_;


    QVector<QVector<int> > *facets = m_floor_plan_->p_facets_;
    QVector<Vec3> *vertices = m_floor_plan_->p_vertices_;
    QVector<QPair<int, int> > *edges = m_floor_plan_->p_edges_;
    QVector<Block3> *blocks = m_floor_plan_->p_blocks_;


   // QProgressBar *progress = m_floor_plan_->progressBar_floorRecon;
    QVector<QVector<Vec3> >* semi_planes=  m_floor_plan_->p_semi_planes_;
    QVector<Vec3>* ending_boundary=   m_floor_plan_->p_ending_layer_boundary_;
    QVector<Vec3>*starting_boundary=  m_floor_plan_->p_starting_layer_boundary_;

    facets->clear();
    vertices->clear();
    blocks->clear();

    starting_boundary->clear();
    ending_boundary->clear();
    semi_planes->clear();

    double y_start = (double)p_starting_layer_ * step;
    double y_end =   (double)p_ending_layer_* step;

    starting_boundary->push_back(Vec3(pc->xmin(), (float)y_start, pc->zmin()));
    starting_boundary->push_back(Vec3(pc->xmax(), (float)y_start, pc->zmin()));
    starting_boundary->push_back(Vec3(pc->xmax(), (float)y_start, pc->zmax()));
    starting_boundary->push_back(Vec3(pc->xmin(), (float)y_start, pc->zmax()));

    ending_boundary->push_back(Vec3(pc->xmin(), (float)y_end + step, pc->zmin()));
    ending_boundary->push_back(Vec3(pc->xmax(), (float)y_end + step, pc->zmin()));
    ending_boundary->push_back(Vec3(pc->xmax(), (float)y_end + step, pc->zmax()));
    ending_boundary->push_back(Vec3(pc->xmin(), (float)y_end + step, pc->zmax()));


    vector<vector<vector< PointXYZRGBNormal> > > all_corners;
    vector<vector<PointXYZRGBNormal> > start_pts;

    float all_mean_dist = 0;
    //int layer_num = p_ending_layer_ - p_starting_layer_;

   // progress->setRange(0, layer_num);

    for(int i_s = p_starting_layer_; i_s<p_ending_layer_ + 1; i_s++)
    {
        vector<int> indices =  (*slices_pts)[i_s];
        vector<PointXYZRGBNormal> slice_data;
        slice_data.resize(indices.size());

        for(int i=0; i< slice_data.size(); i++)
        {
            int id = indices[i];
            slice_data[i] = (*pc->Points)[id];
            slice_data[i].y = 0;
        }

        float mean_dist = meanDistance(slice_data);
        all_mean_dist += mean_dist;

        // 提取顶点
        p_linking_curves_margin_ = m_floor_plan_->doubleSpinBox_curveMargin->value();
        p_RG_line_width_ = m_floor_plan_->doubleSpinBox_lineWidth->value();
        vector<vector<PointXYZRGBNormal> > corners =  cornerExtraction(slice_data, mean_dist,
                                                                       p_RG_line_width_, p_linking_curves_margin_, *main_directions);

        // semi-panes, 用于显示
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

        // 对每一层的多个链接部分进行排序
        Vec3 ref_pt(0,0,0);
        if(corners.size()>1)
        {
            subCurvesPositionAdjustment(corners, ref_pt);
        }

        //对起始点进行调整
        startPosAdjustment(corners, ref_pt);
        startDirectionAdjustment(corners);

        // 获取y坐标
        for(int i=0; i<corners.size(); i++)
        {
            for(int j=0; j< corners[i].size(); j++)
            {
                corners[i][j].y = (*ycoordinates)[i_s];
            }
        }
        all_corners.push_back(corners);

        //存储每个获取每个轮廓的起始点
        vector<PointXYZRGBNormal> pts;
        for(int i=0; i< corners.size(); i++)
        {
            // 每一个起始点的y坐标都是最低层的y坐标
            corners[i][0].y = (*ycoordinates)[i_s];
            pts.push_back(corners[i][0]);
        }
        start_pts.push_back(pts);

       // progress->setValue(i_s - p_starting_layer_);
       // qApp->processEvents();
    }

    all_mean_dist /= (p_ending_layer_ + 1 -  p_starting_layer_);

    p_Letter_margin_ =  m_floor_plan_->doubleSpinBox_letterMargin->value();

    Letter::TR( p_Letter_margin_ );
    cout<<"p_letter_matgin: "<< p_Letter_margin_<<endl;


    // 2.0 创建字典
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

    //2.1 对所有的轮廓进行编码
    vector<string> words = convertCurvesToWords(all_corners, table);

    for(int i=0; i< words.size(); i++)
    {
        cout<<"num: "<<words[i].size();
        cout<<" Code: ";
        cout<<words[i];
        cout<<endl;
    }

    //3.0 对编码进行聚类，获取标签个数，用于多目标的分配
    vector<float> weights;
    vector<vector<PointXYZRGBNormal> > centers_pos;
    vector<string> centers = codeCentersClustering(words, weights, start_pts, centers_pos );
    cout<<"Before Optimization: "<<endl;
    for(int i=0; i< centers.size(); i++)
    {
        cout<<centers[i]<<",  weight: "<<weights[i]<<endl;
    }

    float sim_thresh = 0.5;

    p_MRF_lambda_ = m_floor_plan_->doubleSpinBox_MRFLambda->value();

    vector<string>results = multiLabelAssignment( words, centers, weights,start_pts,
                                                  centers_pos ,sim_thresh, p_MRF_lambda_);
    for(int i=0; i< results.size(); i++)
    {
        cout<<i<< "the layer: " <<results[i]<<endl;
    }

    //5.0 沿着垂直方向进行搜索，连续相同的轮廓聚类成BLOCK3
    vector<Block3> blocks_tmep = getBlocks(results, start_pts, step);

    //reconstruction from the ground
    if(blocks_tmep.size()> 0){  blocks_tmep[0].start_h_ = 0;}

    for(int i=0; i< blocks_tmep.size(); i++)
    {
        cout<<"[" <<endl;
        cout<<blocks_tmep[i].word_<<endl;
        cout<<blocks_tmep[i].start_h_<< ", "<< blocks_tmep[i].end_h_<<endl;
        cout<<endl;
    }

    //6.0 对Blocks 进行解码重建
    map<string, Letter> table_inv;
    for(map<Letter, string> ::iterator iter = table.begin(); iter!= table.end(); iter++)
    {
        table_inv.insert(make_pair(iter->second, iter->first));
    }
    blockReconstruction(blocks_tmep, table_inv);

    //7.0 对blocks进行后处理
    posProcessing(blocks_tmep, 20*all_mean_dist);

    // 8.0 processing results
    vector<Vec3>mid_vertices;
    vector<vector<int> > mid_facets;
    gettingTriangulationsFromAllCorners(all_corners, mid_vertices, mid_facets);
    writeOFFFiles(mid_vertices, mid_facets, "model/mid_model.off");

    vector<Vec3>final_vertices;
    vector<vector<int> > final_facets;
    gettingTriangulationsFromBlocks(blocks_tmep, final_vertices, final_facets);
    writeOFFFiles(final_vertices, final_facets, "model/final_model.off");

    // get vertices
    for(int i=0; i<final_vertices.size(); i++ )
    {
        vertices->append(final_vertices[i]);
    }

    // get facets
    for(int i=0; i< final_facets.size(); i++)
    {
        QVector<int> facet;
        for(int j=0; j<final_facets[i].size(); j++)
        {
            facet.append(final_facets[i][j]);
        }
        facets->append(facet);
    }

    // get edges
    QSet<QPair<int, int> > e;
    for(int i=0; i<final_facets.size(); i++)
    {
        int pt_num = final_facets.size();
        for(int j=0;j< pt_num; j++)
        {
           int id0 = j;
           int id1 = (j+1)% pt_num;

           if(id0> id1) e.insert(qMakePair(id1, id0));
           else e.insert(qMakePair(id0, id1));
        }
    }
    QSet<QPair<int,int > > ::const_iterator iter = e.constBegin();
    while(iter!= e.constEnd())
    {
       (*edges)<< *iter;
        iter++;
    }

    // get blocks
    for(int i=0; i< blocks_tmep.size(); i++)
    {
        blocks->append(blocks_tmep[i]);
    }

    emit enableModelingCeiling();

    pts_ids = 0;
    pc = 0;
    slices_pts = 0;
    ycoordinates = 0;
    main_directions = 0;
    facets = 0;
    vertices = 0;
    blocks = 0;
    //progress = 0;
    semi_planes=  0;
    ending_boundary= 0;
    starting_boundary= 0;
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////
SW::FloorPlanDialog::FloorPlanDialog(QWidget *parent,
                                     QVector<Point> *points, QVector<int> *pt_ids):f_points_(points), f_pt_ids_(pt_ids)
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


    ////doubleSpinBox_letterMargin->setValue(0.1);
    ////doubleSpinBox_letterMargin->setSingleStep(0.01);


    /*****************image number need to set********/
    //spinBox_imageID->setRange(0, 200);
    /***********************************************/

    ////label_displayLayerNumber->setNum(0);
    ////label_displayLayerNumber->setBackgroundColor(Qt::yellow);

    ////label_dispMainDirectionNumbers->setNum(0);
    ////label_dispMainDirectionNumbers->setBackgroundColor(Qt::yellow);


    //progressBar_floorRecon->setValue(0);
    //progressBar_floorRecon->setBackgroundColor(Qt::blue);

    //// p_pc_ = pc;
    //// p_pts_ids_ = pts_ids;
    p_inconsistent_detector_ = new InconsistenRegionDetector(this);

    ////p_slices_acculator_ = new SlicesDataCaculator(this);
    ////p_main_directions_extractor_ = new MainDirectionExtractor(this);
    ////p_floorplan_constructor_ = new FloorPlanReconstructor(this);


    is_knn_changed_ = false;
    is_thresh_changed_ = false;
    ////is_gettingSlices_finished_ = false;
    ////is_computeMainDirects_finished_ = false;

    ////doubleSpinBox_windowDepth->setValue(0.05);
    ////doubleSpinBox_windowDepth->setRange(0,1);
    ////doubleSpinBox_windowDepth->setSingleStep(0.01);

    connect(spinBox_knn, SIGNAL(valueChanged(int)), this, SLOT(KnnChanged()) );
    connect(doubleSpinBox_threshold, SIGNAL(valueChanged(double)), this, SLOT(ThreshChanged()));

    // enable group box
    ////connect(p_inconsistent_detector_, SIGNAL(enableGettingSlices()), this, SLOT(enableGroupBoxGettingSlices()));
    ////connect(p_slices_acculator_, SIGNAL(enableFloorPlanReconstruction()), this, SLOT(enableGroupBoxFloorPlanReconstrucion()));
    ////connect(p_main_directions_extractor_, SIGNAL(enableFloorPlanReconstruction()), this, SLOT(enableGroupBoxFloorPlanReconstrucion()));
    ////connect(p_floorplan_constructor_, SIGNAL(enableModelingCeiling()), this, SLOT(enableGroupBoxModelingCeiling()));

    // compute inconsistent region
    // if button id pressed, signal to detect will be emitted
    connect(pushButton_insistRegionDetection, SIGNAL(clicked()), this, SLOT(emitDetectionSignal()));
    // connect(this, SIGNAL(startToDetect(bool,bool)), this, SLOT(inconsistentRegionDetection(bool,bool)), Qt::QueuedConnection);
    connect(this, SIGNAL(startToDetect(bool,bool)), p_inconsistent_detector_, SLOT(detect(bool,bool)), Qt::QueuedConnection);
    p_inconsistent_detector_->moveToThread(&p_thread_);
    p_thread_.start();


    // getting slices
    ////connect(pushButton_divide_slices, SIGNAL(clicked()), p_slices_acculator_, SLOT(divideToSlices()), Qt::QueuedConnection);
    ////p_slices_acculator_->moveToThread(&p_thread_);
    ////p_thread_.start();

    // compute main directions
    ////connect(pushButton_mainDireciongs, SIGNAL(clicked()),p_main_directions_extractor_, SLOT(computeMainDirections()), Qt::QueuedConnection);
    ////p_main_directions_extractor_->moveToThread(&p_thread_);// thread
    ////p_thread_.start();


    // floor plan reconstrution
    ////connect(horizontalSlider_Slayer,SIGNAL(valueChanged(int)), this, SLOT(starttingLayerChanged(int)));
    ////connect(horizontalSlider_Elayer,SIGNAL(valueChanged(int)), this, SLOT(endingLayerChanged(int)));
    ////connect(doubleSpinBox_lineWidth, SIGNAL(valueChanged(double)), this, SLOT(lineWidthChanged(double )));
    ////connect(doubleSpinBox_curveMargin, SIGNAL(valueChanged(double)), this, SLOT(curveMarginChanged(double)));
    ////connect(doubleSpinBox_MRFLambda, SIGNAL(valueChanged(double)), this, SLOT(MRFLambdaChanged(double)));
    ////connect(doubleSpinBox_letterMargin, SIGNAL(valueChanged(double)), this, SLOT(LetterMarginChanged(double)));


    ////connect(pushButton_floorplanRec, SIGNAL(clicked()), p_floorplan_constructor_, SLOT(reconstruction()), Qt::QueuedConnection);
    ////p_floorplan_constructor_->moveToThread(&p_thread_);
    ////p_thread_.start();

    //gts detection
    ////connect(pushButton_beginGTSDetection_, SIGNAL(clicked()), this, SLOT(startGTSDetection()));

}



