#include "sw_functions.h"
#include"sw_cornerExtraction.h"
#include"sw_graph_vec.h"


#include<fstream>
/////////////////////////////////////////////////////////////////////////////////////////////////
vector<pair<int,float> > KNNLinesToPoint(PointXYZRGBNormal &pt, vector<bool>used ,vector<vector<PointXYZRGBNormal> >& lines, int Knn)
{
    vector<pair<int, float> > distances;
    distances.resize(lines.size());
    Vec3 pt0(pt.x, pt.y, pt.z);
    for(int i=0; i< lines.size(); i++)
    {
        if(used[i] ==1)
        {
            distances[i].first = i;
            distances[i].second = 1000000;
            continue;
        }
        Vec3 pt1(lines[i][0].x, lines[i][0].y, lines[i][0].z);
        Vec3 pt2(lines[i][lines[i].size()-1].x, lines[i][lines[i].size()-1].y, lines[i][lines[i].size()-1].z);

        Vec3 diff1 = pt1 - pt0;
        Vec3 diff2 = pt2 - pt0;

        float distance = fmin(diff1.norm(), diff2.norm());

        distances[i].first = i;
        distances[i].second = distance;
    }
    sort(distances.begin(), distances.end(), comparePairFloatLess);

    vector<pair<int, float> > results;
    Knn  = min(Knn, (int)lines.size());
    for(int i=0; i<Knn; i++)
    {
        results.push_back(distances[i]);
    }
    return results;
}
//////////////////////////////////链接直线方法一////////////////////////////////////////////
vector<vector<PointXYZRGBNormal> > linkLinesToCurves(vector<vector<PointXYZRGBNormal> > lines, float TR)
{
    // 将线段按照大小顺序进行排列
    vector<vector<PointXYZRGBNormal> > final_linked_lines;
    vector<vector<PointXYZRGBNormal> > lines_sort;
    vector<pair<int, float> > length_tables;
    for(int i=0; i< lines.size(); i++)
    {
        Vec3 diff(lines[i][0].x - lines[i][1].x, 0, lines[i][0].z - lines[i][1].z);
        length_tables.push_back(make_pair(i, diff.norm()));
    }
    sort(length_tables.begin(), length_tables.end(), comparePairFloatGreat);
    for(int i=0; i< length_tables.size(); i++)
    {
        int id = length_tables[i].first;
        lines_sort.push_back(lines[id]);
    }
    lines.swap(lines_sort);

    vector<bool> used;
    used.resize(lines.size(), 0);

    for(int i=0; i< lines.size(); i++)
    {
        // 获取种子点
        if(used[i] == 1) continue;
        vector<PointXYZRGBNormal> seed_line;
        seed_line.insert(seed_line.end(), lines[i].begin(), lines[i].end());
        used[i] = 1;

        // 对种子点进行生长
        bool terminate = false;

        while(terminate == false)
        {
            // 两头生长，先对头点进行生长
            PointXYZRGBNormal pt_head = *seed_line.begin();
            Vec3 pt_headv(pt_head.x, pt_head.y, pt_head.z);
            vector<pair<int, float> > distances = KNNLinesToPoint(pt_head, used,lines, 2);
            int NN_id = distances[0].first;
            if(used[NN_id] == 0&& distances[0].second< TR)
            {
                Vec3 pt_head_N(lines[NN_id][0].x,lines[NN_id][0].y, lines[NN_id][0].z );
                Vec3 pt_tail_N(lines[NN_id][lines[NN_id].size()-1].x,
                               lines[NN_id][lines[NN_id].size()-1].y,
                               lines[NN_id][lines[NN_id].size()-1].z );
                Vec3 diff_head = pt_headv - pt_head_N;
                Vec3 diff_tail = pt_headv - pt_tail_N;

                float dist_head = diff_head.norm();
                float dist_tail = diff_tail.norm();

                if(dist_head>  dist_tail)
                {
                    seed_line.insert(seed_line.begin(), lines[NN_id].begin(),  lines[NN_id].end());
                    used[NN_id] =1;
                    continue;
                }
                if(dist_tail> dist_head)
                {
                    vector<PointXYZRGBNormal> lines_tmp;
                    for(int k= lines[NN_id].size() -1; k>=0; k--)
                    {
                        lines_tmp.push_back(lines[NN_id][k]);
                    }
                    seed_line.insert(seed_line.begin(), lines_tmp.begin(), lines_tmp.end());
                    used[NN_id] =1;
                    continue;
                }
                else{

                    continue;
                }
            }

            //两头生长，处理尾巴点
            PointXYZRGBNormal pt_tail = *(seed_line.end() -1);
            Vec3 pt_tailv(pt_tail.x, pt_tail.y, pt_tail.z);
            distances = KNNLinesToPoint(pt_tail, used, lines, 2);
            NN_id = distances[0].first;
            if(used[NN_id] == 0&& distances[0].second< TR)
            {
                Vec3 pt_head_N(lines[NN_id][0].x,lines[NN_id][0].y, lines[NN_id][0].z );
                Vec3 pt_tail_N(lines[NN_id][lines[NN_id].size()-1].x,
                               lines[NN_id][lines[NN_id].size()-1].y,
                               lines[NN_id][lines[NN_id].size()-1].z );
                Vec3 diff_head = pt_tailv - pt_head_N;
                Vec3 diff_tail = pt_tailv - pt_tail_N;

                float dist_head = diff_head.norm();
                float dist_tail = diff_tail.norm();

                if(dist_head>  dist_tail)
                {
                    vector<PointXYZRGBNormal> lines_tmp;
                    for(int k= lines[NN_id].size() -1; k>=0; k--)
                    {
                        lines_tmp.push_back(lines[NN_id][k]);
                    }
                    seed_line.insert(seed_line.end(), lines_tmp.begin(), lines_tmp.end());
                    used[NN_id] =1;
                    continue;
                }
                if(dist_tail> dist_head)
                {
                    seed_line.insert(seed_line.end(), lines[NN_id].begin(), lines[NN_id].end());
                    used[NN_id] =1;
                    continue;
                }
                else{
                    continue;
                }
            }

            if(seed_line.size()> 2)
                final_linked_lines.push_back(seed_line);

            terminate = true;

        }


    }

    return final_linked_lines;
}
#if 0
/////////////////////////////////////////////////////////////////////////////////////////////////////
vector<vector<PointXYZRGBNormal> > linkLinesToCurves(vector<vector<PointXYZRGBNormal> > lines, float TR)
{
    CENTERS_EMERGING::Graph_link<float> graph;
    graph.makeGraph(lines.size()*2);

    int iter = 0;
    for(int i=0; i< lines.size(); i++)
    {
        Vec3 diff(lines[i][0].x - lines[i][1].x, 0, lines[i][0].z - lines[i][1].z);

        int id0 = iter;
        CENTERS_EMERGING::Node node0(lines[i][0], id0);

        int id1 = iter+1;
        CENTERS_EMERGING::Node node1(lines[i][1], id1);

        graph.addNode(node0);
        graph.addNode(node1);

        graph.insertEdge(id0, id1, diff.norm());

        iter+=2;
    }

    bool flag = true;
    while(flag == true)
    {
        flag = false;
        vector<pair<int, float> > distances;
        vector<pair<int ,int> >indices;

        for(int i=0; i<graph.nodes_.size(); i++)
        {
            if(graph.getNodeDegree(i) ==2) continue;
            flag = true;

            int id0 = graph.nodes_[i].id_;

            for(int j=0; j< graph.nodes_.size(); j++)
            {
                int id1 = graph.nodes_[j].id_;
                if(id0 == id1) continue;
                if(graph.getNodeDegree(id1) ==2 )continue;
                if(graph.getEdge(id0, id1)!= -1)continue;

                indices.push_back(make_pair(id0, id1));

                Vec3 diff(graph.node(id0).pt_.x - graph.node(id1).pt_.x,
                          0,
                          graph.node(id0).pt_.z - graph.node(id1).pt_.z);
                distances.push_back(make_pair(distances.size(), diff.norm()));
            }
        }
        if(distances.size() ==0) break;
        sort(distances.begin(), distances.end(), comparePairFloatLess);

        int pair_id = distances[0].first;
        float d = distances[0].second;


        graph.insertEdge(indices[pair_id].first, indices[pair_id].second, d);
    }

    ////////////////////////////////////////////////////////////////////
    // 将图显示出来看看

    // 曲线的结果
    float min_x = 6.53111;
    float min_z = -40.4436;
    float TR_ = 5*0.0202005;

    cv::Mat_<cv::Vec3b> imgCurveResults(767, 637);
    imgCurveResults.setTo(0);
    for(int i=0; i<graph.nodes_.size(); i++)
    {
        int x = (graph.node(i).pt_.x - min_x)/TR_;
        int z = (graph.node(i).pt_.z - min_z)/TR_;

        cv::circle(imgCurveResults, cv::Point2i(x, z), 6, cv::Scalar(255,0,255), 2);

        for(int j=i+1; j<graph.nodes_.size(); j++ )
        {
            if(graph.getEdge(i, j)!=-1)
            {
                int xx = (graph.node(j).pt_.x - min_x )/TR_;
                int zz = (graph.node(j).pt_.z - min_z )/TR_;
                line(imgCurveResults, cv::Point2i(x,z), cv::Point2i(xx, zz),cv::Scalar(0, 0, 255), 2 );
            }
        }
    }

    cv::imshow("iamge", imgCurveResults);
    cv::waitKey();

}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
// 将曲线调整到主方向上
void curvesAdjustment(vector<vector<PointXYZRGBNormal> >& input_curves, vector<Vec3> & main_directions, float TR)
{
    for(int i=0; i< input_curves.size(); i++)
    {
        for(int j=0; j< input_curves[i].size(); j++)
        {
            int id0 = j;
            int id1 = (j+1)% input_curves[i].size();

            Vec3 dir0(input_curves[i][id1].x -  input_curves[i][id0].x,
                      0,
                      input_curves[i][id1].z -  input_curves[i][id0].z);

            if(dir0.norm() ==0 )continue;
            dir0.normalize();


            vector<pair<int, float> > distances;
            for(int k=0; k< main_directions.size(); k++)
            {
                Vec3 dir1(main_directions[k].x_, 0, main_directions[k].z_);

                distances.push_back(make_pair(k, abs(dir0* dir1)));
            }
            sort(distances.begin(), distances.end(), comparePairFloatLess);

            int dir_id = distances[0].first;

            Vec3 pt(input_curves[i][id0].x ,
                    0,
                    input_curves[i][id0].z);

            Vec3 normal(main_directions[dir_id].x_, 0, main_directions[dir_id].z_);

            Vec3 pt0(input_curves[i][id0].x, 0,  input_curves[i][id0].z);
            Vec3 pt1(input_curves[i][id1].x, 0,  input_curves[i][id1].z);

            Vec3 new_pt0 = pt0 -  ((pt0-pt)* normal)* normal;
            Vec3 new_pt1 = pt1 -  ((pt1-pt)* normal)* normal;

            Vec3 diff0 = pt0 - new_pt0;
            Vec3 diff1 = pt1 - new_pt1;

            if(diff0.norm()>TR|| diff1.norm()> TR)continue;

            input_curves[i][id0].x = new_pt0.x_;
            input_curves[i][id0].z = new_pt0.z_;

            input_curves[i][id1].x = new_pt1.x_;
            input_curves[i][id1].z = new_pt1.z_;
        }
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////
// 删除曲线中相同的点
vector<vector<PointXYZRGBNormal> > deleteSamePointsInCurves(vector<vector<PointXYZRGBNormal> >& input_curves, float TR)
{
    vector< vector<PointXYZRGBNormal> > output_curves;

    //  去除曲线中相同的点
    for(int i=0; i< input_curves.size(); i++)
    {
        vector<PointXYZRGBNormal> pts;
        for(int j=0; j< input_curves[i].size(); j++)
        {
            int id0 = j;
            int id1 = (j+1) % input_curves[i].size();

            Vec3 pt0(input_curves[i][id0].x, 0, input_curves[i][id0].z);
            Vec3 pt1(input_curves[i][id1].x, 0, input_curves[i][id1].z);

            Vec3 diff = pt0 - pt1;

            float r = diff.norm();

            // cout<<j<<" the diff: "<< diff.norm()<<endl;
            if(diff.norm() < TR)
            {
            }
            else
            {
                pts.push_back(input_curves[i][id0]);
            }
        }

        output_curves.push_back(pts);
    }

    return output_curves;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//删除曲线中的冗余点
vector<vector< PointXYZRGBNormal> > deleteRedundancyPointInCurves(vector<vector<PointXYZRGBNormal> >& input_curves)
{
    // 去除直线中多余的点
    vector<vector<PointXYZRGBNormal> > output_curves;
    for(int i= 0; i< input_curves.size(); i++)
    {
        vector<PointXYZRGBNormal> pts;
        for(int j= 0; j< input_curves[i].size(); j++)
        {
            int id0 = j;
            int id1 = (j+1) % input_curves[i].size();
            int id2 = (j+2) % input_curves[i].size();

            Vec3 v0(input_curves[i][id1].x - input_curves[i][id0].x,
                    0,
                    input_curves[i][id1].z - input_curves[i][id0].z);
            Vec3 v1(input_curves[i][id2].x - input_curves[i][id1].x,
                    0,
                    input_curves[i][id2].z - input_curves[i][id1].z);

            v0.normalize();
            v1.normalize();

            float r = v0* v1;
            r = min(r, (float)0.9999);
            r = max(r, (float) -0.9999);

            float angle = acos(r) * 180/ 3.1415;

            if(angle < 25|| angle > 130)
            {
            }
            else
            {
                pts.push_back(input_curves[i][id1]);
            }
        }
        output_curves.push_back(pts);
    }
    return output_curves;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector<vector<PointXYZRGBNormal> >curvesProcessing(vector<vector<PointXYZRGBNormal> > &input_curves, float TR)
{
    vector<vector<PointXYZRGBNormal> > output_curves;
    output_curves = deleteSamePointsInCurves(input_curves, TR);

    int old_num = 1;
    int new_num = 0;
    while(old_num!= new_num)
    {
        old_num = 0;
        for(int i=0; i< output_curves.size(); i++)
        {
            for(int j=0; j< output_curves[i].size(); j++)
            {
                old_num++;
            }
        }
        output_curves = deleteRedundancyPointInCurves(output_curves);
        output_curves = deleteSamePointsInCurves(output_curves, TR);

        new_num = 0;
        for(int i=0; i< output_curves.size(); i++)
        {
            for(int j=0; j< output_curves[i].size(); j++)
            {
                new_num++;
            }
        }
    }

    output_curves = deleteRedundancyPointInCurves(output_curves);

    //    for(int i=0; i< output_curves.size(); i++)
    //    {
    //        for(int j=0; j< output_curves[i].size(); j++)
    //        {
    //            cout<<j <<": ( "<< output_curves[i][j].x<<", " <<output_curves[i][j].z <<" )"<<endl;
    //        }
    //    }

    return output_curves;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
vector<vector<PointXYZRGBNormal > > cornerExtraction(vector<PointXYZRGBNormal > & input,
                                                     float TR, float RG_TR, float Link_TR, vector<Vec3> &main_directions)
{
    // ********************1  对法向量进行双边滤波 *********************/
    cout<<"Normals Bilateral Filtering...."<<endl;
    vector<PointXYZRGBNormal > points_after_normal_filter;
    points_after_normal_filter.insert(points_after_normal_filter.end(), input.begin(), input.end() );
    bilateralFilterNormal(points_after_normal_filter, 6*TR, 15);
    cout<<"Done!"<<endl;
    cout<<endl;




    //*********************2 graph 进行优化  ***************************************************************//
    cout<<"Graph Clustering...."<<endl;
    vector<Vec3> all_normals;
    all_normals.resize(points_after_normal_filter.size());
    for(int i=0; i<points_after_normal_filter.size(); i++)
    {
        all_normals[i].x_ = points_after_normal_filter[i].normal_x;
        all_normals[i].y_ = points_after_normal_filter[i].normal_y;
        all_normals[i].z_ = points_after_normal_filter[i].normal_z;
    }
    alpha_expansion_vec AEC(all_normals, main_directions);
    AEC.setAngleThresh(35);
    AEC.setMaxIterNum(5);
    AEC.computeKnnNeighbours(30);
    AEC.setLambda(20);
    AEC.optimization();
    vector<Vec3> all_normals_results;
    all_normals_results.resize(all_normals.size());
    for(int i=0; i< AEC.labels_.size(); i++)
    {
        int label  =  AEC.labels_[i];
        all_normals_results[i] = main_directions[label];
    }
    cout<<"Done!"<<endl;
    cout<<endl;


#if 0
    string name = "../../SlicesData/test.txt";
    ofstream writef;
    writef.open(name.c_str(), ios::out);

    writef<< points_after_normal_filter.size()<<endl;
    for(int k= 0; k< points_after_normal_filter.size(); k++)
    {
        writef<< points_after_normal_filter[k].x<<" ";
        writef<< points_after_normal_filter[k].y<<" ";
        writef<< points_after_normal_filter[k].z<<" ";

        writef<< points_after_normal_filter[k].r<<" ";
        writef<< points_after_normal_filter[k].g<<" ";
        writef<< points_after_normal_filter[k].b<<" ";

        writef<< all_normals_results[k].x_<<" ";
        writef<< all_normals_results[k].y_<<" ";
        writef<< all_normals_results[k].z_<<" ";

        writef<< points_after_normal_filter[k].inconsist_<<endl;
    }
    writef.close();
#endif

    //**********************3  先进行区域生长聚类，再进行位置优化，这样可以避免相互干扰的问题***************//
    cout<<"Region Growing...."<<endl;
    vector<PointXYZRGBNormal> RG_input;
    RG_input.insert(RG_input.end(), points_after_normal_filter.begin(), points_after_normal_filter.end());
    for(int i=0; i< RG_input.size(); i++)
    {
        RG_input[i].normal_x = all_normals_results[i].x_;
        RG_input[i].normal_y = all_normals_results[i].y_;
        RG_input[i].normal_z = all_normals_results[i].z_;
    }
    SW::RegionGrowing RG;
    RG.setInput(RG_input);
    RG.set_angle_thresh(15);
    RG.neighbours(50);
    RG.set_max_dist(2*RG_TR);
    RG.set_max_width(RG_TR);
    RG.regionGrowing();
    cout<<"Region Growing Clusters num: "<<RG.clusters_.size()<<endl;
    //for(int i=0; i<RG.clusters_.size(); i++)
    //    cout<<i<<" th cluster "<< RG.clusters_[i].size()<<" pts"<<endl;
    cout<<"Done!"<<endl;
    cout<<endl;





    //*******************4  对直线的位置进行优化，同时对优化后的直线进行拟合*****************************//
    vector<PointXYZRGBNormal> points_after_pos_filter;


    int num_thresh = 30;
    cout<<"Line Fitting ...."<<endl;
    vector<vector<float> > line_params; //每个聚类的直线参数
    vector<vector<PointXYZRGBNormal> > line_points; // 对应的每条直线所包含的点
    for(int i=0; i< RG.clusters_.size(); i++)
    {
        if(RG.clusters_[i].size()< num_thresh )continue;

        vector<PointXYZRGBNormal> sub_segments;
        for(int j=0; j< RG.clusters_[i].size(); j++)
        {
            int id = RG.clusters_[i][j];
            sub_segments.push_back(RG.pts_[id]);
        }

        //    // 对位置进行优化
        bialteralFilterPosition(sub_segments, 30*TR,15);
        points_after_pos_filter.insert(points_after_pos_filter.end(), sub_segments.begin(), sub_segments.end());

        //    //对每个区域进行直线拟合,并保存相应的点
        vector<float> params = leastSquareFittingLine(sub_segments);
        line_params.push_back(params);
        line_points.push_back(sub_segments);
    }
    cout<<"Done!"<<endl;
    cout<<endl;

#if 1

    //*******************5  对相交直线的交点进行处理 ************************************************//
    cout<<"Knn Lines Neighbouring ...."<<endl;
    int Knn = 2;
    vector<vector<float> >cluster_distances;
    vector<vector<int> > cluster_neighbrs;
    cluster_neighbrs =  knnClusterNeighbours( Knn, line_points, cluster_distances );
    vector< vector<PointXYZRGBNormal> > line_vertex;
    line_vertex.resize(line_points.size());
    // 5.0 获取直线的顶点
    for(int i=0; i< line_points.size(); i++)
    {
        vector<float> coord_x;
        for(int j=0; j< line_points[i].size(); j++)
        {
            coord_x.push_back(line_points[i][j].x);
        }

        vector<float> ::iterator iter = max_element(coord_x.begin(),coord_x.end());
        float x0 = *iter;
        iter = min_element(coord_x.begin(), coord_x.end());
        float x1 = *iter;

        float z0 = (-line_params[i][0]* x0 - line_params[i][2])/line_params[i][1];
        float z1 = (-line_params[i][0]* x1 - line_params[i][2])/line_params[i][1];

        PointXYZRGBNormal pt0(x0, 0, z0);
        PointXYZRGBNormal pt1(x1, 0, z1);

        line_vertex[i].push_back(pt0);
        line_vertex[i].push_back(pt1);
    }
    //5.1 对相交的直线的顶点进行处理
    for(int i=0; i< cluster_neighbrs.size(); i++)
    {
        vector<float> line_params_s = line_params[i];
        for(int j=0; j<cluster_neighbrs[i].size(); j++ )
        {
            int id_n = cluster_neighbrs[i][j];
            if(i == id_n) continue;

            vector<float> line_params_n = line_params[id_n];

            switch(lineRelationShape(line_params_s, line_params_n, 4*TR) )
            {
            case 0: // PARALLEL
            {
#if 0
                vector<pair<int, int> > vertex_pairs;
                for(int m=0; m< 2; m++)
                {
                    for(int n=0; n< 2; n++)
                    {
                        vertex_pairs.push_back(make_pair(m, n));
                    }
                }

                vector<pair<int,float> > distances;
                int iter = 0;
                for(int m=0; m< 2; m++)
                {
                    Vec3 vertex0(line_vertex[i][m].x,0, line_vertex[i][m].z);
                    for(int n=0; n< 2; n++)
                    {
                        Vec3 vertex1(line_vertex[id_n][m].x,0, line_vertex[id_n][m].z);
                        Vec3 diff = vertex1 - vertex0;
                        distances.push_back(make_pair(iter, diff.norm()));
                        iter++;
                    }
                }
                sort(distances.begin(), distances.end(), comparePairLess);
                pair<int, int> index= vertex_pairs[distances[0].first];

                parallel_corners.push_back(line_vertex[i][index.first]);
                parallel_corners.push_back(line_vertex[id_n][index.second]);

#endif
                break;
            }
            case 1: // INTERSECT
            {
                PointXYZRGBNormal pt = intersection(line_params_s, line_params_n);
                //intersect_corners.push_back(pt);

                Vec3 pt_base(pt.x, 0, pt.z);
                Vec3 pt00(line_vertex[i][0].x, 0, line_vertex[i][1].z);
                Vec3 pt01(line_vertex[i][1].x, 0, line_vertex[i][1].z);

                Vec3 diff00 =  pt00- pt_base;
                Vec3 diff01 =  pt01- pt_base;

                if(diff00.norm()< diff01.norm()) line_vertex[i][0] = pt;
                else{
                    line_vertex[i][1]= pt;
                }

                Vec3 pt20(line_vertex[id_n][0].x, 0, line_vertex[id_n][1].z);
                Vec3 pt21(line_vertex[id_n][1].x, 0, line_vertex[id_n][1].z);

                Vec3 diff20 =  pt20- pt_base;
                Vec3 diff21 =  pt21- pt_base;

                if(diff20.norm()< diff21.norm()) line_vertex[id_n][0] = pt;
                else if(diff20.norm()>= diff21.norm()){
                    line_vertex[id_n][1]= pt;
                }
                break;
            }
            case 2: // COLINEAR
            {
                break;
            }
            default:break;
            }
        }
    }
    cout<<"Done!"<<endl;
    cout<<endl;





    //*******************6  链接直线 ************************************************//
    cout<<"Linking Lines..."<<endl;
    vector<vector<PointXYZRGBNormal> >curves =  linkLinesToCurves(line_vertex,Link_TR);
    cout<<"Done!"<<endl;
    cout<<endl;




    //*******************7 对链接的直线进行调整************************************************//
    cout<<"Curves Processing..."<<endl;
    //7.0 根据主方向，对直线进行调整
    curvesAdjustment(curves, main_directions, 10*TR);
    // 7.1 删除相同的点和临近的点
    vector< vector<PointXYZRGBNormal> > curves_after_process = curvesProcessing(curves, 10*TR);
    curves.swap( curves_after_process);
    //7.2 根据主方向，对直线进行调整
    curvesAdjustment(curves, main_directions, 10*TR);
    cout<<"Done!"<<endl;
    cout<<endl;

    // 经过处理后，有些点的个数可能为0,删除这些曲线
    vector<vector<PointXYZRGBNormal> >curves_tmp;
    for(int i=0; i< curves.size(); i++)
    {
        if(curves[i].size()> 0)
        {
            curves_tmp.push_back(curves[i]);
        }
    }
    curves.swap(curves_tmp);

#endif
#if 0

    /*******************************用于调试*********************************************************/
    //  创建颜色列表
    vector<vector<int > >colors;
    for(int i=0; i< main_directions.size(); i++)
    {
        vector<int> c;
        c.push_back(rand()&255);
        c.push_back(rand()&255);
        c.push_back(rand()&255);
        colors.push_back(c);
    }

    //  显示输入数据
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat_<cv::Vec3b> imgTemplate(z_times, x_times);
    imgTemplate.setTo(0);
    for(int i=0; i<input.size(); i++)
    {
        int x = (input[i].x - min_x)/reso;
        int z = (input[i].z - min_z)/reso;

        int xx = (input[i].x - min_x + input[i].normal_x)/reso;
        int zz = (input[i].z - min_z + input[i].normal_z)/reso;

        line(imgTemplate, cv::Point2i(x,z), cv::Point2i(xx, zz), cv::Scalar(255, 255, 0) );

        cv::circle(imgTemplate, cv::Point2i(x, z), 1, cv::Scalar(255,0,255), 1);


        int margin = (int)(RG_TR/reso);

        line(imgTemplate, cv::Point2i(10,20), cv::Point2i(100, 20), cv::Scalar(255, 0, 0), 2 );
        line(imgTemplate, cv::Point2i(10,20+ margin), cv::Point2i(100, 20+ margin), cv::Scalar(255, 0, 0), 2 );
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 显示MRF 聚类后的结果
    cv::Mat_<cv::Vec3b> imgMRFClustering(z_times, x_times);
    imgMRFClustering.setTo(0);
    for(int i=0; i<points_after_normal_filter.size(); i++)
    {
        int x = (points_after_normal_filter[i].x - min_x)/reso;
        int z = (points_after_normal_filter[i].z - min_z)/reso;

        int xx = (points_after_normal_filter[i].x - min_x + all_normals_results[i].x_)/reso;
        int zz = (points_after_normal_filter[i].z - min_z + all_normals_results[i].z_)/reso;

        int label = AEC.labels_[i];
        line(imgMRFClustering, cv::Point2i(x,z), cv::Point2i(xx, zz),cv::Scalar(colors[label][0],colors[label][1],colors[label][2]) );
        cv::circle(imgMRFClustering, cv::Point2i(x, z), 1, cv::Scalar(255,0,255), 1);
    }
    for(int i=0; i<main_directions.size(); i++)
    {
        int x = imgMRFClustering.cols/2;
        int z = imgMRFClustering.rows/2;

        int xx = x +  5*main_directions[i].x_/reso;
        int zz = z +  5*main_directions[i].z_/reso;

        cv::circle(imgMRFClustering, cv::Point2i(x, z), 4,cv::Scalar(255, 0, 255 ), 4);
        line(imgMRFClustering, cv::Point2i(x,z), cv::Point2i(xx, zz), cv::Scalar(colors[i][0],colors[i][1],colors[i][2]), 4);
    }
    for(int i=0; i<main_directions.size(); i++)
    {
        int x = imgMRFClustering.cols/2;
        int z = imgMRFClustering.rows/2;

        int xx = x +  main_directions[i].x_/reso;
        int zz = z +  main_directions[i].z_/reso;

        cv::circle(imgMRFClustering, cv::Point2i(x, z), 4,cv::Scalar(255, 0, 255 ), 4);
        line(imgMRFClustering, cv::Point2i(x,z), cv::Point2i(xx, zz), cv::Scalar(colors[i][0],colors[i][1],colors[i][2]), 4);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 显示区域生长后的结果
    cv::Mat_<cv::Vec3b> imgRGClustering(z_times, x_times);
    imgRGClustering.setTo(0);
    for(int i=0; i<RG.clusters_.size(); i++)
    {
        float r = rand()&255;
        float g = rand()&255;
        float b = rand()&255;

        for(int j=0; j<RG.clusters_[i].size(); j++)
        {
            if(RG.clusters_[i].size() < num_thresh)continue;

            int pt_id = RG.clusters_[i][j];
            int x = (RG.pts_[pt_id].x - min_x)/reso;
            int z = (RG.pts_[pt_id].z - min_z)/reso;

            int xx = (RG.pts_[pt_id].x - min_x + RG.pts_[pt_id].normal_x)/reso;
            int zz = (RG.pts_[pt_id].z - min_z + RG.pts_[pt_id].normal_z)/reso;

            line(imgRGClustering, cv::Point2i(x,z), cv::Point2i(xx, zz), cv::Scalar(r, g, b) );
            cv::circle(imgRGClustering, cv::Point2i(x, z), 1, cv::Scalar(255, 0, 0), 1);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // 对位置进行调整后的结果
    cv::Mat_<cv::Vec3b> imgPosBilateralFilter(z_times, x_times);
    imgPosBilateralFilter.setTo(0);
    for(int i=0; i<points_after_pos_filter.size(); i++)
    {
        int x = (points_after_pos_filter[i].x - min_x)/reso;
        int z = (points_after_pos_filter[i].z - min_z)/reso;

        int xx = (points_after_pos_filter[i].x - min_x + all_normals_results[i].x_)/reso;
        int zz = (points_after_pos_filter[i].z - min_z + all_normals_results[i].z_)/reso;

        //int label = AE.labels_[i];
        //line(imgPosBilateralFilter, cv::Point2i(x,z), cv::Point2i(xx, zz),cv::Scalar(colors[label][0],colors[label][1],colors[label][2]) );
        cv::circle(imgPosBilateralFilter, cv::Point2i(x, z), 1, cv::Scalar(255,0,255), 1);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 1
    // 直线的端点
    for(int i=0; i< line_vertex.size(); i++)
    {
        for(int j=0; j< line_vertex[i].size(); j++)
        {
            int x = (line_vertex[i][j].x - min_x)/reso;
            int z = (line_vertex[i][j].z - min_z)/reso;

            cv::circle(imgPosBilateralFilter, cv::Point2i(x, z), 6, cv::Scalar(255,0,0), 4);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 曲线的结果
    cv::Mat_<cv::Vec3b> imgCurveResults(z_times, x_times);
    imgCurveResults.setTo(0);


    for(int i=0; i<input.size(); i++)
    {
        int x = (input[i].x - min_x)/reso;
        int z = (input[i].z - min_z)/reso;

        int xx = (input[i].x - min_x + input[i].normal_x)/reso;
        int zz = (input[i].z - min_z + input[i].normal_z)/reso;

        cv::circle(imgCurveResults, cv::Point2i(x, z), 1, cv::Scalar(255,0,255), 1);
        //cv::circle(imgCurveResults, cv::Point2i(x, z), 1, cv::Scalar(0,0,255), 1);
    }

    for(int i=0; i<curves.size(); i++)
    {
        int r = rand()&255;
        int g = rand()&255;
        int b = rand()&255;

        for(int j=0; j< curves[i].size(); j++)
        {
            int id0 = j;
            int id1 = (j+1)%curves[i].size();

            int x = (curves[i][id0].x - min_x)/reso;
            int z = (curves[i][id0].z - min_z)/reso;

            int xx = (curves[i][id1].x - min_x )/reso;
            int zz = (curves[i][id1].z - min_z )/reso;


            line(imgCurveResults, cv::Point2i(x,z), cv::Point2i(xx, zz),cv::Scalar(r, g, b), 2 );
            cv::circle(imgCurveResults, cv::Point2i(x, z), 2, cv::Scalar(255,0,255), 1);
        }

        int x = (curves[i][0].x - min_x)/reso;
        int z = (curves[i][0].z - min_z)/reso;
        int xx = (curves[i][1].x - min_x )/reso;
        int zz = (curves[i][1].z - min_z )/reso;

        circle(imgCurveResults, cv::Point2i(x,z), 6 ,cv::Scalar(255, 0, 0), 2 );
        circle(imgCurveResults, cv::Point2i(xx,zz), 6 ,cv::Scalar(0, 255, 0), 2 );
    }
#endif
    cv::imshow("Original Data", imgTemplate);
    cv::imshow("MRF Clustering", imgMRFClustering);
    cv::imshow("Region Growing Results", imgRGClustering);
    cv::imshow("Position Bilateral Filter Results", imgPosBilateralFilter);
    cv::imshow("Curve Results", imgCurveResults);
#endif


    return curves;

}


