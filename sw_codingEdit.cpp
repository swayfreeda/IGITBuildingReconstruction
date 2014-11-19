#include"sw_codingEdit.h"
#include"sw_functions.h"
#include"sw_graph_str.h"

using namespace SW;


float Letter::TR_ = 0.01; //先进行定义，static 类成员的定义只能在 main()函数外，否则会出错

//----------------------------------------isWordValid-------------------------------------------------------------------//
// 判断编码是否有效
bool isWordValid(string & word)
{
    bool valid = true;
    for(int i=0; i< word.size(); i++)
    {
        if(word[i] == '&')
        {
            valid = false;
            break;
        }
    }
    return valid;
}


//----------------------------------------subCurvesPositionAdjustment---------------------------------------------------//
// 如果每一层含有多个轮廓， 则对轮廓进行调整，保证是按照同一顺序排列
void subCurvesPositionAdjustment(vector<vector<PointXYZRGBNormal> > &curves,
                                 Vec3&ref_pt)
{
    vector<pair<int, float> > distances;
    for(int i= 0; i<curves.size(); i++)
    {
        float mean_x = 0;
        float mean_y = 0;
        float mean_z = 0;

        // 对每一个轮廓的点，求中心点的位置
        int cNum  = curves[i].size();
        for(int j=0; j< cNum; j++)
        {
            mean_x += curves[i][j].x/cNum;
            mean_y += curves[i][j].y/cNum;
            mean_z += curves[i][j].z/cNum;
        }

        // 计算到参考点的距离
        Vec3 mean_p(mean_x, 0, mean_z);
        Vec3 diff = mean_p - ref_pt;

        float dist = diff.norm();
        distances.push_back(make_pair(i, dist));
    }
    sort(distances.begin(), distances.end(), comparePairFloatLess);

    vector<vector<PointXYZRGBNormal> > 	curves_tmp;
    for(int i=0;i< distances.size();i++)
    {
        int index = distances[i].first;
        curves_tmp.push_back(curves[index]);
    }
    curves.swap(curves_tmp);
}


//----------------------------------------startPosAdjustment------------------------------------------------------------//
//对提取的轮廓进行对齐，保证起始位置一致
void startPosAdjustment(vector<vector<PointXYZRGBNormal> > & curves, Vec3 &ref_pt)
{
    // 根据参考点的位置获取起始点的索引
    vector<int > start_id;
    for(int i=0; i< curves.size(); i++)
    {
        vector<pair<int, float> > distances;
        for(int j=0; j< curves[i].size(); j++)
        {
            Vec3 pt(curves[i][j].x, 0, curves[i][j].z);
            Vec3 diff= pt - ref_pt;
            distances.push_back(make_pair(j, diff.norm()));
        }
        sort(distances.begin(), distances.end(), comparePairFloatLess);
        start_id.push_back(distances[0].first);
    }
    // 对点的顺序进行重新调整
    for(int i=0; i< curves.size(); i++)
    {
        vector<PointXYZRGBNormal> pts = curves[i];
        int index = start_id[i];

        vector<PointXYZRGBNormal> tmp;
        for(int j = index;  j< curves[i].size(); j++)
        {
            tmp.push_back(curves[i][j]);
        }
        for(int j=0; j< index; j++)
        {
            tmp.push_back(curves[i][j]);
        }
        curves[i].swap(tmp);
    }
}




//----------------------------------------startDirectionAdjustment-------------------------------------------------------//
//对提取的轮廓进行调整，保证曲线是按照逆时针方向旋转
void startDirectionAdjustment(vector<vector<PointXYZRGBNormal> > &curves)
{
    Vec3 gravity(0, 1, 0);

    for(int i=0; i< curves.size(); i++)
    {
        Vec3 pt0(curves[i][curves[i].size()-1].x,
                 0,
                 curves[i][curves[i].size()-1].z);


        Vec3 pt1(curves[i][0].x,
                 0,
                 curves[i][0].z);

        Vec3 pt2(curves[i][1].x,
                 0,
                 curves[i][1].z);

        Vec3 diff10 = pt1 - pt0;
        Vec3 diff21 = pt2 - pt1;

        if(cross(diff10, diff21)* gravity < 0)
        {
            reverse(curves[i].begin()+ 1, curves[i].end());
        }
    }
}



//----------------------------------------operator <<--------------------------------------------------------------------//
/**************************重载输出流**************************************************/
ostream &operator << (ostream& os, const Letter& let)
{
    os<<"[ ClockWise: "<< let.clockWise_<<endl;
    os<< "Angle: "<< let.angle_<<endl;
    os<< "Length: "<< let.length_<<" ]"<<endl;
}




//-----------------------------------------make_dictionary---------------------------------------------------------------//
//创建字典
// TR用来控制直线段相同的误差范围
map<Letter, string> make_dictionary(vector<vector<PointXYZRGBNormal> > &allCorners)
{

    map<Letter, int> hist;
    Vec3 gravity(0, 1, 0);

    for(int i_c=0; i_c< allCorners.size(); i_c++)
    {
        //cout<<"layer: "<< i_c<<": "<<endl;
       // cout<<"******************************************************************************"<<endl;
        vector<PointXYZRGBNormal> corners = allCorners[i_c];
        int corner_num = corners.size();

        // 第一个字符
        {
            Vec3 dir0(0, 0, 1);
            Vec3 dir1( corners[1].x - corners[0].x,
                       corners[1].y - corners[0].y,
                       corners[1].z - corners[0].z);
            float length = dir1.norm();

            dir1.normalize();
            float sign = gravity * (cross(dir0, dir1));
            float angle = acos(dir0* dir1)*180/3.1415;

            int clockWise;
            if(sign > 0) clockWise = 1;
            else clockWise = 0;

            Letter first(clockWise, angle, length);

           // cout<<" Letter: "<< first;

            //cout<<"first: "<< first<<endl;
            if(hist.count(first) == 0)
            {
                // string str(1, alphabet[table.size()]);
                // table[first] = str;
                hist[first] = 1;
            }
            else{

                // 更新字典，求角度和长度的平均值
//                map<Letter, int> ::iterator iter = hist.find(first);
//                int clockwise_ = clockWise;
//                int num = iter->second +1;
//                float angle_ = 0.5* iter->first.angle_ + 0.5* first.angle_;
//                float length = 0.5 * iter->first.length_ + 0.5 * first.length_;

//                Letter letter_(clockwise_, angle_, length);
//                hist.erase(iter);
//                hist.insert(map<Letter, int>::value_type(letter_, num));

                hist[first] ++;
            }
        }
        // 中间字符
        {
            for(int i=1; i< corners.size() -1; i++)
            {
                Vec3 dir0( corners[i].x - corners[i-1].x,
                           corners[i].y - corners[i-1].y,
                           corners[i].z - corners[i-1].z);
                dir0.normalize();

                Vec3 dir1(corners[i+1].x - corners[i].x,
                          corners[i+1].y - corners[i].y,
                          corners[i+1].z - corners[i].z);
                float length = dir1.norm();

                dir1.normalize();
                float sign = gravity*(cross(dir0, dir1));
                float angle = acos(dir0* dir1)*180/3.1415;
                angle = (float) (int)(angle + 0.5);
                if(abs(angle - 90)< 5) angle = 90.0;

                float clockWise;
                if(sign> 0) clockWise = 1;
                else clockWise = 0;

                Letter mid(clockWise, angle, length);

               //  cout<<" Letter: "<< mid;
                // cout<<"mid: "<< mid<<endl;

                if(hist.count(mid) == 0)
                {
                    //string str(1, alphabet[table.size()]);
                    //table[mid] = str;
                    hist[mid] = 1;
                }
                else{
//                    map<Letter, int> ::iterator iter = hist.find(mid);
//                    int clockwise_ = iter->first.clockWise_;
//                    int num = iter->second +1;
//                    float angle_ = 0.5* iter->first.angle_ + 0.5* mid.angle_;
//                    float length = 0.5 * iter->first.length_ + 0.5 * mid.length_;

//                    Letter letter_(clockwise_, angle_, length);
//                    hist.erase(iter);
//                    hist.insert(map<Letter, int>::value_type(letter_, num));

                    hist[mid]++;
                }
            }
        }
        // 最后一个字符
        {
            Vec3  dir0( corners[corner_num -1].x - corners[corner_num -2].x,
                        corners[corner_num -1].y - corners[corner_num -2].y,
                        corners[corner_num -1].z - corners[corner_num -2].z);
            dir0.normalize();

            Vec3 dir1(corners[0].x - corners[corner_num-1].x,
                      corners[0].y - corners[corner_num-1].y,
                      corners[0].z - corners[corner_num-1].z);
            float length = dir1.norm();

            dir1.normalize();
            float sign = gravity*(cross(dir0, dir1));

            float angle = acos(dir0* dir1)*180/3.1415;
            angle = (float) (int)(angle + 0.5);
            if(abs(angle - 90)< 5) angle = 90.0;

            float clockWise;
            if(sign> 0) clockWise = 1;
            else clockWise = 0;

            Letter last(clockWise, angle, length);

           //  cout<<" Letter: "<< last;


            // cout<<"last: "<< last <<endl;
            if(hist.count(last) == 0)
            {
                //string str(1, alphabet[table.size()]);
                //table[last] = str;
                hist[last] = 1;
            }
            else{

//                map<Letter, int> ::iterator iter = hist.find(last);
//                int clockwise_ = iter->first.clockWise_;
//                int num = iter->second +1;
//                float angle_ = 0.5* iter->first.angle_ + 0.5* last.angle_;
//                float length = 0.5 * iter->first.length_ + 0.5 * last.length_;


//                Letter letter_(clockwise_, angle_, length);
//                hist.erase(iter);
//                hist.insert(map<Letter, int>::value_type(letter_, num));
                hist[last]++;

            }
        }
    }

//    cout<<"****************************TABLE*****************************************"<<endl;
//    for(map<Letter, int> ::iterator iter = hist.begin(); iter!= hist.end(); iter++)
//    {
//        cout<<"Letter: "<< iter->first<<" Num: "<< iter->second<<endl;
//    }
    
    map<Letter, string> table;
    for(map<Letter, int> :: iterator iter = hist.begin(); iter!= hist.end(); iter++)
    {
        if((iter->second)>1)
        {
            string str(1, alphabet[table.size()]);
            table[iter->first] = str;
        }
    }
    cout<<"dictionary size: "<< table.size()<<endl;
    return table;
}




//-----------------------------------------coding-----------------------------------------------------------------------//
// 对轮廓进行编码
string  coding(vector<PointXYZRGBNormal>&corners, map<Letter, string> &dictionary)
{
    string word;
    Vec3 gravity(0, 1, 0);
    int corner_num = corners.size();

    // 第一个字符
    {
        Vec3 dir0(0, 0, 1);
        Vec3 dir1( corners[1].x - corners[0].x,
                   corners[1].y - corners[0].y,
                   corners[1].z - corners[0].z);
        float length = dir1.norm();

        dir0.normalize();
        dir1.normalize();
        float sign = gravity * (cross(dir0, dir1));
        float angle = acos(dir0* dir1)*180/3.1415;


        float clockWise;
        if(sign > 0) clockWise = 1;
        else clockWise = 0;

        Letter first(clockWise, angle, length);
        if(dictionary.count(first))
        {
            word  = word + dictionary[first];
        }
        else{
            string str(1, '&');
            word = word + str;
        }
    }
    // 中间字符
    {
        for(int i=1; i< corners.size() -1; i++)
        {
            Vec3 dir0( corners[i].x - corners[i-1].x,
                       corners[i].y - corners[i-1].y,
                       corners[i].z - corners[i-1].z);
            dir0.normalize();

            Vec3 dir1(corners[i+1].x - corners[i].x,
                      corners[i+1].y - corners[i].y,
                      corners[i+1].z - corners[i].z);
            float length = dir1.norm();

            dir1.normalize();
            float sign = gravity*(cross(dir0, dir1));
            float angle = acos(dir0* dir1)*180/3.1416;
            if(abs(angle - 90)< 5) angle = 90.0;

            float clockWise;
            if(sign> 0) clockWise = 1;
            else clockWise = 0;

            Letter mid(clockWise, angle, length);
            if(dictionary.count(mid))
            {
                word  = word + dictionary[mid];
            }
            else{
                string str(1, '&');
                word = word + str;
            }

        }
    }
    // 最后一个字符
    {
        Vec3 dir0( corners[corner_num -1].x - corners[corner_num -2].x,
                   corners[corner_num -1].y - corners[corner_num -2].y,
                   corners[corner_num -1].z - corners[corner_num -2].z);
        dir0.normalize();

        Vec3 dir1(corners[0].x - corners[corner_num-1].x,
                  corners[0].y - corners[corner_num-1].y,
                  corners[0].z - corners[corner_num-1].z);
        float length = dir1.norm();

        dir1.normalize();
        float sign = gravity*(cross(dir0, dir1));
        float angle = acos(dir0* dir1)*180/3.1416;
        if(abs(angle - 90)< 5) angle = 90.0;

        float clockWise;
        if(sign> 0) clockWise = 1;
        else clockWise = 0;

        Letter last(clockWise, angle, length);
        if(dictionary.count(last))
        {
            word  = word + dictionary[last];
        }
        else{
            string str(1, '&');
            word = word + str;
        }

    }
    return word;
}





//-----------------------------------------convertCurvesToWords--------------------------------------------------------//
// 对所有的层的轮廓进行编码
vector<string>  convertCurvesToWords(vector<vector<vector< PointXYZRGBNormal> > > &allCorners,
                                     map< Letter, string> & table)
{
    // 编码
    vector<string> words;
    for(int i=0; i< allCorners.size(); i++)
    {
        string word_per_layer="";
        for(int j=0; j< allCorners[i].size(); j++)
        {
            vector<PointXYZRGBNormal> corners = allCorners[i][j];
            if(corners.size()==0) continue;
            string word = coding(corners, table);
            word_per_layer += word ;
            word_per_layer += " ";
        }
        words.push_back(word_per_layer);
    }
    return words;

}



//-----------------------------------------deCoding---------------------------------------------------------------------//
// 解码，由字符串得到轮廓并进行重建
vector<PointXYZRGBNormal> deCoding(string &word, map<string, Letter> &table_inv)
{
    vector<PointXYZRGBNormal> points;
    Vec3 start(0, 0, 1);
    Vec3 axis(0, 1, 0);

    PointXYZRGBNormal pt(0, 0, 0);
    points.push_back(pt);

    for(int i=0; i<word.size()-1; i++)
    {
        string str(1, word[i]);
        Letter letter = table_inv[str];

        if(letter.clockWise_ ==0 )
            axis[1] = -1;
        if(letter.clockWise_ ==1)
            axis[1] = 1;

        float angle = letter.angle_* 3.1416/180;
        float length = letter.length_;

        cv::Mat rotation;
        rotationMatrixFromAngleAxis(axis, angle, rotation);

        // 获取方向
        cv::Mat start_m(3, 1, CV_32FC1);
        for(int i=0; i<3; i++) start_m.at<float>(i)=  start[i];

        cv::Mat newstart_m = rotation*start_m;
        Vec3 newstart;
        for(int i=0; i< 3; i++)newstart[i] = newstart_m.at<float>(i);

        newstart.normalize();
        newstart = newstart* length;

        pt.x = newstart[0] + points[points.size()-1].x;
        pt.y = newstart[1] + points[points.size()-1].y;
        pt.z = newstart[2] + points[points.size()-1].z;

        points.push_back(pt);

        newstart.normalize();
        start = newstart;
    }

    return points;

}





//------------------------------------------EditDist---------------------------------------------------------------------//
//计算编辑距离
int EditDist(string str0, string str1 )
{
    int M = str0.length() +1;
    int N = str1.length() +1;

    vector<vector<int> > Dist;
    vector<int> tmp;
    tmp.resize(M, 0);
    Dist.resize(N,tmp);

    // first row
    for(int j=0; j< M; j++)
    {
        Dist[0][j] = j;
    }

    // first column
    for(int i=0; i<N; i++)
    {
        Dist[i][0] = i;
    }

    // D[i][j]
    for(int i=1; i< N; i++)
    {
        for(int j=1; j< M; j++)
        {
            int dist = min(Dist[i][j-1] +1, Dist[i-1][j] +1);

            int cost = 0;
            if(str0[j-1] != str1[i -1])
            {
                cost =1;
            }

            dist = min(dist, Dist[i-1][j-1] + cost);

            Dist[i][j] = dist;
        }
    }

    //   for(int i=0; i<Dist.size(); i++)
    //   {
    //       for(int j=0; j<Dist[i].size(); j++)
    //       {
    //           cout<<Dist[i][j]<<", ";
    //       }
    //       cout<<endl;
    //   }

    return Dist[N-1][M-1];
}




//-------------------------------------------similarity-------------------------------------------------------------------//
//计算相似性
float similarity(string str0, string str1)
{
    int length = max(str0.length(), str1.length() );

    if(length == 0) return 1;

    int dist = EditDist(str0, str1);
    //cout<<"Edit dist: "<<dist<<endl;
    return 1 - (float)dist/(float)length;
}




//--------------------------------------------codeCentersClustering------------------------------------------------------//
//对字符串进行聚类，获取聚类中心
// 本质上是一个一个统计的过程，统计个数最多的编码，作为聚类中心
vector<string> codeCentersClustering(vector<string> &words, vector<float> &weights,
                                     vector<vector<PointXYZRGBNormal> >&start_pos,
                                     vector<vector<PointXYZRGBNormal> > & centers_pos)
{
    centers_pos.clear();

    vector<string> centers;
    vector<int> stastics;

    int invalid_num = 0;
    for(int i=0; i< words.size(); i++)
    {
        if(isWordValid(words[i]) == false)
        {
          invalid_num ++;
          continue;
        }
        bool flag = false;
        for(int j=0; j< centers.size(); j++)
        {
            float dist = 1 - similarity(words[i], centers[j]);
            if(dist ==0)
            {
                flag = true;
                stastics[j] ++;
                break;
            }
        }
        if(flag == false)
        {
            centers.push_back(words[i]);
            centers_pos.push_back(start_pos[i]);
            stastics.push_back(1);
        }
    }

    int word_num = words.size();
    weights.resize(centers.size(), 0);
    for(int i=0; i< stastics.size(); i++)
    {
        weights[i] = (float)stastics[i]/(float)(word_num - invalid_num );
    }

    return centers;
}




//---------------------------------------------multiLabelAssignment------------------------------------------------------//
//多目标分配----图优化
vector<string> multiLabelAssignment(vector<string>& words, vector<string> &centers,
                                    vector<float>&weights, vector<vector<PointXYZRGBNormal> >& start_pos,
                                    vector<vector<PointXYZRGBNormal> >& centers_pos,
                                    float sim_thresh, float labmda)
{
    vector<vector<int> > neighbrs;
    neighbrs.resize(words.size());
    for(int i=1; i< words.size()-1; i++)
    {
        neighbrs[i].push_back(i-1);
        neighbrs[i].push_back(i+1);
    }
    neighbrs[0].push_back(1);
    neighbrs[0].push_back(2);
    neighbrs[words.size()-1].push_back(words.size() -2);
    neighbrs[words.size()-1].push_back(words.size() -3);


    alpha_expansion_str AES(words, centers);
    AES.setSimThresh(sim_thresh);
    AES.setMaxIterNum(5);
    AES.setNeighbours(neighbrs);
    AES.setLambda(labmda);
    AES.optimization();

    vector<string> results;
    results.resize(words.size());
    for(int i=0; i< AES.labels_.size(); i++)
    {
        int label = AES.labels_[i];
        results[i] = centers[label];
        start_pos[i].clear();
        start_pos[i].insert(start_pos[i].end(), centers_pos[label].begin(), centers_pos[label].end());
    }
    return results;
}



//---------------------------------------------getBlocks-----------------------------------------------------------------//
//从每层的编码中获取BLOCK3
vector<Block3> getBlocks(vector<string> &words, vector<vector<PointXYZRGBNormal> >& start_pts, float step)
{

    vector< Block3> blocks;

    string str_base = words[0];
    Block3 block;
    block.word_ =  str_base;
    block.start_h_ = start_pts[0][0].y;
    block.end_h_= block.start_h_ + step;
    block.start_pos_.insert(block.start_pos_.end(), start_pts[0].begin(),start_pts[0].end());

    if(words.size()==1)
    {
        blocks.push_back(block);
        return blocks;
    }
    else{
        for(int i=1; i< words.size(); i++)
        {
            string str_cur = words[i];
            if(similarity(str_base, str_cur)== 1)
            {
                block.end_h_ += step;
            }
            else{
                blocks.push_back(block);
                str_base = words[i];
                block.word_ =  str_base;
                block.start_h_ = block.end_h_;

                block.end_h_= block.start_h_ + step;
                block.start_pos_.clear();
                block.start_pos_.insert(block.start_pos_.end(), start_pts[i].begin(),start_pts[i].end());
            }
            if(i == words.size()-1)
            {
                blocks.push_back(block);
            }
        }
    }
}





//---------------------------------------------blockReconstruction------------------------------------------------------//
//对Block进行重建
//Block3::word_中含有多个轮廓，每个轮廓的字符由空格间隔
//重建出的轮廓存储在Block3::curves_中
void blockReconstruction(vector<Block3> &blocks, map<string, Letter> &table_inv)
{
    for(int i=0; i< blocks.size(); i++)
    {
        cout<<i<< "th block" <<endl;

        vector<vector<PointXYZRGBNormal> > curves;
        string word;
        for(string::iterator iter = blocks[i].word_.begin(); iter!=blocks[i].word_.end(); iter++)
        {
            if(*iter != ' ')
            {
                word.push_back(*iter);
            }
            else
            {
                cout<<"Code: " << word<< " ";
                vector<PointXYZRGBNormal> pts = deCoding(word, table_inv);
                curves.push_back(pts);
                word.clear();
            }
        }
        cout<<endl;


        for(int j=0; j< curves.size(); j++)
        {
            for(int k=0; k< curves[j].size(); k++)
            {
                curves[j][k].x += blocks[i].start_pos_[j].x;
                //curves[j][k].y += blocks[i].start_pos_[j].y;
                curves[j][k].z += blocks[i].start_pos_[j].z;
            }
        }
        blocks[i].curves_.clear();
        blocks[i].curves_.insert(blocks[i].curves_.end(), curves.begin(), curves.end());
        // 考虑到起始点的位置
    }
}





//----------------------------------------------distanceFromPointToLine-------------------------------------------------//
// distance between point to line representd by two points
float distanceFromPointToLine(Vec3 pt, vector<Vec3>& line)
{
    Vec3 pt0 = line[0];
    Vec3 pt1 = line[1];

    Vec3 diff = pt - pt0;
    Vec3 dir = pt1 - pt0;
    Vec3 normal(dir.z_, 0, -dir.x_);
    normal.normalize();

    float dis = diff* normal;
    return fabs(dis);
}




//------------------------------------------------projectPtToLine--------------------------------------------------------//
//projet a point onto a line
Vec3 projectPtToLine(Vec3 pt,  vector<Vec3> &line)
{
    Vec3 pt0 =  line[0];
    Vec3 pt1 =  line[1];

    Vec3 diff = pt - pt0;
    Vec3 dir =  pt1 - pt0;

    Vec3 normal(dir.z_, 0, -dir.x_);
    normal.normalize();

    float pro = diff * normal;

    Vec3 pt_p = pt - pro * normal;
    return pt_p;

}





//--------------------------------------------------posProcessing-------------------------------------------------------//
//对Blocks进行后处理，去掉相邻的Block之间的间隙
void posProcessing(vector<Block3> &blocks, float TR)
{

    // 平面的方向要与四个主方向之一对齐
    // 2.0 检测平面

#if 1
    for(int i=0;i< blocks.size()-1; i++)
    {

        int id_base = blocks[i].curves_.size() >= blocks[i+1].curves_.size()? i+1: i;
        int id = (id_base == i)? i+1: i;

        //1.0 取第i层的所有的点作为参考点
        vector<PointXYZRGBNormal> base_pts;
        for(int j=0; j< blocks[id_base].curves_.size(); j++)
        {
            for(int k=0; k< blocks[id_base].curves_[j].size(); k++)
            {
                base_pts.push_back(blocks[id_base].curves_[j][k]);
            }
        }
        vector<int> used;
        used.resize(base_pts.size(), 0);

        //1.1 对于第i+ 1层的每一点，搜索base_pts 中距离最近的点，并将其设置成相同的点
        for(int j=0; j<blocks[id].curves_.size(); j++)
        {
            for(int k=0; k< blocks[id].curves_[j].size(); k++)
            {
                Vec3 pt0( blocks[id].curves_[j][k].x, 0, blocks[id].curves_[j][k].z);

                vector<pair<int, float> > distances;
                for(int m=0; m< base_pts.size(); m++)
                {
                    if(used[m]==1)continue;
                    Vec3 pt1( base_pts[m].x, 0, base_pts[m].z);
                    Vec3 diff = pt1- pt0;
                    distances.push_back(make_pair(m, diff.norm()));
                }
                if(distances.size()==0) continue;
                sort(distances.begin(), distances.end(), comparePairFloatLess);

                if(distances[0].second> TR) continue;
                int index = distances[0].first;
                blocks[id].curves_[j][k].x = base_pts[index].x;
                blocks[id].curves_[j][k].z = base_pts[index].z;
                used[index] =1;
            }
        }
#if 1
        ///////////////////////////////////////////////////////////////////////////////////
        // 2.0 区于第i层所有的直线所谓参考
        vector<vector<Vec3> > lines;
        for(int j=0; j< blocks[id_base].curves_.size(); j++)
        {
            int cNum = blocks[id_base].curves_[j].size();
            for(int k=0; k< cNum; k++)
            {
                vector<Vec3> pts;
                int id0 = k%cNum;
                int id1 = (k+1) % cNum;;
                Vec3 pt0( blocks[id_base].curves_[j][id0].x,
                          blocks[id_base].curves_[j][id0].y,
                          blocks[id_base].curves_[j][id0].z);
                Vec3 pt1( blocks[id_base].curves_[j][id1].x,
                          blocks[id_base].curves_[j][id1].y,
                          blocks[id_base].curves_[j][id1].z);
                pts.push_back(pt0);
                pts.push_back(pt1);
                lines.push_back(pts);
            }
        }

        //2.1 对于第i层的每一点，搜索距离最近的直线，并将点投影到直线上

        for(int j=0; j<blocks[id].curves_.size(); j++)
        {
            for(int k=0; k< blocks[id].curves_[j].size(); k++)
            {
                Vec3 pt0( blocks[id].curves_[j][k].x, 0, blocks[id].curves_[j][k].z);

                vector<pair<int, float> > distances;
                for(int m=0; m< lines.size(); m++)
                {
                    float dis = distanceFromPointToLine(pt0, lines[m]);
                    distances.push_back(make_pair(m, dis));
                }
                if(distances.size()==0) continue;
                sort(distances.begin(), distances.end(), comparePairFloatLess);

                if(distances[0].second> TR) continue;
                int index = distances[0].first;

                Vec3 pt(blocks[id].curves_[j][k].x, 0, blocks[id].curves_[j][k].z );
                pt = projectPtToLine(pt, lines[index]);

                blocks[id].curves_[j][k].x = pt.x_;
                blocks[id].curves_[j][k].z = pt.z_;
            }
        }
#endif

    }
#endif

}




//--------------------------------------------------writeCurvesToOFFFile------------------------------------------------//
//将未进行编码之前的模型三角化，保存成OFF文件
void writeCurvesToOFFFile(vector<vector<vector<PointXYZRGBNormal> > > & allcorners, string file_dir)

{
    map<Vec3, int> table;
    float step =0.1;

    if(allcorners.size()>=2)
    {
      step = allcorners[1][0][0].y - allcorners[0][0][0].y;
    }

     for(int i=0; i< allcorners.size(); i++)
    {
         vector<vector<PointXYZRGBNormal> > curves =  allcorners[i];
         float lower = curves[0][0].y;
         float upper = lower + step ;

         for(int j=0; j< curves.size(); j++)
         {
             for(int k=0; k< curves[j].size(); k++)
             {
                  Vec3 vec_low(curves[j][k].x, lower, curves[j][k].z);
                  Vec3 vec_up (curves[j][k].x, upper, curves[j][k].z);

                 if(table.count(vec_low)==0)
                 {
                     table.insert(make_pair(vec_low, 0));
                 }

                 if(table.count(vec_up)==0)
                 {
                     table.insert(make_pair(vec_up, 0));
                 }
             }
         }
    }

     //注意，此时的table 中的值 并不是要保存的索引值，因为插入元素后，table中的顺序发生改变了
     int index = 0;
     for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
     {
        iter->second = index;
        index++;
     }

     vector<vector<int> > facets;
     for(int i=0; i< allcorners.size(); i++)
     {
         vector<vector<PointXYZRGBNormal> > curves =  allcorners[i];
         float lower = curves[0][0].y;
         float upper = lower + step ;


         for(int j=0; j< curves.size(); j++)
         {
             int num = curves[j].size();
             for(int k=0; k< curves[j].size(); k++)
             {
                int id0 = k% num;
                int id1 = (k+1) % num;

                Vec3 vec0(curves[j][id0].x, lower,  curves[j][id0].z);
                Vec3 vec1(curves[j][id1].x, lower,  curves[j][id1].z);
                Vec3 vec2(curves[j][id1].x, upper,  curves[j][id1].z);
                Vec3 vec3(curves[j][id0].x, upper,  curves[j][id0].z);

                vector<int> indices0;
                indices0.push_back(table[vec0]);
                indices0.push_back(table[vec1]);
                indices0.push_back(table[vec3]);
                facets.push_back(indices0);

                vector<int> indices1;
                indices1.push_back(table[vec1]);
                indices1.push_back(table[vec2]);
                indices1.push_back(table[vec3]);
                facets.push_back(indices1);

             }
         }
     }

     int pts_num = table.size();
     int facet_num = facets.size();

     ofstream writef;
     writef.open(file_dir.c_str(), ios::out);
     writef<<"COFF"<<endl;
     writef<<pts_num<<" "<< facet_num<<" "<< "0"<<endl;

     for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
     {
         writef<< iter->first.x_<<" "<< iter->first.y_<<" "<< iter->first.z_<<endl;
     }

     for(int i=0; i< facets.size(); i++)
     {
         writef<< facets[i].size()<<" ";
         for(int j=0; j< facets[i].size(); j++)
         {
             writef<<facets[i][j]<<" ";
         }
         writef<<endl;
     }

     writef.close();




}




//--------------------------------------------------writeBlocksToOFFFile------------------------------------------------//
//将BLOCK进行三角化，并保存成OFF文件
void writeBlocksToOFFFile(const vector<Block3> &blocks, string file_dir)
{

    //1.0 collect  points  and indices
    map<Vec3, int> table;
    for(int i=0; i< blocks.size(); i++)
    {
        vector<vector<PointXYZRGBNormal> > curves =  blocks[i].curves_;
        float lower = blocks[i].start_h_;
        float upper = blocks[i].end_h_;

        for(int j=0; j< curves.size(); j++)
        {
            for(int k=0; k< curves[j].size(); k++)
            {
                 Vec3 vec_low(curves[j][k].x, lower, curves[j][k].z);
                 Vec3 vec_up (curves[j][k].x, upper, curves[j][k].z);

                if(table.count(vec_low)==0)
                {
                    table.insert(make_pair(vec_low, 0));
                }

                if(table.count(vec_up)==0)
                {
                    table.insert(make_pair(vec_up, 0));
                }
            }
        }
    }

    //注意，此时的table 中的值 并不是要保存的索引值，因为插入元素后，table中的顺序发生改变了
    int index = 0;
    for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
    {
       iter->second = index;
       index++;
    }


   //2.0 获取平面信息
    vector<vector<int> > facets;
    for(int i=0; i< blocks.size(); i++)
    {
        vector<vector<PointXYZRGBNormal> > curves = blocks[i].curves_;
        float lower = blocks[i].start_h_;
        float upper = blocks[i].end_h_;

        for(int j=0; j< curves.size(); j++)
        {
            int num = curves[j].size();
            for(int k=0; k< curves[j].size(); k++)
            {
               int id0 = k% num;
               int id1 = (k+1) % num;

               Vec3 vec0(curves[j][id0].x, lower,  curves[j][id0].z);
               Vec3 vec1(curves[j][id1].x, lower,  curves[j][id1].z);
               Vec3 vec2(curves[j][id1].x, upper,  curves[j][id1].z);
               Vec3 vec3(curves[j][id0].x, upper,  curves[j][id0].z);

               vector<int> indices0;
               indices0.push_back(table[vec0]);
               indices0.push_back(table[vec1]);
               indices0.push_back(table[vec3]);
               facets.push_back(indices0);

               vector<int> indices1;
               indices1.push_back(table[vec1]);
               indices1.push_back(table[vec2]);
               indices1.push_back(table[vec3]);
               facets.push_back(indices1);

            }
        }
    }


    //3.0 写文档
     int pts_num = table.size();
     int facet_num = facets.size();

     ofstream writef;
     writef.open(file_dir.c_str(), ios::out);
     writef<<"COFF"<<endl;
     writef<<pts_num<<" "<< facet_num<<" "<< "0"<<endl;

     for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
     {
         writef<< iter->first.x_<<" "<< iter->first.y_<<" "<< iter->first.z_<<endl;
     }

     for(int i=0; i< facets.size(); i++)
     {
         writef<< facets[i].size()<<" ";
         for(int j=0; j< facets[i].size(); j++)
         {
             writef<<facets[i][j]<<" ";
         }
         writef<<endl;
     }

     writef.close();
}




//--------------------------------------------------gettingTriangulationsFromAllCorners---------------------------------//
//从编码的曲线中获取三角面片
void gettingTriangulationsFromAllCorners(const vector<vector<vector<PointXYZRGBNormal> > > & allcorners,
                                         vector<Vec3>& vertices, vector<vector<int> > & facets)
{
    facets.clear();
    vertices.clear();

    map<Vec3, int> table;
    float step =0.1;

    if(allcorners.size()>=2)
    {
      step = allcorners[1][0][0].y - allcorners[0][0][0].y;
    }

     for(int i=0; i< allcorners.size(); i++)
    {
         vector<vector<PointXYZRGBNormal> > curves =  allcorners[i];
         float lower = curves[0][0].y;
         float upper = lower + step ;

         for(int j=0; j< curves.size(); j++)
         {
             for(int k=0; k< curves[j].size(); k++)
             {
                  Vec3 vec_low(curves[j][k].x, lower, curves[j][k].z);
                  Vec3 vec_up (curves[j][k].x, upper, curves[j][k].z);

                 if(table.count(vec_low)==0)
                 {
                     table.insert(make_pair(vec_low, 0));
                 }

                 if(table.count(vec_up)==0)
                 {
                     table.insert(make_pair(vec_up, 0));
                 }
             }
         }
    }

     //注意，此时的table 中的值 并不是要保存的索引值，因为插入元素后，table中的顺序发生改变了
     int index = 0;
     for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
     {
        iter->second = index;
        index++;
     }

     for(int i=0; i< allcorners.size(); i++)
     {
         vector<vector<PointXYZRGBNormal> > curves =  allcorners[i];
         float lower = curves[0][0].y;
         float upper = lower + step ;

         for(int j=0; j< curves.size(); j++)
         {
             int num = curves[j].size();
             for(int k=0; k< curves[j].size(); k++)
             {
                int id0 = k% num;
                int id1 = (k+1) % num;

                Vec3 vec0(curves[j][id0].x, lower,  curves[j][id0].z);
                Vec3 vec1(curves[j][id1].x, lower,  curves[j][id1].z);
                Vec3 vec2(curves[j][id1].x, upper,  curves[j][id1].z);
                Vec3 vec3(curves[j][id0].x, upper,  curves[j][id0].z);

                vector<int> indices0;
                indices0.push_back(table[vec0]);
                indices0.push_back(table[vec1]);
                indices0.push_back(table[vec3]);
                facets.push_back(indices0);

                vector<int> indices1;
                indices1.push_back(table[vec1]);
                indices1.push_back(table[vec2]);
                indices1.push_back(table[vec3]);
                facets.push_back(indices1);

             }
         }
     }

     for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
     {
	 vertices.push_back(iter->first);
     }

}



//---------------------------------------------------gettingTriangulationsFromBlocks------------------------------------//
///////////////////////////////从blocks 中获取三角面片///////////////////////////////////////////////////
void gettingTriangulationsFromBlocks(const vector<Block3> &blocks, 
                                     vector<Vec3>& vertices, vector<vector<int> >&facets)
{
    facets.clear();
    vertices.clear();

    //1.0 collect  points  and indices
    map<Vec3, int> table;
    for(int i=0; i< blocks.size(); i++)
    {
        vector<vector<PointXYZRGBNormal> > curves =  blocks[i].curves_;
        float lower = blocks[i].start_h_;
        float upper = blocks[i].end_h_;

        for(int j=0; j< curves.size(); j++)
        {
            for(int k=0; k< curves[j].size(); k++)
            {
                 Vec3 vec_low(curves[j][k].x, lower, curves[j][k].z);
                 Vec3 vec_up (curves[j][k].x, upper, curves[j][k].z);

                if(table.count(vec_low)==0)
                {
                    table.insert(make_pair(vec_low, 0));
                }

                if(table.count(vec_up)==0)
                {
                    table.insert(make_pair(vec_up, 0));
                }
            }
        }
    }

    //注意，此时的table 中的值 并不是要保存的索引值，因为插入元素后，table中的顺序发生改变了
    int index = 0;
    for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
    {
       iter->second = index;
       index++;
    }


   //2.0 获取平面信息
    for(int i=0; i< blocks.size(); i++)
    {
        vector<vector<PointXYZRGBNormal> > curves = blocks[i].curves_;
        float lower = blocks[i].start_h_;
        float upper = blocks[i].end_h_;

        for(int j=0; j< curves.size(); j++)
        {
            int num = curves[j].size();
            for(int k=0; k< curves[j].size(); k++)
            {
               int id0 = k% num;
               int id1 = (k+1) % num;

               Vec3 vec0(curves[j][id0].x, lower,  curves[j][id0].z);
               Vec3 vec1(curves[j][id1].x, lower,  curves[j][id1].z);
               Vec3 vec2(curves[j][id1].x, upper,  curves[j][id1].z);
               Vec3 vec3(curves[j][id0].x, upper,  curves[j][id0].z);

               vector<int> indices0;
               indices0.push_back(table[vec0]);
               indices0.push_back(table[vec1]);
               indices0.push_back(table[vec3]);
               facets.push_back(indices0);

               vector<int> indices1;
               indices1.push_back(table[vec1]);
               indices1.push_back(table[vec2]);
               indices1.push_back(table[vec3]);
               facets.push_back(indices1);

            }
        }
    }

     for(map<Vec3, int> ::iterator iter = table.begin(); iter!= table.end(); iter++)
     {
	 vertices.push_back(iter->first);
     }
 
}



//---------------------------------------------------writeOFFFiles-----------------------------------------------------//
void writeOFFFiles(const vector<Vec3>& vertices, const vector<vector<int> >&facets, string file_dir)
{
     int pts_num = vertices.size();
     int facet_num = facets.size();

     ofstream writef;
     writef.open(file_dir.c_str(), ios::out);
     writef<<"COFF"<<endl;
     writef<<pts_num<<" "<< facet_num<<" "<< "0"<<endl;

     for(int i=0; i< vertices.size(); i++)
     {
         writef<< vertices[i].x_<<" "<< vertices[i].y_<<" "<< vertices[i].z_<<endl;
     }

     for(int i=0; i< facets.size(); i++)
     {
         writef<< facets[i].size()<<" ";
         for(int j=0; j< facets[i].size(); j++)
         {
             writef<<facets[i][j]<<" ";
         }
         writef<<endl;
     }

     writef.close();
}



















