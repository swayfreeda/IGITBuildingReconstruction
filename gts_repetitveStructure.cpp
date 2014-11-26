#include"gts_minESingleV1.h"

#include"gts_repetitiveStructure.h"
#include<QFileDialog>
#include"opencv2/opencv.hpp"
#include"sw_functions.h"
#include <opencv2/core/core.hpp>
#include "opencv2/legacy/legacy.hpp"
#include<cmath>
#include<QTextStream>
#include<QTransform>

#include<QMessageBox>
#include<QRgb>

#define PI 3.1415916



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double gaussian(const Vec3 &sample, const Vec3 & center, const cv::Mat & cov)
{
    cv::Mat var(1, 3, CV_64FC1);
    for(int i=0; i< 3; i++)
    {
        var.at<double> (i) = (double)( sample[i] - center[i] );
    }

    cv::Mat vm = var *cov.inv()*var.t();

    double tmp  = vm.at<double> (0);
    tmp = exp(-0.5* tmp);

    //cout<<"det: "<< determinant(cov)<<endl;

    double w =  sqrt( determinant(cov));

    return 1.0/w * tmp;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void EMTraining(const vector<Vec3> &samples, int nCluster,
                vector<Vec3> &centers, vector<float> &weights, vector<cv::Mat> & covs)
{
    centers.clear();
    weights.clear();
    covs.clear();

    centers.resize(nCluster);
    weights.resize(nCluster);
    covs.resize(nCluster);

    /*************************EM 算法 ********************************/
#if 0
    cv::Mat samplesM(samples.size(), 3, CV_32FC1);
    cv::Mat labelsM  (samples.size(), 1, CV_32SC1);
    for(int i=0; i< samples.size(); i++)
    {
        samplesM.at<float>(i, 0) = samples[i].x_;
        samplesM.at<float>(i, 1) = samples[i].y_;
        samplesM.at<float>(i, 2) = samples[i].z_;
    }

    CvEM em_model;
    CvEMParams params;
    params.covs = NULL;
    params.means = NULL;
    params.weights = NULL;
    params.probs = NULL;
    params.nclusters = nCluster;
    params.cov_mat_type = CvEM::COV_MAT_GENERIC;
    params.start_step   = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 300;
    params.term_crit.epsilon = 0.01;
    params.term_crit.type = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;


    em_model.train(samplesM, cv::Mat(), params,&labelsM);

    // get means
    cv::Mat meansM = em_model.getMeans();

    // get weights
    cv::Mat weightsM = em_model.getWeights();

    // get covs
    em_model.getCovs(covs);

    for(int i=0; i< nCluster; i++)
    {
        weights[i] = weightsM.at<double>(i);
    }

    for(int i=0; i< meansM.rows; i++)
    {
        for(int j=0; j< meansM.cols; j++)
        {
            centers[i][j] = meansM.at<double>(i, j);
        }
    }

#endif
    /*************************Kmeans 算法 ********************************/
    cv::Mat samplesM(samples.size(), 3, CV_32FC1);
    cv::Mat labelsM  (samples.size(), 1, CV_32SC1);
    for(int i=0; i< samples.size(); i++)
    {
        samplesM.at<float>(i, 0) = samples[i].x_;
        samplesM.at<float>(i, 1) = samples[i].y_;
        samplesM.at<float>(i, 2) = samples[i].z_;
    }

    cv::TermCriteria criteria;
    criteria.maxCount = 300;
    criteria.epsilon  = 0.01;

    cv::kmeans(samplesM, nCluster,labelsM , criteria, 100, 0);

    // collect indexs for each cluster
    vector<vector<int> > clusters;
    clusters.resize(nCluster);
    for(int i=0; i< labelsM.rows; i++)
    {
        int label = labelsM.at<int>(i);
        clusters[label].push_back(i);
    }

    // compute centers
    for(int i=0; i< clusters.size(); i++)
    {
        int num = clusters[i].size();
        for(int j=0;j< num; j++)
        {
            int id = clusters[i][j];
            centers[i] =centers[i] + samples[id]/(float)num;
        }
    }

    // compute weights
    for(int i=0; i< clusters.size(); i++)
    {
        weights[i] =(float) clusters[i].size()/(float)samples.size();
    }

    // compute covarx matrix
    for(int i=0; i< clusters.size(); i++)
    {
        int num  = clusters[i].size();
        cv::Mat subSamples(num, 3, CV_64FC1);
        for(int j=0; j< num; j++)
        {
            int id = clusters[i][j];
            subSamples.at<double>(j, 0) = samples[id].x_ - centers[i].x_;
            subSamples.at<double>(j, 1) = samples[id].y_ - centers[i].y_;
            subSamples.at<double>(j, 2) = samples[id].z_ - centers[i].z_;
        }
        cv::Mat  tmp = subSamples.t()* subSamples/(num-1);
        //cout<<"subSamples: "<<subSamples<<endl;
        tmp.copyTo(covs[i]);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double EMPredict(vector<Vec3> &centers, vector<float> &weights, vector<cv::Mat> & covs, Vec3 &sample)
{
    double pro = 0.0;
    for(int i=0; i< centers.size(); i++)
    {
        pro += weights[i]*gaussian(sample, centers[i], covs[i]);
    }
    return pro;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QPoint transformedPoint(const QPoint & pt, const cv::Mat & H)
{
    cv::Mat ptM(3, 1, CV_64FC1);
    ptM.at<double>(0) =  (double)pt.x();
    ptM.at<double>(1) =  (double)pt.y();
    ptM.at<double>(2) = 1.0;
    cv::Mat dstPt = H*ptM;

    QPoint result;
    result.setX( dstPt.at<double>(0)/dstPt.at<double>(2));
    result.setY( dstPt.at<double>(1)/dstPt.at<double>(2));

    ptM.release();
    dstPt.release();

    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ValidPoint(const QPoint &pt, int  height, int width)
{
    return ( pt.y()>=0&& pt.y()< height && pt.x()>=0&& pt.x()<width);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTSDetectionDialog::GTSDetectionDialog(QWidget *parent)
{
    setupUi(this);

    m_scale_ = 1.00;
    m_with_rectification_ = false;

    label_input->setBackgroundColor(Qt::gray);
    widget_rectified->setBackgroundColor(Qt::gray);
    label_output->setBackgroundColor(Qt::gray);
    label_initialResults->setBackgroundColor(Qt::gray);

    // signals and slots
    connect(pushButton_loadImage, SIGNAL(clicked()), this, SLOT(loadImage()));
    connect(pushButton_background, SIGNAL(clicked()), widget_rectified, SLOT(drawStrokeOnBackground()));
    connect(pushButton_foreground, SIGNAL(clicked()), widget_rectified,SLOT(drawRectOnForeground()));
    connect(pushButton_detect, SIGNAL(clicked()), this, SLOT(detect()));
    // connect(verticalSlider_imgScale, SIGNAL(valueChanged(int)), SLOT(imageScale(int)));

    connect(pushButton_rectification, SIGNAL(clicked()), this, SLOT(rectification()));
    connect(pushButton_quad, SIGNAL(clicked()), widget_rectified, SLOT(drawQuad()));

    connect(this, SIGNAL(dispFinalResultSignal()), this, SLOT(dispFinalResult()));
    connect(this, SIGNAL(dispInitialResultSignal()), this, SLOT(dispInitialResult()));

    connect(pushButton_abandon, SIGNAL(clicked()), this, SLOT(abandonResults()));
    connect(pushButton_accept, SIGNAL(clicked()), this, SLOT(acceptResults()));

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QVector<QVector<double> > GTSDetectionDialog::getInitialLabels()
{

#if 1
    // EM clusters
    vector<Vec3> bg_samples;
    vector<Vec3> gts_samples;

    QImage img = m_rected_image_.copy(0, 0, m_rected_image_.width(), m_rected_image_.height());
    img.convertToFormat(QImage::Format_RGB32);

    /*******************get features(RGB) from image ***********/
    uchar *imgBits = img.bits();
    int height = img.height();
    int width = img.width();

    // back ground samples
    foreach(QPoint pt, m_bg_samples)
    {
        int i = pt.y();
        int j = pt.x();

        Vec3 sample;
        int line_num = i* width *4;
        sample[0] = (float)imgBits[line_num + j*4 + 0];
        sample[1] = (float)imgBits[line_num + j*4 + 1];
        sample[2] = (float)imgBits[line_num + j*4 + 2];

        if(sample.x_ ==0&& sample.y_ ==0&& sample.z_ ==0)continue;

        /************还需要加上三维的一些信息，来确定建筑的范围 ********************************/

        /********************************************************************************/
        bg_samples.push_back(sample);
    }

    // gts samples
    int start_h = m_GTS_pos_[0].topLeft().y();
    int end_h = m_GTS_pos_[0].bottomRight().y();

    int start_w = m_GTS_pos_[0].topLeft().x();
    int end_w = m_GTS_pos_[0].bottomRight().x();

    for(int i=start_h; i<=end_h; i++)
    {
        for(int j=start_w; j<=end_w; j++ )
        {
            int line_num = i* width *4;

            Vec3 sample;

            sample[0] = (float)imgBits[line_num + j*4 + 0];
            sample[1] = (float)imgBits[line_num + j*4 + 1];
            sample[2] = (float)imgBits[line_num + j*4 + 2];

            gts_samples.push_back(sample);
        }
    }


    /**********************EM clusters******************************/
    int nCluster = 3;
   if(bg_samples.size()< nCluster&& gts_samples.size()< nCluster)
   {
       QMessageBox::warning(this, tr("Warning"), tr("Samples are not Enough!"));
       m_bg_samples.clear();
       m_GTS_pos_.clear();
   }
   else{

       vector<Vec3> bg_centers;
       vector<float> bg_weights;
       vector<cv::Mat> bg_covs;
       EMTraining(bg_samples, nCluster, bg_centers, bg_weights, bg_covs);

       //    for(int i=0; i< bg_covs.size(); i++)
       //    {
       //        cout<<"center: "<< bg_centers[i].x_<<", "<<bg_centers[i].y_<<", "<< bg_centers[i].z_ <<endl;
       //        cout<<"cov matrix: "<<bg_covs[i]<<endl;
       //        cout<<"inv: "<< bg_covs[i].inv()<<endl;
       //        cout<<"det: "<<determinant(bg_covs[i])<<endl;
       //    }

       vector<Vec3> gts_centers;
       vector<float> gts_weights;
       vector<cv::Mat> gts_covs;
       EMTraining(gts_samples, nCluster, gts_centers, gts_weights, gts_covs);

       //    for(int i=0; i< bg_covs.size(); i++)
       //    {
       //        cout<<"center: "<< gts_centers[i].x_<<", "<<gts_centers[i].y_<<", "<< gts_centers[i].z_ <<endl;
       //        cout<<"cov matrix: "<<gts_covs[i]<<endl;
       //        cout<<"inv: "<<gts_covs[i].inv()<<endl;
       //        cout<<"det: "<<determinant(gts_covs[i])<<endl;
       //    }

       vector<vector<double> > bg_probs;
       vector<vector<double> > gts_probs;

       bg_probs.resize(height);
       gts_probs.resize(height);
       for(int i=0; i<height; i++)
       {
           int line_num_img = i* width *4;

           for(int j=0; j< width; j++)
           {

               // 不在 mask 范围之内的标定为背景
               if(!m_rected_mask_.containsPoint(QPoint(j, i), Qt::OddEvenFill))
               {
                   bg_probs[i].push_back(0.01);
                   gts_probs[i].push_back(0.99);
                   continue;
               }
               Vec3 sample;
               sample[0] = (float)imgBits[line_num_img + j*4 + 0];
               sample[1] = (float)imgBits[line_num_img + j*4 + 1];
               sample[2] = (float)imgBits[line_num_img + j*4 + 2];

               if(sample.x_ ==0&& sample.y_ ==0&& sample.z_ ==0)
               {
                   bg_probs[i].push_back(-1);
                   gts_probs[i].push_back(-1);
               }

               else{
                   double gts_prob = EMPredict(gts_centers, gts_weights, gts_covs, sample);
                   double bg_prob = EMPredict(bg_centers, bg_weights, bg_covs, sample);

                   // cout<<"gts_prob: "<< gts_prob<<", "<< "bg_prob: "<< bg_prob<<endl;
                   bg_probs[i].push_back(-log(bg_prob));
                   gts_probs[i].push_back(-log(gts_prob));
               }
           }
       }

       // 1:  for gts
       // 0:  for back ground
       QVector<QVector<double> > initial_labels;
       initial_labels.resize(height);
       for(int i=0; i< bg_probs.size(); i++)
       {
           for(int j=0; j< bg_probs[i].size(); j++)
           {
               if(bg_probs[i][j]==-1&&gts_probs[i][j]==-1)
               {
                   initial_labels[i].append(-1);
               }
               else{
                   double label = bg_probs[i][j]> gts_probs[i][j]? 1: 0;
                   initial_labels[i].append(label);
               }
           }
       }
       return initial_labels;
   #endif
   }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double GTSDetectionDialog::computeEnergy(QVector<QVector<double> > &P,
                                         QVector<QVector<double> > &R, QVector<QVector<double> > &Q)
{

    QVector<QVector< double> > labels = getLabels(P, R, Q);

    double error = 0;
    for(int i=0; i< labels.size(); i++)
    {
        for(int j=0; j< labels[i].size(); j++)
        {
            double tmp = abs( m_initial_labels_[i][j] -  labels[i][j]);
            if( tmp !=0)
            {
                error +=  tmp;
            }
        }
    }
    return error;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QVector<QVector<double> > GTSDetectionDialog::optimizeP(QVector<QVector<double> > &R,QVector<QVector<double> > &Q)
{

    // trans of Q
    QVector<QVector<double> > QTM;
    QTM.resize(m_img_w_);
    for(int i=0; i< Q.size(); i++)
    {
        for(int j=0; j< Q[i].size(); j++)
        {
            QTM[j].append(Q[i][j]);
        }
    }

    // get the cols that more than 0ne values
    QVector<int> col_ids;
    for(int i=0; i< QTM.size(); i++)
    {
        if(QTM[i].count(1)> 0)
        {
            col_ids.append(i);
        }
    }

    //Y
    double* Yptr = new double[m_img_h_ * (int)col_ids.size()];
    for(int i=0;i < m_initial_labels_.size(); i++)
    {
        for(int j=0; j<col_ids.size(); j++ )
        {
            int id = col_ids[j];
            Yptr[j* m_img_h_ +i] = m_initial_labels_[i][id];
        }
    }

    // X
    double *Xptr = new double[m_rect_h_ * (int)col_ids.size()];
    cv::Mat RM(m_rect_h_, m_rect_w_, CV_64FC1);
    for(int i=0; i< R.size(); i++)
    {
        for(int j=0; j< R[i].size(); j++)
        {
            RM.at<double>(i,j) = R[i][j];
        }
    }
    cv::Mat subQM(m_rect_w_, (int) col_ids.size(), CV_64FC1);
    for(int i=0; i< Q.size(); i++)
    {
        for(int j=0; j< col_ids.size(); j++)
        {
            int id = col_ids[j];
            subQM.at<double>(i, j) = Q[i][id];
        }
    }
    cv::Mat XM = RM* subQM;
    for(int i=0; i< XM.rows; i++)
    {
        for(int j=0; j< XM.cols; j++)
        {
            Xptr[j* XM.rows +i] = XM.at<double>(i, j);
        }
    }

    double * YTptr = new double [ (int)col_ids.size() * m_img_h_];
    for(int i=0; i<col_ids.size(); i++ )
    {
        for(int j=0; j< m_img_h_; j++)
        {
            YTptr[ j* col_ids.size() + i] = Yptr[ i * m_img_h_ + j];
        }
    }

    double * XTptr = new double [(int)col_ids.size() * m_rect_h_];
    for(int i=0; i<col_ids.size(); i++ )
    {
        for(int j=0; j< m_rect_h_; j++)
        {
            XTptr[j* col_ids.size() + i] = Xptr[ i* m_rect_h_ +j];
        }
    }

    //trans of P
    double* PTptr = new double[m_rect_h_ * m_img_h_];
    for(int i=0; i< m_rect_h_; i++)
    {
        for(int j=0; j< m_img_h_; j++)
        {
            PTptr[j* m_rect_h_ + i] = 0;
        }
    }



    int H = (int)col_ids.size();
    int W = m_img_h_;
    int N = m_rect_h_;
    int block = 0;

    int r1 = m_GTS_pos_[0].topLeft().y();
    int r2 = m_GTS_pos_[0].bottomRight().y();

    // optimimization
    MinESingle(PTptr, YTptr , Xptr, H, W, N, r1,r2,block);

    QVector<QVector<double> > P;
    P.resize(m_img_h_);
    for(int i=0; i< P.size(); i++)
    {
        for(int j=0; j< m_rect_h_; j++)
        {
            P[i].append( PTptr[i* m_rect_h_ +j]);
        }
    }

    RM.release();
    subQM.release();
    XM.release();

    delete [] Yptr;
    delete [] PTptr;
    delete [] Xptr;
    delete [] XTptr;
    delete [] YTptr;

    return P;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QVector<QVector<double> > GTSDetectionDialog::optimizeQ(QVector<QVector<double> > &P, QVector<QVector<double> > &R)
{
    // get the rows that has value 1
    QVector<int> row_ids;
    for(int i=0; i< P.size(); i++)
    {
        if(P[i].count(1)> 0)
        {
            row_ids.append(i);
        }
    }

    //coressponding labels // col priority
    double * Yptr = new double[ (int)row_ids.size() * m_img_w_];
    for(int i=0; i< row_ids.size(); i++)
    {
        int id = row_ids[i];
        for(int j=0; j< m_initial_labels_[id].size(); j++)
        {
            Yptr[j * row_ids.size() + i ] = m_initial_labels_[id][j];
        }
    }

    // P(ids, :)*R
    double *Xptr = new double[(int)row_ids.size() * m_rect_w_ ];
    cv::Mat subPM(row_ids.size(), m_rect_h_, CV_64FC1);
    for(int i=0; i< row_ids.size(); i++)
    {
        int id = row_ids[i];
        for(int j=0; j< P[id].size(); j++)
        {
            subPM.at<double>(i,j) = P[id][j];
        }
    }
    cv::Mat RM(m_rect_h_, m_rect_w_, CV_64FC1);
    for(int i=0; i< R.size(); i++)
    {
        for(int j=0; j< R[i].size(); j++)
        {
            RM.at<double>(i,j) = R[i][j];
        }
    }
    cv::Mat XM = subPM * RM;
    for(int i=0; i< XM.rows; i++)
    {
        for(int j=0; j< XM.cols; j++)
        {
            Xptr[j* XM.rows + i] = XM.at<double> (i, j);
        }
    }

    // Q
    double *Qptr = new double [m_rect_w_ * m_img_w_];
    for(int i=0; i< m_rect_w_; i++)
    {
        for(int j=0; j< m_img_w_; j++)
        {
            Qptr[j * m_rect_w_ + i] = 0;
        }
    }

    int H = (int) row_ids.size();
    int W = m_img_w_;
    int N = m_rect_w_;
    int block = 0;

    int c1 = m_GTS_pos_[0].topLeft().x();
    int c2 = m_GTS_pos_[0].bottomRight().x();

    // optimimization
    MinESingle(Qptr, Yptr , Xptr, H, W, N, c1,c2,block);

    QVector<QVector<double> > Q;
    Q.resize(m_rect_w_);
    for(int i=0; i< m_rect_w_; i++)
    {
        for(int j=0; j< m_img_w_; j++)
        {
            Q[i].append(Qptr[j * m_rect_w_+ i]);
        }
    }

    subPM.release();
    RM.release();
    XM.release();
    delete []Qptr;
    delete []Yptr;
    delete []Xptr;

    return Q;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// get labels given P,R and Q
QVector<QVector<double> > GTSDetectionDialog::getLabels(QVector<QVector<double> > &P,
                                                        QVector<QVector<double> > &R,QVector<QVector<double> > &Q)
{

    QVector<QVector< double > > labels;
    cv::Mat PM(m_img_h_, m_rect_h_, CV_32FC1);
    cv::Mat RM((int)m_rect_h_, (int)m_rect_w_, CV_32FC1);
    RM.setTo(1.0);
    cv::Mat QM(m_rect_w_, m_img_w_, CV_32FC1);
    // P
    for(int i=0; i< P.size(); i++)
    {
        for(int j=0; j< P[i].size(); j++)
        {
            PM.at<float>(i, j) = (float)P[i][j];
        }
    }

    // R
    for(int i=0; i< R.size(); i++)
    {
        for(int j=0; j< R[i].size(); j++)
        {
            RM.at<float>(i, j) = (float)R[i][j];
        }
    }

    //Q
    for(int i=0; i<Q.size(); i++)
    {
        for(int j=0; j< Q[i].size(); j++)
        {
            QM.at<float>(i, j) = (float)Q[i][j];
        }
    }

    // L = P*R*Q

    cv::Mat L = PM * RM * QM;

    labels.resize(L.rows);
    for(int i=0; i< L.rows; i++)
    {
        for(int j=0; j< L.cols; j++)
        {
            labels[i].append( L.at<float> (i, j));// in case of floatting
        }
    }

    PM.release();
    RM.release();
    QM.release();
    L.release();

    return labels;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::optimization(QVector<QVector<double> > &P,
                                      QVector<QVector<double> > &R,QVector<QVector<double> > &Q)
{

    int maxIter = 50;
    double epsion = 0.1;

    int iter =0;

    m_E_ = computeEnergy(P, R, Q);
    cout<<"initial E: "<< m_E_<<endl;

    while(iter < maxIter)
    {
        QVector<QVector<double> > Q_tmp = optimizeQ(P, R);

        Q.swap(Q_tmp);

        QVector<QVector<double> > P_tmp = optimizeP(R, Q);

        P.swap(P_tmp);

        double pre_E = m_E_;
        m_E_ = computeEnergy(P, R, Q);

        cout<< iter<<" th iteration: "<< m_E_<<endl;

        if(abs(m_E_ - pre_E)< epsion) break;

        iter++;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::wheelEvent(QWheelEvent *event)
{
    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15;

    m_scale_ += (float)numSteps* 0.05;

    imageScale();

    event->accept();
    update();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::loadImage()
{
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open Image File"),".",
                                                     tr("Image Files(*.png *.PNG *JPEG *.jpg)") );
    if(!file_name.isEmpty())
    {
        m_src_image_.load(file_name);
    }

    m_image_ = m_src_image_.scaled(QSize(m_scale_* m_src_image_.width(),
                                         m_scale_* m_src_image_.height()) );
    m_img_h_ = m_image_.height();
    m_img_w_ = m_image_.width();

    label_input->setPixmap(QPixmap::fromImage(m_image_));
    //    label_input->setFixedHeight(m_image_.height());
    //    label_input->setFixedWidth(m_image_.width());


    widget_rectified->setImage(m_image_);
    widget_rectified->setDrawImage(true);


    // 调整显示的区域的大小
    //    widget_rectified->setFixedHeight(m_image_.height());
    //    widget_rectified->setFixedWidth(m_image_.width());

    //    label_initialResults->setFixedHeight(m_image_.height());
    //    label_initialResults->setFixedWidth(m_image_.width());

    //    label_output->setFixedHeight(m_image_.height());
    //    label_output->setFixedWidth(m_image_.width());
    update();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::rectification()
{
    // used for rectification
    QVector<QPoint> quad = widget_rectified->getQuad();


    if(quad.size() ==0)
    {
        QMessageBox::warning(this, tr("Warning"), tr("Draw a Quadrilateral First!"));
    }
    else{
        // src points
        vector<cv::Point2f> src_pts;
        src_pts.resize(quad.size());
        for(int i=0; i< quad.size(); i++)
        {
            src_pts[i].x = (float)quad[i].x();
            src_pts[i].y = (float)quad[i].y();
        }
        // dst points
        float min_x = 10000;  float min_y = 10000;
        float max_x = -10000; float max_y = -10000;

        for(int i=0; i< src_pts.size(); i++)
        {
            if(min_x> src_pts[i].x) min_x = src_pts[i].x;
            if(min_y> src_pts[i].y) min_y = src_pts[i].y;
            if(max_x< src_pts[i].x) max_x = src_pts[i].x;
            if(max_y< src_pts[i].y) max_y = src_pts[i].y;
        }

        cv::Point2f dst0(min_x, min_y);    cv::Point2f dst1(max_x, min_y);
        cv::Point2f dst2(max_x, max_y);    cv::Point2f dst3(min_x, max_y);

        vector<cv::Point2f> dst_pts;
        dst_pts.push_back(dst0); dst_pts.push_back(dst1);
        dst_pts.push_back(dst2); dst_pts.push_back(dst3);

#if 0
        for(int i=0; i< src_pts.size(); i++)
        {
            cout<<"src: ( "<<src_pts[i].x<<", "<< src_pts[i].y<<" )------>";
            cout<<"dst: ( "<<dst_pts[i].x<<", "<< dst_pts[i].y<<" ) "<<endl;
        }
#endif
        m_H_ = findHomography(src_pts, dst_pts, cv::RANSAC, 2);
#if 0
        for(int i=0; i< src_pts.size(); i++)
        {
            cv::Mat pt(3, 1, CV_64FC1);
            pt.at<double>(0) = src_pts[i].x;
            pt.at<double>(1) = src_pts[i].y;
            pt.at<double>(2) = 1.0;
            cv::Mat dstPt = H*pt;
            cout<<"src: "<< pt<<"----> dst: "<< dstPt/dstPt.at<double>(2)<<endl;
        }


        cout<<"Homography: "<< H<<endl;

        cv::Mat HT = H.t();

        cout<<"inverse of Homography: "<< HT<<endl;

        QTransform transform(HT.at<double>(0,0),HT.at<double>(0,1),HT.at<double>(0,2),
                             HT.at<double>(1,0),HT.at<double>(1,1),HT.at<double>(1,2),
                             0,0,HT.at<double>(2,2));
        QImage img = m_image_.transformed(transform);
#endif
        m_image_.convertToFormat(QImage::Format_RGB32);

        int height = m_image_.height();
        int width = m_image_.width();

        m_rected_image_ =   QImage(QSize(width, height), QImage::Format_RGB32);


        // a mapping from new image to old images
        for(int i=0; i< height; i++)
        {
            int lineNum = i* width *4;
            for(int j=0; j< width; j++)
            {
                QPoint pt = transformedPoint(QPoint(j, i), m_H_.inv());
                int ii = pt.y();
                int jj = pt.x();

                if(ValidPoint(pt, height, width))
                {

                    m_rected_image_.setPixel(j, i, m_image_.pixel(jj, ii));
                }
                else{

                    m_rected_image_.setPixel(j, i, qRgb(0, 0, 0));
                }
            }
        }


        // same operations for mask images
        m_rected_mask_.clear();
        for(int i=0; i<m_mask_.size(); i++)
        {
            QPoint pt_r = transformedPoint(m_mask_[i], m_H_);
            m_rected_mask_<<pt_r;
        }

        widget_rectified->setImage(m_rected_image_);
        widget_rectified->setRectedMask(m_rected_mask_);

        widget_rectified->setDrawImage(true);
        widget_rectified->setDrawRectedMask(true);

        m_with_rectification_ = true;

        update();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::detect()
{
    clear();

    // 图像无需矫正的情况下
    if(m_with_rectification_ == false)
    {
        m_rected_image_ = m_image_.copy(0,0, m_image_.width(), m_image_.height());
        m_rected_mask_.clear();

        for(int i=0; i< m_mask_.size(); i++)
        {
            m_rected_mask_<<m_mask_[i];
        }
    }

    //m_bg_samples.clear();
    m_bg_samples = widget_rectified->getBGSamples();

    // ************************保证样本再Mask 范围内 *************************//
    QVector<QPoint> new_samples;
    foreach(QPoint pt, m_bg_samples)
    {
        if(m_rected_mask_.containsPoint(pt, Qt::OddEvenFill) )
        {
            new_samples.append(pt);
        }
    }
    m_bg_samples.swap(new_samples);


#if 0  //用于调试
    QString filename("bg_coords.txt");
    QFile  file( filename);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        cout<<"error to read label File!"<<endl;
    }
    QTextStream in(&file);
    int line_num = 0;
    while(!in.atEnd())
    {
        QString line = in.readLine();

        QStringList fields = line.split(" ");
        QPoint pt;

        pt.setY( fields.takeFirst().toInt() -1 );
        pt.setX( fields.takeFirst().toInt() -1 );

        m_bg_samples.append(pt);

        line_num ++;
    }
#endif

    // 获取重复结构的样本
    // m_GTS_pos_.clear();
    m_GTS_pos_= widget_rectified->getGTSPos();
#if 0  // 用于调试
    QRect rect;
    rect.setBottomRight(QPoint(114, 148));
    rect.setTopLeft(QPoint(76, 115));
    m_GTS_pos_.push_back(rect);
#endif

    m_rect_h_ = m_GTS_pos_[0].height();
    m_rect_w_ = m_GTS_pos_[0].width();

    //////////////////initial labels////////////////////////////
    m_initial_labels_.clear();
    m_initial_labels_= getInitialLabels();

    emit dispInitialResultSignal();

    /*****************intialization************************/
    QVector<QVector<double> > P;
    QVector<QVector<double> > Q;
    QVector<QVector<double> > R;

    R.resize(m_rect_h_);
    for(int i=0; i< R.size(); i++)
    {
        for(int j=0; j< m_rect_w_; j++)
        {
            R[i].push_back(1);
        }
    }

    // gts samples
    int start_h = m_GTS_pos_[0].topLeft().y();
    int end_h = m_GTS_pos_[0].bottomRight().y();

    P.resize(m_img_h_);
    for(int i=0; i< P.size(); i++)
    {
        P[i].resize(m_rect_h_);

        for(int j=0; j< m_rect_h_; j++)
        {
            if(i<start_h)
            {
                P[i][j] = 0;
            }
            // identical matrix
            else if(i<= end_h)
            {
                if(j==  i - start_h) P[i][j]=1;
                else{ P[i][j] = 0;}
            }
            else{

                P[i][j]=0;
            }
        }

    }

    int start_w = m_GTS_pos_[0].topLeft().x();
    int end_w = m_GTS_pos_[0].bottomRight().x();

    Q.resize(m_rect_w_);
    for(int i=0; i<Q.size() ; i++)
    {
        Q[i].resize(m_img_w_);
        for(int j=0; j< start_w; j++)
        {
            Q[i][j] = 0;
        }
        for(int j= start_w; j<= end_w; j++)
        {
            if(i == j - start_w)
            {
                Q[i][j] = 1;
            }
            else
            {
                Q[i][j] = 0;
            }
        }

        for(int j= end_w +1; j< m_img_w_; j++)
        {
            Q[i][j] =0;
        }
    }

    /**************** optimization ***************************/
    optimization(P, R, Q);

    QVector<int> row_indices;
    QVector<int> col_indices;

    for(int i= 0; i< P.size(); i++)
    {
        if(P[i][0] == 1)
        {
            row_indices.append(i);
        }
    }

    for(int j=0; j< Q[0].size(); j++)
    {
        if(Q[0][j]==1)
        {
            col_indices.append(j);
        }
    }

    for(int i=0; i< row_indices.size(); i++)
    {
        for(int j=0; j< col_indices.size(); j++)
        {
            QPoint topLeft(col_indices[j], row_indices[i]);
            QPoint topRight(col_indices[j] + m_rect_w_ -1, row_indices[i]);
            QPoint bottomRight(col_indices[j] + m_rect_w_ -1,
                               row_indices[i] + m_rect_h_ -1);
            QPoint bottomLeft(col_indices[j], row_indices[i] + m_rect_h_ -1);


            QPolygon quad;
            quad<<topLeft;
            quad<<topRight;
            quad<<bottomRight;
            quad<<bottomLeft;
            m_detected_gts_rected_.append(quad);

            if(m_with_rectification_ == true)
            {
                //转化都原来的空间
                QPolygon quad_src;
                quad_src<< transformedPoint(topLeft, m_H_.inv())/m_scale_;
                quad_src<< transformedPoint(topRight, m_H_.inv())/m_scale_;
                quad_src<< transformedPoint(bottomRight, m_H_.inv())/m_scale_;
                quad_src<< transformedPoint(bottomLeft, m_H_.inv())/m_scale_;

                m_detected_gts_.append(quad_src);
            }
            if( m_with_rectification_  == false)
            {
                QPolygon quad_src;
                quad_src<< topLeft/m_scale_;
                quad_src<< topRight/m_scale_;
                quad_src<< bottomRight/m_scale_;
                quad_src<< bottomLeft/m_scale_;

                m_detected_gts_.append(quad_src);
            }
        }

    }

    //cout<<"rect num: "<< m_detected_gts_.size()<<endl;

    //foreach(QRect rect, m_detected_gts_)
    //{
    // cout<<"top: "<<rect.topLeft().x()<<", "<< rect.topLeft().y()<<endl;
    // cout<<"bottom: "<< rect.bottomRight().x()<<", "<< rect.bottomRight().y()<<endl;
    //}

    /****************  get final labels ***********************/
    m_final_labels_ = getLabels(P, R, Q);

    emit dispFinalResultSignal();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::dispInitialResult()
{

    QImage img = m_rected_image_.copy(0, 0, m_img_w_, m_img_h_);

    uchar *imgBits = img.bits();

    QColor gts_color = widget_rectified->getFGColor();
    QColor bg_color  = widget_rectified->getBGColor();

    for(int i=0; i< img.height(); i++)
    {
        int lineNum = i* img.width() *4;
        for(int j=0; j< img.width(); j++)
        {
            if((int)m_initial_labels_[i][j] == 1)
            {
                imgBits[lineNum + j*4 + 0] = (uchar)(gts_color.red());
                imgBits[lineNum + j*4 + 1] = (uchar)(gts_color.green());
                imgBits[lineNum + j*4 + 2] = (uchar)(gts_color.blue());
            }
            else if((int)m_initial_labels_[i][j] == 0)
            {
                imgBits[lineNum + j*4 + 0] = (uchar)(bg_color.red());
                imgBits[lineNum + j*4 + 1] = (uchar)(bg_color.green());
                imgBits[lineNum + j*4 + 2] = (uchar)(bg_color.blue());
            }
        }
    }
    label_initialResults->setPixmap(QPixmap::fromImage(img));
    img.save("initial_result.jpg");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::dispFinalResult()
{
    QImage img = m_rected_image_.copy(0, 0, m_img_w_, m_img_h_);

    uchar *imgBits = img.bits();

    QColor gts_color = widget_rectified->getFGColor();
    QColor bg_color  = widget_rectified->getBGColor();

    float alpha = 0.75;
    foreach(QPolygon quad, m_detected_gts_rected_)
    {
        if(quad.size()< 4) continue;

        int start_h = quad[0].y();
        int start_w = quad[0].x();

        int end_h = quad[2].y();
        int end_w = quad[2].x();

        for(int i=start_h; i<= end_h; i++)
        {
            int lineNum = i* img.width() *4;
            for(int j= start_w; j<= end_w; j++)
            {
                imgBits[lineNum + j*4 + 0] = (uchar)( alpha* (float)imgBits[lineNum + j*4 + 0]
                                                      + (1-alpha)* (float)gts_color.red());
                imgBits[lineNum + j*4 + 1] = (uchar)( alpha* (float)imgBits[lineNum + j*4 + 1]
                                                      + (1-alpha)* (float)gts_color.green());
                imgBits[lineNum + j*4 + 2] = (uchar)( alpha* (float)imgBits[lineNum + j*4 + 2]
                                                      + (1-alpha)* (float)gts_color.blue());
            }
        }
    }

    label_output->setPixmap(QPixmap::fromImage(img));
    update();

    img.save("final_result.jpg");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GTSDetectionDialog::imageScale()
{
    m_image_ = m_src_image_.scaled( (QSize(m_scale_* m_src_image_.width(),
                                           m_scale_* m_src_image_.height()) ));
    m_img_h_ = m_image_.height();
    m_img_w_ = m_image_.width();

    label_input->setPixmap(QPixmap::fromImage(m_image_));
    //    label_input->setFixedHeight(m_image_.height());
    //    label_input->setFixedWidth(m_image_.width());


    QPolygon tmp;
    for(int i=0; i< m_src_mask_.size(); i++)
    {
        tmp<<m_scale_*m_src_mask_[i];
    }
    m_mask_.swap(tmp);
    widget_rectified->setMask(m_mask_);

    widget_rectified->setImage(m_image_);
    widget_rectified->setDrawImage(true);


    // 调整显示的区域的大小
    //    widget_rectified->setFixedHeight(m_image_.height());
    //    widget_rectified->setFixedWidth(m_image_.width());

    //    label_initialResults->setFixedHeight(m_image_.height());
    //    label_initialResults->setFixedWidth(m_image_.width());

    //    label_output->setFixedHeight(m_image_.height());
    //    label_output->setFixedWidth(m_image_.width());

    update();
}
