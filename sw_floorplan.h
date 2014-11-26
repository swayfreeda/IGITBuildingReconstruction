// Description: Classes related to the FloorPlan reconstructor are defined
//              *InconsistenRegionDetector
//              *SlicesDataCaculator
//              *MainDirectionExtractor
//              *FloorPlanReconstructor
//              *FloorPlanDialog

// Time: 11/09/2014
// Author: Sway
// Organization: Institute of Automation, Chinese Academy of Sciences


#ifndef SW_FLOORPLAN_H
#define SW_FLOORPLAN_H

#include"ui_FloorPlan.h"
#include "sw_dataType.h"
#include"sw_codingEdit.h"

#include"gts_repetitiveStructure.h"

#include <QApplication>
#include<QDialog>
#include <QWidget>

#include"QGLViewer/qglviewer.h"
#include<QVector2D>
#include<QMessageBox>
#include<QSet>
#include<QProgressBar>
#include<QApplication>
#include<QThread>
#include<QListWidgetItem>

typedef PointXYZRGBNormal Point;

namespace SW {
class FloorPlanDialog;

class  GLViewer;
//----------------------------------------------CLASS FloorPlanDisplay-----------------------------------------//
//this class containes all the variables for display the process of the functions
class FloorPlanDisplay
{

    friend class QLViewer;
public:
    FloorPlanDisplay(){}

public:

    // indices of points in slice
    QVector<QVector<uint> > f_slice_pt_ids_;

    // ycoordinates of each slice
    QVector<float> f_Y_coords_;

    // semiplanes of slice data
    QVector<QVector<Vec3> > f_semi_planes_;

    // starting layer
    QVector<Vec3> f_starting_layer_boundary_;

    // endding layer
    QVector<Vec3> f_ending_layer_boundary_;

};

////////////////////////////CLASS InconsistentRegionDetector/////////////////////////////////////////////////
class InconsistenRegionDetector: public QObject
{
    Q_OBJECT

public:
    // constructor
    InconsistenRegionDetector(){}
    InconsistenRegionDetector(FloorPlanDialog * pf): p_floor_plan_(pf)
    {
    }

    // compute the curvatures of each point in the point cloud
    void computeCurvatures();

    // compute the inconsistent region in the point cloud
    void getInconsistentRegion();

public slots:
    void detect( bool is_knn_changed, bool is_thresh_changed);

signals:

    void enableGettingSlices();

    void updateGLViewer();
private:

    // get Knn value and threshold
    FloorPlanDialog* p_floor_plan_;

    QVector<float>  p_curvatures_;

    /*********Parameters for Inconsistent
                       Region Detection *********************/
    //number of nearest neighbours
    int p_Knn_;

    //the threhold to distinguish inconsistent region
    double p_threshold_;

};


////////////////////////////CLASS SlicesDataCaculator////////////////////////////////////////////////////////
class SlicesDataCaculator: public QObject{

    Q_OBJECT
public:
    SlicesDataCaculator(){}
    SlicesDataCaculator( FloorPlanDialog * pf): p_floor_plan_(pf){}


public slots:
    // divide the scene to the slices data
    void divideToSlices();

signals:
    void enableFloorPlanReconstruction();
    void updateGLViewer();

private:
    FloorPlanDialog* p_floor_plan_;
};


////////////////////////////CLASS MainDirectionExtractor//////////////////////////////////////////////////////////////////////////////////
class MainDirectionExtractor:public QObject{
    Q_OBJECT

public:
    MainDirectionExtractor(){}
    MainDirectionExtractor(FloorPlanDialog * pf): m_floor_plan_(pf) {}

protected slots:

    void computeMainDirections();

signals:
    void enableFloorPlanReconstruction();

private:

    FloorPlanDialog* m_floor_plan_;
    float p_mean_dist_;

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class FloorPlanReconstructor: public QObject{
    Q_OBJECT

public:
    FloorPlanReconstructor(){}
    FloorPlanReconstructor(FloorPlanDialog * pf): m_floor_plan_(pf)
    {
    }

    inline void setStartingLayer(int v){p_starting_layer_ = v;}
    inline int getStartingLayer(){return p_starting_layer_;}

    inline void setEndingLayer(int v){ p_ending_layer_ = v;}
    inline int getEndingLayer(){return p_ending_layer_;}

    inline void setRGLineWidth(double v){p_RG_line_width_ = v;}
    inline double getRGLineWidth(){return p_RG_line_width_;}

    inline void setLinkingCurvesMargin(double v){p_linking_curves_margin_ = v;}
    inline double getLinkingCurvesMargin(){return p_linking_curves_margin_;}

    inline void setMRFLambda(double v){ p_MRF_lambda_ = v;}
    inline double getMRFLambda(){return p_MRF_lambda_;}

    inline void setLetterMargin(double v){ p_Letter_margin_ = v;}
    inline double getLetterMargin(){return p_Letter_margin_;}


protected slots:

    // set starting layer number
    inline void starttingLayerChanged(int v)
    {
        setStartingLayer(v);
    }

    // set ending layer number
    inline void endingLayerChanged(int v)
    {
        setEndingLayer(v);
    }

    // set line width
    inline void lineWidthChanged(double v)
    {
        setRGLineWidth(v);
    }

    // set curve margin
    inline void curveMarginChanged(double v)
    {
        setLinkingCurvesMargin(v);
    }

    //set MRF Lambda
    inline void MRFLambdaChanged(double v)
    {
        setMRFLambda(v);
    }

    // set Letter Margin Changed
    inline void LetterMarginChanged(double v)
    {
        setLetterMargin(v);
    }


    /*
     * reconstruct buiding layer by layer
     */
    void reconstruction();

signals:
    void enableModelingCeiling();

    void updateGLViewer();

private:

    //shared point cloud pointer
    FloorPlanDialog* m_floor_plan_;


    // the indice of the starting layer
    int p_starting_layer_;

    // the indice of the ending layer
    int p_ending_layer_;

    // the max width of the line for region growing
    double p_RG_line_width_;

    // the max margin between two curves
    double p_linking_curves_margin_;

    // the lambda for MRF Optimization
    double p_MRF_lambda_;

    // the margin for Letter
    double p_Letter_margin_;
};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class FloorPlanDialog: public QDialog, public Ui::FloorPlanDialog
{
    Q_OBJECT

public:

    //// friend class can visit the protected and private members
    friend class InconsistenRegionDetector;
    friend class SlicesDataCaculator;
    friend class MainDirectionExtractor;
    friend class FloorPlanReconstructor;

    FloorPlanDialog(QWidget * parent, PointCloud * pc, Mesh * mesh,
                    QMap<QString, Plane3D> * planes,
                    QMap<QString, cv::Mat_<cv::Vec3b> > *images, QMap<QString, Camera> * cameras);
    FloorPlanDialog(){}
    ~FloorPlanDialog(){}

    //--------------------------------------------------INTERFACES------------------------------------------------------------------//
    inline void setKnnChanged(bool flag){
        is_knn_changed_ = flag;
    }
    inline void setThreshChanged(bool flag){
        is_thresh_changed_ = flag;
    }
    inline bool isKnnChanged(){return is_knn_changed_;}
    inline bool isThreshChanged(){return is_thresh_changed_;}
    inline void setIsGettingSlicesFinished(bool flag){ is_gettingSlices_finished_ = flag;}
    inline bool isGettingSlicesFinished(){return is_gettingSlices_finished_;}
    inline void setIsComputeMainDirectsFinsihed(bool flag){ is_computeMainDirects_finished_ = flag;}
    inline bool isComputeMainDirectsFinished(){return is_computeMainDirects_finished_;}

    QVector<QVector<uint> > getSlicePtsIds(){return p_slice_pts_;}
    QVector<float> getYcoords(){return p_ycoordinates_; }

    InconsistenRegionDetector *getIncinsistDetector(){return  p_inconsistent_detector_;}
    SlicesDataCaculator * getSlicesCalculator(){return  p_slices_acculator_; }
    FloorPlanReconstructor * getFloorPlanReconstructor(){ return p_floorplan_constructor_;}

    inline void setCurrentPlane3DPtr(Plane3D * ptr){p_current_plane3D_ptr_ = ptr;}

    QVector<QPolygon > getGTS(){ return p_detected_gts_; }


    // void compute GTS
    void computeMaskForGTS();


private slots:

    // Knn value changed
    inline void KnnChanged()
    {
        setKnnChanged(true);
        update();
    }

    // threshold changed
    inline void ThreshChanged()
    {
        setThreshChanged(true);
        update();
    }

    // emit signal to start detecting  inconsistent region
    inline void emitDetectionSignal()
    {
        emit  startToDetect(isKnnChanged(), isThreshChanged());
    }

    // set the button to be abled
    inline void enableButton()
    {
        pushButton_insistRegionDetection->setEnabled(true);
    }

    // enable Group Box Getting Slices
    inline void enableGroupBoxGettingSlices()
    {
        GroupBox_gettingSlices->setEnabled(true);
        Label_setStepValue->setEnabled(true);
        doubleSpinBox_stepValue->setEnabled(true);
        pushButton_divide_slices->setEnabled(true);
        checkBox_dispSices->setEnabled(true);

        label_setLayerNumber->setEnabled(true);
        label_displayLayerNumber->setEnabled(true);
        label_mainDirecionNum->setEnabled(true);
        label_dispMainDirectionNumbers->setEnabled(true);
        pushButton_mainDireciongs->setEnabled(true);

        doubleSpinBox_stepValue->setRange(0, 2.0);
        doubleSpinBox_stepValue->setSingleStep(0.05);
        doubleSpinBox_stepValue->setValue(0.1);

        doubleSpinBox_minAngle->setRange(0, 90);
        doubleSpinBox_minAngle->setSingleStep(1);
        doubleSpinBox_minAngle->setValue(25.0);
    }

    // enable Group Box FloorPlanReconstruction
    inline void enableGroupBoxFloorPlanReconstrucion()
    {
        GroupBox_gettingFloorPlan->setEnabled(true);

        horizontalSlider_Slayer->setRange(0, p_slice_pts_.size()-1);
        horizontalSlider_Slayer->setLineStep(1);
        horizontalSlider_Slayer->setValue(2);

        horizontalSlider_Elayer->setRange(0, p_slice_pts_.size()-1);
        horizontalSlider_Elayer->setLineStep(1);
        horizontalSlider_Elayer->setValue(p_slice_pts_.size()-1);

        doubleSpinBox_lineWidth->setRange(0.01, 1.0);
        doubleSpinBox_lineWidth->setSingleStep(0.02);
        doubleSpinBox_lineWidth->setValue(0.1);

        doubleSpinBox_curveMargin->setRange(0, 100.0);
        doubleSpinBox_curveMargin->setSingleStep(0.1);
        doubleSpinBox_curveMargin->setValue(0.5);

        doubleSpinBox_MRFLambda->setRange(0, 50);
        doubleSpinBox_MRFLambda->setSingleStep(0.1);
        doubleSpinBox_MRFLambda->setValue(0.5);
    }

    // enable Group Box Modeling Ceiling
    inline void enableGroupBoxModelingCeiling()
    {
        GroupBox_modelingCeiling->setEnabled(true);
    }


    ////////////////////////////////////CreatePlanesFromTriangulations////////////////////////////////////////////////
    // perform a region growing algorithms on the triangulations, triangulations belonging to the same
    // plane are clustered into the same plane, afterwards, a plane is created, and corresponding
    // triangulations are added.
    void createPlanesFromTriangulations();



    /////////////////////////////////////////setCurrentImage///////////////////////////////////////////////////////////
    inline void setCurrentImage( QListWidgetItem* item)
    {
        cv::Mat_<cv::Vec3b> img = (*p_images_)[item->text()];
        p_current_image_ = convertToQImage(img);

        p_current_camera_ = (*p_cameras_)[item->text()];

        update();
    }



    //////////////////////////////////////// begin GTS detection//////////////////////////////////////////////////////
    inline void startGTSDetection(){

        p_gts_detection_ = new GTSDetectionDialog (this);

        connect(p_gts_detection_->pushButton_accept, SIGNAL(clicked()), this, SLOT(collectGTS()));

        // compute the mask for GTS
        computeMaskForGTS();

        // set current image
        p_gts_detection_->setImage(p_current_image_);

        // set current mask
        p_gts_detection_->setMask(p_current_mask_);


        p_gts_detection_->show();
    }



    //////////////////////////////////////// collect GTS /////////////////////////////////////////////////////////////
    inline void collectGTS()
    {
        QVector<QPolygon> gts = p_gts_detection_->getGTS();

        p_detected_gts_.clear();
        foreach(QPolygon polygon, gts)
        {
            p_detected_gts_.append(polygon);
        }
    }



    /////////////////////////////////////////////backprojection///////////////////////////////////////////////////////
    // back projetion to form the windows
    void backProjection();


    //////////////////////////////////////////////add window planes///////////////////////////////////////////////////
    // add window planes
    void addWindowPlanes();


    //////////////////////////////////////////////accept added window planes//////////////////////////////////////////
    void acceptAddedWindowPlanes();



    //////////////////////////////////////////////abort added window planes///////////////////////////////////////////
    void abortAddedWindowPlanes();


    ////////////////////////////////////////////// update all mesh ////////////////////////////////////////////////////
    void updateMeshAll();

signals:

    //// void enableImageGuideModeling();

    void startToDetect(bool, bool);

    void createNewPlane(QString name);

    void start_gts_detection();

    void startDisplayBackProjQuads();

    void endDisplayBackProjQuads();

    void startDisplayAddedWindowPlanes();

    void endDisplayAddedWindowPlanes();

    void updateGLViewer();

    void startDisplayModellingResults(int state);

    void endDisplaySinglePlaneTrians(bool flag);

private:

    InconsistenRegionDetector * p_inconsistent_detector_;
    SlicesDataCaculator * p_slices_acculator_;
    MainDirectionExtractor * p_main_directions_extractor_;
    FloorPlanReconstructor* p_floorplan_constructor_;
    GTSDetectionDialog * p_gts_detection_;


    //-----------------------------------------GTS detection----------------------------------------//
    QVector<QPolygon> p_detected_gts_;

    // current image
    QImage p_current_image_;

    // current mask
    QPolygon p_current_mask_;  // outer polygon of a plane

    // current camera
    Camera p_current_camera_;



    //-----------------------------------------inconsistent region related------------------------//
    bool is_knn_changed_;
    bool is_thresh_changed_;

    bool is_gettingSlices_finished_;
    bool is_computeMainDirects_finished_;

    //-----------------------------------------slices ralated-----------------------------------//
    // indices of slices points
    QVector<QVector<uint> >p_slice_pts_;

    // y coordinates of each silces
    QVector<float> p_ycoordinates_;

    // distance between neighboring slices
    float p_step_;

    // main directions
    QVector<Vec3> p_main_directions_;

    // blocks in the scene
    QVector<SW::Block3> p_blocks_;

    // for multiple thread
    QThread p_thread_;

    // pointer of current plane
    Plane3D * p_current_plane3D_ptr_;


    // width of the plane
    float p_window_depth_;

public:

    // contain variables to be displayed
    FloorPlanDisplay p_floorplan_displays_;

private:

    // point cloud of the scene
    PointCloud * p_pc_;

    // mesh containes vertices, facets and edges
    Mesh *p_mesh_;

    // shared pointer of images
    QMap<QString, cv::Mat_<cv::Vec3b> > *p_images_;

    // shared pointer of cameras
    QMap<QString, Camera> *p_cameras_;

    // planes of the scene
    QMap<QString, Plane3D> * p_plane3Ds_;
};
}


#endif // SW_FLOORPLAN_H
