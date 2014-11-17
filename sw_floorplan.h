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

////#include"SW_PointCloud.h"
////#include"codingEdit.h"
//#include"gts_repetitiveStructure.h"

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
    QVector<QVector<uint> > f_slice_pt_ids_;
    QVector<float> f_Y_coords_;

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
    /*
     * reconstruct buiding layer by layer
     */
    void reconstruction();

signals:
    void enableModelingCeiling();

private:

    //shared point cloud pointer
    FloorPlanDialog* m_floor_plan_;


    int p_starting_layer_;
    int p_ending_layer_;

    double p_RG_line_width_;
    double p_linking_curves_margin_;
    double p_MRF_lambda_;
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

    FloorPlanDialog(QWidget * parent, QVector<Point> * points, QVector<uint> * pt_ids);
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

#if 0
    inline void setFacetsPtr(QVector<QVector<int> > * facets)
    {
        p_facets_ = facets;
    }
    inline void setVerticesPtr(QVector<Vec3> * vertices)
    {
        p_vertices_ = vertices;
    }
    inline void setEdgesPtr(QVector<QPair<int, int> >*edges)
    {
        p_edges_ = edges;
    }
    inline void setBlocksPtr(QVector<Block3> * blocks)
    {
        p_blocks_ = blocks;
    }
#endif

    inline void setStartingLayerPtr(QVector<Vec3>* starting_layer){p_starting_layer_boundary_ = starting_layer;}
    inline void setEndingLayerPtr(QVector<Vec3>* ending_layer){p_ending_layer_boundary_ = ending_layer;}

#if 0
    inline void setSemiPlanesPtr(QVector<QVector<Vec3> > * semi_planes){ p_semi_planes_ = semi_planes;}
#endif

    // QVector<QPolygon > getGTS(){ return p_detected_gts_; }

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

    // set starting layer number
    inline void starttingLayerChanged(int v)
    {
        p_floorplan_constructor_->setStartingLayer(v);
    }

    // set ending layer number
    inline void endingLayerChanged(int v)
    {
        p_floorplan_constructor_->setEndingLayer(v);
    }

    // set line width
    inline void lineWidthChanged(double v)
    {
        p_floorplan_constructor_->setRGLineWidth(v);
    }

    // set curve margin
    inline void curveMarginChanged(double v)
    {
        p_floorplan_constructor_->setLinkingCurvesMargin(v);
    }


    //set MRF Lambda
    inline void MRFLambdaChanged(double v)
    {
        p_floorplan_constructor_->setMRFLambda(v);
    }


    inline void LetterMarginChanged(double v)
    {
        p_floorplan_constructor_->setLetterMargin(v);
    }


 #if 0
    // begin GTS detection
    inline void startGTSDetection(){

        p_gts_detection_ = new GTSDetectionDialog (this);

        connect(p_gts_detection_->pushButton_accept, SIGNAL(clicked()), this, SLOT(collectGTS()));
        p_gts_detection_->setImage(p_current_image_);
        p_gts_detection_->setMask(p_current_mask_);
        p_gts_detection_->show();
    }


    inline void setImage( const QImage &image)
    {
        p_current_image_ = image.copy(0, 0, image.width(), image.height());
    }

    inline void setMask(const QPolygon & mask)
    {
        p_current_mask_.clear();
        for(int i=0; i< mask.size(); i++)
        {
            p_current_mask_<< mask[i];
        }
        //        p_gts_detection_->widget_rectified->setMask(mask);
        update();
    }


    inline void collectGTS()
    {
        QVector<QPolygon> gts = p_gts_detection_->getGTS();

        p_detected_gts_.clear();
        foreach(QPolygon polygon, gts)
        {
            p_detected_gts_.append(polygon);
        }
    }
#endif

signals:

    //// void enableImageGuideModeling();

    void startToDetect(bool, bool);

    //// void start_gts_detection();

private:

    InconsistenRegionDetector * p_inconsistent_detector_;
    SlicesDataCaculator * p_slices_acculator_;
    MainDirectionExtractor * p_main_directions_extractor_;
    FloorPlanReconstructor* p_floorplan_constructor_;

    //// GTSDetectionDialog * p_gts_detection_;
    //// QVector<QPolygon> p_detected_gts_;

    //// QImage p_current_image_;
    //// QPolygon p_current_mask_;  // outer polygon of a plane

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

    float p_step_;

    // main directions
    QVector<Vec3> p_main_directions_;

    QVector<QVector<uint> > *p_facets_;
    QVector<Vec3> * p_vertices_;
    QVector<QPair<uint, uint> > *p_edges_;
    //// QVector<Block3> *p_blocks_;
    QVector<QVector<Vec3> > *p_semi_planes_;

    QVector<Vec3>* p_starting_layer_boundary_;
    QVector<Vec3>* p_ending_layer_boundary_;


    QThread p_thread_;

public:

    FloorPlanDisplay p_floorplan_displays_;

private:

    QVector<Point> * p_points_;
    QVector<uint> * p_pt_ids_;
};
}


#endif // SW_FLOORPLAN_H
