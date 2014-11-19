/********************************************************************************
** Form generated from reading UI file 'FloorPlan.ui'
**
** Created: Wed Nov 19 16:23:15 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FLOORPLAN_H
#define UI_FLOORPLAN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_FloorPlanDialog
{
public:
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_KNN;
    QSpinBox *spinBox_knn;
    QLabel *label_Threshold;
    QDoubleSpinBox *doubleSpinBox_threshold;
    QCheckBox *checkBox_dispInconsist;
    QSpacerItem *horizontalSpacer_6;
    QPushButton *pushButton_insistRegionDetection;
    QGroupBox *GroupBox_gettingSlices;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_5;
    QGridLayout *gridLayout_2;
    QDoubleSpinBox *doubleSpinBox_stepValue;
    QLabel *Label_setStepValue;
    QLabel *label_setLayerNumber;
    QLabel *label_displayLayerNumber;
    QCheckBox *checkBox_dispSices;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_divide_slices;
    QHBoxLayout *horizontalLayout;
    QLabel *label_mainDirecionNum;
    QLabel *label_dispMainDirectionNumbers;
    QLabel *label_minmum_angle;
    QDoubleSpinBox *doubleSpinBox_minAngle;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *pushButton_mainDireciongs;
    QGroupBox *GroupBox_gettingFloorPlan;
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout;
    QLabel *label_setEndingLayer;
    QSlider *horizontalSlider_Slayer;
    QLabel *label_setStartingLayer;
    QSlider *horizontalSlider_Elayer;
    QHBoxLayout *horizontalLayout_4;
    QLabel *Label_setLineWidth;
    QDoubleSpinBox *doubleSpinBox_lineWidth;
    QLabel *Label_setStepValue_3;
    QDoubleSpinBox *doubleSpinBox_curveMargin;
    QLabel *Label_setMRFValue;
    QDoubleSpinBox *doubleSpinBox_MRFLambda;
    QLabel *label;
    QDoubleSpinBox *doubleSpinBox_letterMargin;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *checkBox_dispProcess;
    QCheckBox *checkBox_dispModel;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *pushButton_floorplanRec;
    QPushButton *pushButton;
    QGroupBox *GroupBox_modelingCeiling;
    QHBoxLayout *horizontalLayout_16;
    QVBoxLayout *verticalLayout_4;
    QCheckBox *checkBox_edit_mode;
    QPushButton *pushButton_addCeilingPlane;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *pushButton_acceptFloorPlygon;
    QPushButton *pushButton_cancelFloorPolygon;
    QSpacerItem *horizontalSpacer_3;
    QVBoxLayout *verticalLayout_6;
    QCheckBox *checkBox_adjustment_mode;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *pushButton_selecPointsOnEditPlane;
    QPushButton *pushButton_bundleVertices;
    QHBoxLayout *horizontalLayout_10;
    QPushButton *pushButton_accetpAdjusment;
    QPushButton *pushButton_cancelAdjustment;
    QSpacerItem *horizontalSpacer_7;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *pushButton_birdView;
    QPushButton *pushButton_lateralView;
    QPushButton *pushButton_DTsOnEditPlane;
    QGroupBox *GroupBox_IGM;
    QHBoxLayout *horizontalLayout_13;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton_PolygonPlanes;
    QPushButton *pushButton_beginGTSDetection_;
    QPushButton *pushButton_backprojection;
    QSpacerItem *horizontalSpacer_8;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_11;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_2;
    QDoubleSpinBox *doubleSpinBox_windowDepth;
    QPushButton *pushButton_addWindowPlanes;
    QHBoxLayout *horizontalLayout_12;
    QPushButton *pushButton_accpectWindowPlanes;
    QPushButton *pushButton_cacelWindowPlanes;

    void setupUi(QDialog *FloorPlanDialog)
    {
        if (FloorPlanDialog->objectName().isEmpty())
            FloorPlanDialog->setObjectName(QString::fromUtf8("FloorPlanDialog"));
        FloorPlanDialog->resize(821, 644);
        verticalLayout_3 = new QVBoxLayout(FloorPlanDialog);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        groupBox = new QGroupBox(FloorPlanDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        horizontalLayout_8 = new QHBoxLayout(groupBox);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_KNN = new QLabel(groupBox);
        label_KNN->setObjectName(QString::fromUtf8("label_KNN"));

        horizontalLayout_7->addWidget(label_KNN);

        spinBox_knn = new QSpinBox(groupBox);
        spinBox_knn->setObjectName(QString::fromUtf8("spinBox_knn"));

        horizontalLayout_7->addWidget(spinBox_knn);

        label_Threshold = new QLabel(groupBox);
        label_Threshold->setObjectName(QString::fromUtf8("label_Threshold"));

        horizontalLayout_7->addWidget(label_Threshold);

        doubleSpinBox_threshold = new QDoubleSpinBox(groupBox);
        doubleSpinBox_threshold->setObjectName(QString::fromUtf8("doubleSpinBox_threshold"));

        horizontalLayout_7->addWidget(doubleSpinBox_threshold);

        checkBox_dispInconsist = new QCheckBox(groupBox);
        checkBox_dispInconsist->setObjectName(QString::fromUtf8("checkBox_dispInconsist"));

        horizontalLayout_7->addWidget(checkBox_dispInconsist);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_6);

        pushButton_insistRegionDetection = new QPushButton(groupBox);
        pushButton_insistRegionDetection->setObjectName(QString::fromUtf8("pushButton_insistRegionDetection"));

        horizontalLayout_7->addWidget(pushButton_insistRegionDetection);


        horizontalLayout_8->addLayout(horizontalLayout_7);


        verticalLayout_3->addWidget(groupBox);

        GroupBox_gettingSlices = new QGroupBox(FloorPlanDialog);
        GroupBox_gettingSlices->setObjectName(QString::fromUtf8("GroupBox_gettingSlices"));
        GroupBox_gettingSlices->setEnabled(false);
        verticalLayout = new QVBoxLayout(GroupBox_gettingSlices);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        doubleSpinBox_stepValue = new QDoubleSpinBox(GroupBox_gettingSlices);
        doubleSpinBox_stepValue->setObjectName(QString::fromUtf8("doubleSpinBox_stepValue"));
        doubleSpinBox_stepValue->setEnabled(false);

        gridLayout_2->addWidget(doubleSpinBox_stepValue, 0, 1, 1, 1);

        Label_setStepValue = new QLabel(GroupBox_gettingSlices);
        Label_setStepValue->setObjectName(QString::fromUtf8("Label_setStepValue"));
        Label_setStepValue->setEnabled(false);

        gridLayout_2->addWidget(Label_setStepValue, 0, 0, 1, 1);

        label_setLayerNumber = new QLabel(GroupBox_gettingSlices);
        label_setLayerNumber->setObjectName(QString::fromUtf8("label_setLayerNumber"));

        gridLayout_2->addWidget(label_setLayerNumber, 1, 0, 1, 1);

        label_displayLayerNumber = new QLabel(GroupBox_gettingSlices);
        label_displayLayerNumber->setObjectName(QString::fromUtf8("label_displayLayerNumber"));
        label_displayLayerNumber->setEnabled(false);
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        label_displayLayerNumber->setFont(font);
        label_displayLayerNumber->setAutoFillBackground(true);

        gridLayout_2->addWidget(label_displayLayerNumber, 1, 1, 1, 1);


        horizontalLayout_5->addLayout(gridLayout_2);

        checkBox_dispSices = new QCheckBox(GroupBox_gettingSlices);
        checkBox_dispSices->setObjectName(QString::fromUtf8("checkBox_dispSices"));

        horizontalLayout_5->addWidget(checkBox_dispSices);

        horizontalSpacer = new QSpacerItem(331, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);

        pushButton_divide_slices = new QPushButton(GroupBox_gettingSlices);
        pushButton_divide_slices->setObjectName(QString::fromUtf8("pushButton_divide_slices"));
        pushButton_divide_slices->setEnabled(false);

        horizontalLayout_5->addWidget(pushButton_divide_slices);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_mainDirecionNum = new QLabel(GroupBox_gettingSlices);
        label_mainDirecionNum->setObjectName(QString::fromUtf8("label_mainDirecionNum"));
        label_mainDirecionNum->setEnabled(false);

        horizontalLayout->addWidget(label_mainDirecionNum);

        label_dispMainDirectionNumbers = new QLabel(GroupBox_gettingSlices);
        label_dispMainDirectionNumbers->setObjectName(QString::fromUtf8("label_dispMainDirectionNumbers"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_dispMainDirectionNumbers->sizePolicy().hasHeightForWidth());
        label_dispMainDirectionNumbers->setSizePolicy(sizePolicy);
        label_dispMainDirectionNumbers->setFont(font);
        label_dispMainDirectionNumbers->setAutoFillBackground(true);
        label_dispMainDirectionNumbers->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(label_dispMainDirectionNumbers);

        label_minmum_angle = new QLabel(GroupBox_gettingSlices);
        label_minmum_angle->setObjectName(QString::fromUtf8("label_minmum_angle"));

        horizontalLayout->addWidget(label_minmum_angle);

        doubleSpinBox_minAngle = new QDoubleSpinBox(GroupBox_gettingSlices);
        doubleSpinBox_minAngle->setObjectName(QString::fromUtf8("doubleSpinBox_minAngle"));

        horizontalLayout->addWidget(doubleSpinBox_minAngle);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_5);

        pushButton_mainDireciongs = new QPushButton(GroupBox_gettingSlices);
        pushButton_mainDireciongs->setObjectName(QString::fromUtf8("pushButton_mainDireciongs"));
        pushButton_mainDireciongs->setEnabled(false);

        horizontalLayout->addWidget(pushButton_mainDireciongs);


        verticalLayout->addLayout(horizontalLayout);


        verticalLayout_3->addWidget(GroupBox_gettingSlices);

        GroupBox_gettingFloorPlan = new QGroupBox(FloorPlanDialog);
        GroupBox_gettingFloorPlan->setObjectName(QString::fromUtf8("GroupBox_gettingFloorPlan"));
        GroupBox_gettingFloorPlan->setEnabled(false);
        GroupBox_gettingFloorPlan->setCheckable(false);
        GroupBox_gettingFloorPlan->setChecked(false);
        verticalLayout_2 = new QVBoxLayout(GroupBox_gettingFloorPlan);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_setEndingLayer = new QLabel(GroupBox_gettingFloorPlan);
        label_setEndingLayer->setObjectName(QString::fromUtf8("label_setEndingLayer"));

        gridLayout->addWidget(label_setEndingLayer, 1, 0, 1, 1);

        horizontalSlider_Slayer = new QSlider(GroupBox_gettingFloorPlan);
        horizontalSlider_Slayer->setObjectName(QString::fromUtf8("horizontalSlider_Slayer"));
        horizontalSlider_Slayer->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_Slayer, 0, 1, 1, 1);

        label_setStartingLayer = new QLabel(GroupBox_gettingFloorPlan);
        label_setStartingLayer->setObjectName(QString::fromUtf8("label_setStartingLayer"));

        gridLayout->addWidget(label_setStartingLayer, 0, 0, 1, 1);

        horizontalSlider_Elayer = new QSlider(GroupBox_gettingFloorPlan);
        horizontalSlider_Elayer->setObjectName(QString::fromUtf8("horizontalSlider_Elayer"));
        horizontalSlider_Elayer->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_Elayer, 1, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        Label_setLineWidth = new QLabel(GroupBox_gettingFloorPlan);
        Label_setLineWidth->setObjectName(QString::fromUtf8("Label_setLineWidth"));

        horizontalLayout_4->addWidget(Label_setLineWidth);

        doubleSpinBox_lineWidth = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_lineWidth->setObjectName(QString::fromUtf8("doubleSpinBox_lineWidth"));

        horizontalLayout_4->addWidget(doubleSpinBox_lineWidth);

        Label_setStepValue_3 = new QLabel(GroupBox_gettingFloorPlan);
        Label_setStepValue_3->setObjectName(QString::fromUtf8("Label_setStepValue_3"));

        horizontalLayout_4->addWidget(Label_setStepValue_3);

        doubleSpinBox_curveMargin = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_curveMargin->setObjectName(QString::fromUtf8("doubleSpinBox_curveMargin"));

        horizontalLayout_4->addWidget(doubleSpinBox_curveMargin);

        Label_setMRFValue = new QLabel(GroupBox_gettingFloorPlan);
        Label_setMRFValue->setObjectName(QString::fromUtf8("Label_setMRFValue"));

        horizontalLayout_4->addWidget(Label_setMRFValue);

        doubleSpinBox_MRFLambda = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_MRFLambda->setObjectName(QString::fromUtf8("doubleSpinBox_MRFLambda"));

        horizontalLayout_4->addWidget(doubleSpinBox_MRFLambda);

        label = new QLabel(GroupBox_gettingFloorPlan);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_4->addWidget(label);

        doubleSpinBox_letterMargin = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_letterMargin->setObjectName(QString::fromUtf8("doubleSpinBox_letterMargin"));

        horizontalLayout_4->addWidget(doubleSpinBox_letterMargin);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_4);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        checkBox_dispProcess = new QCheckBox(GroupBox_gettingFloorPlan);
        checkBox_dispProcess->setObjectName(QString::fromUtf8("checkBox_dispProcess"));

        horizontalLayout_2->addWidget(checkBox_dispProcess);

        checkBox_dispModel = new QCheckBox(GroupBox_gettingFloorPlan);
        checkBox_dispModel->setObjectName(QString::fromUtf8("checkBox_dispModel"));

        horizontalLayout_2->addWidget(checkBox_dispModel);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        pushButton_floorplanRec = new QPushButton(GroupBox_gettingFloorPlan);
        pushButton_floorplanRec->setObjectName(QString::fromUtf8("pushButton_floorplanRec"));

        horizontalLayout_2->addWidget(pushButton_floorplanRec);

        pushButton = new QPushButton(GroupBox_gettingFloorPlan);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_2->addWidget(pushButton);


        verticalLayout_2->addLayout(horizontalLayout_2);


        verticalLayout_3->addWidget(GroupBox_gettingFloorPlan);

        GroupBox_modelingCeiling = new QGroupBox(FloorPlanDialog);
        GroupBox_modelingCeiling->setObjectName(QString::fromUtf8("GroupBox_modelingCeiling"));
        GroupBox_modelingCeiling->setEnabled(false);
        GroupBox_modelingCeiling->setFlat(false);
        GroupBox_modelingCeiling->setCheckable(false);
        GroupBox_modelingCeiling->setChecked(false);
        horizontalLayout_16 = new QHBoxLayout(GroupBox_modelingCeiling);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        checkBox_edit_mode = new QCheckBox(GroupBox_modelingCeiling);
        checkBox_edit_mode->setObjectName(QString::fromUtf8("checkBox_edit_mode"));

        verticalLayout_4->addWidget(checkBox_edit_mode);

        pushButton_addCeilingPlane = new QPushButton(GroupBox_modelingCeiling);
        pushButton_addCeilingPlane->setObjectName(QString::fromUtf8("pushButton_addCeilingPlane"));

        verticalLayout_4->addWidget(pushButton_addCeilingPlane);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        pushButton_acceptFloorPlygon = new QPushButton(GroupBox_modelingCeiling);
        pushButton_acceptFloorPlygon->setObjectName(QString::fromUtf8("pushButton_acceptFloorPlygon"));

        horizontalLayout_9->addWidget(pushButton_acceptFloorPlygon);

        pushButton_cancelFloorPolygon = new QPushButton(GroupBox_modelingCeiling);
        pushButton_cancelFloorPolygon->setObjectName(QString::fromUtf8("pushButton_cancelFloorPolygon"));

        horizontalLayout_9->addWidget(pushButton_cancelFloorPolygon);


        verticalLayout_4->addLayout(horizontalLayout_9);


        horizontalLayout_16->addLayout(verticalLayout_4);

        horizontalSpacer_3 = new QSpacerItem(71, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_16->addItem(horizontalSpacer_3);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        checkBox_adjustment_mode = new QCheckBox(GroupBox_modelingCeiling);
        checkBox_adjustment_mode->setObjectName(QString::fromUtf8("checkBox_adjustment_mode"));

        verticalLayout_6->addWidget(checkBox_adjustment_mode);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        pushButton_selecPointsOnEditPlane = new QPushButton(GroupBox_modelingCeiling);
        pushButton_selecPointsOnEditPlane->setObjectName(QString::fromUtf8("pushButton_selecPointsOnEditPlane"));

        horizontalLayout_15->addWidget(pushButton_selecPointsOnEditPlane);

        pushButton_bundleVertices = new QPushButton(GroupBox_modelingCeiling);
        pushButton_bundleVertices->setObjectName(QString::fromUtf8("pushButton_bundleVertices"));

        horizontalLayout_15->addWidget(pushButton_bundleVertices);


        verticalLayout_6->addLayout(horizontalLayout_15);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        pushButton_accetpAdjusment = new QPushButton(GroupBox_modelingCeiling);
        pushButton_accetpAdjusment->setObjectName(QString::fromUtf8("pushButton_accetpAdjusment"));

        horizontalLayout_10->addWidget(pushButton_accetpAdjusment);

        pushButton_cancelAdjustment = new QPushButton(GroupBox_modelingCeiling);
        pushButton_cancelAdjustment->setObjectName(QString::fromUtf8("pushButton_cancelAdjustment"));

        horizontalLayout_10->addWidget(pushButton_cancelAdjustment);


        verticalLayout_6->addLayout(horizontalLayout_10);


        horizontalLayout_16->addLayout(verticalLayout_6);

        horizontalSpacer_7 = new QSpacerItem(70, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_16->addItem(horizontalSpacer_7);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        pushButton_birdView = new QPushButton(GroupBox_modelingCeiling);
        pushButton_birdView->setObjectName(QString::fromUtf8("pushButton_birdView"));

        horizontalLayout_14->addWidget(pushButton_birdView);

        pushButton_lateralView = new QPushButton(GroupBox_modelingCeiling);
        pushButton_lateralView->setObjectName(QString::fromUtf8("pushButton_lateralView"));

        horizontalLayout_14->addWidget(pushButton_lateralView);


        verticalLayout_5->addLayout(horizontalLayout_14);

        pushButton_DTsOnEditPlane = new QPushButton(GroupBox_modelingCeiling);
        pushButton_DTsOnEditPlane->setObjectName(QString::fromUtf8("pushButton_DTsOnEditPlane"));

        verticalLayout_5->addWidget(pushButton_DTsOnEditPlane);


        horizontalLayout_16->addLayout(verticalLayout_5);


        verticalLayout_3->addWidget(GroupBox_modelingCeiling);

        GroupBox_IGM = new QGroupBox(FloorPlanDialog);
        GroupBox_IGM->setObjectName(QString::fromUtf8("GroupBox_IGM"));
        GroupBox_IGM->setEnabled(true);
        GroupBox_IGM->setCheckable(false);
        GroupBox_IGM->setChecked(false);
        horizontalLayout_13 = new QHBoxLayout(GroupBox_IGM);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        pushButton_PolygonPlanes = new QPushButton(GroupBox_IGM);
        pushButton_PolygonPlanes->setObjectName(QString::fromUtf8("pushButton_PolygonPlanes"));
        pushButton_PolygonPlanes->setAutoFillBackground(true);

        horizontalLayout_3->addWidget(pushButton_PolygonPlanes);

        pushButton_beginGTSDetection_ = new QPushButton(GroupBox_IGM);
        pushButton_beginGTSDetection_->setObjectName(QString::fromUtf8("pushButton_beginGTSDetection_"));
        pushButton_beginGTSDetection_->setAutoFillBackground(true);

        horizontalLayout_3->addWidget(pushButton_beginGTSDetection_);


        verticalLayout_7->addLayout(horizontalLayout_3);

        pushButton_backprojection = new QPushButton(GroupBox_IGM);
        pushButton_backprojection->setObjectName(QString::fromUtf8("pushButton_backprojection"));
        pushButton_backprojection->setAutoFillBackground(true);

        verticalLayout_7->addWidget(pushButton_backprojection);


        horizontalLayout_13->addLayout(verticalLayout_7);

        horizontalSpacer_8 = new QSpacerItem(154, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_13->addItem(horizontalSpacer_8);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_2 = new QLabel(GroupBox_IGM);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_6->addWidget(label_2);

        doubleSpinBox_windowDepth = new QDoubleSpinBox(GroupBox_IGM);
        doubleSpinBox_windowDepth->setObjectName(QString::fromUtf8("doubleSpinBox_windowDepth"));

        horizontalLayout_6->addWidget(doubleSpinBox_windowDepth);


        horizontalLayout_11->addLayout(horizontalLayout_6);

        pushButton_addWindowPlanes = new QPushButton(GroupBox_IGM);
        pushButton_addWindowPlanes->setObjectName(QString::fromUtf8("pushButton_addWindowPlanes"));
        pushButton_addWindowPlanes->setAutoFillBackground(true);

        horizontalLayout_11->addWidget(pushButton_addWindowPlanes);


        verticalLayout_8->addLayout(horizontalLayout_11);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        pushButton_accpectWindowPlanes = new QPushButton(GroupBox_IGM);
        pushButton_accpectWindowPlanes->setObjectName(QString::fromUtf8("pushButton_accpectWindowPlanes"));

        horizontalLayout_12->addWidget(pushButton_accpectWindowPlanes);

        pushButton_cacelWindowPlanes = new QPushButton(GroupBox_IGM);
        pushButton_cacelWindowPlanes->setObjectName(QString::fromUtf8("pushButton_cacelWindowPlanes"));

        horizontalLayout_12->addWidget(pushButton_cacelWindowPlanes);


        verticalLayout_8->addLayout(horizontalLayout_12);


        horizontalLayout_13->addLayout(verticalLayout_8);


        verticalLayout_3->addWidget(GroupBox_IGM);

        GroupBox_gettingSlices->raise();
        GroupBox_gettingFloorPlan->raise();
        GroupBox_modelingCeiling->raise();
        GroupBox_IGM->raise();
        groupBox->raise();
#ifndef QT_NO_SHORTCUT
        label_KNN->setBuddy(spinBox_knn);
        label_Threshold->setBuddy(doubleSpinBox_threshold);
        Label_setStepValue->setBuddy(doubleSpinBox_stepValue);
        Label_setLineWidth->setBuddy(doubleSpinBox_lineWidth);
        Label_setStepValue_3->setBuddy(doubleSpinBox_curveMargin);
        Label_setMRFValue->setBuddy(doubleSpinBox_MRFLambda);
#endif // QT_NO_SHORTCUT
        QWidget::setTabOrder(pushButton_insistRegionDetection, pushButton_divide_slices);
        QWidget::setTabOrder(pushButton_divide_slices, pushButton_mainDireciongs);
        QWidget::setTabOrder(pushButton_mainDireciongs, horizontalSlider_Slayer);
        QWidget::setTabOrder(horizontalSlider_Slayer, horizontalSlider_Elayer);
        QWidget::setTabOrder(horizontalSlider_Elayer, pushButton_floorplanRec);

        retranslateUi(FloorPlanDialog);

        QMetaObject::connectSlotsByName(FloorPlanDialog);
    } // setupUi

    void retranslateUi(QDialog *FloorPlanDialog)
    {
        FloorPlanDialog->setWindowTitle(QApplication::translate("FloorPlanDialog", "FloorPlan Reconstruction", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("FloorPlanDialog", " Insistent Region Detection", 0, QApplication::UnicodeUTF8));
        label_KNN->setText(QApplication::translate("FloorPlanDialog", "&Nearest Neighbours Number", 0, QApplication::UnicodeUTF8));
        label_Threshold->setText(QApplication::translate("FloorPlanDialog", "&Threshold", 0, QApplication::UnicodeUTF8));
        checkBox_dispInconsist->setText(QApplication::translate("FloorPlanDialog", "Display Inconsist Region", 0, QApplication::UnicodeUTF8));
        pushButton_insistRegionDetection->setText(QApplication::translate("FloorPlanDialog", "Detecting", 0, QApplication::UnicodeUTF8));
        GroupBox_gettingSlices->setTitle(QApplication::translate("FloorPlanDialog", "Getting Slices", 0, QApplication::UnicodeUTF8));
        Label_setStepValue->setText(QApplication::translate("FloorPlanDialog", "&Step Value", 0, QApplication::UnicodeUTF8));
        label_setLayerNumber->setText(QApplication::translate("FloorPlanDialog", "Layer Numbers", 0, QApplication::UnicodeUTF8));
        label_displayLayerNumber->setText(QString());
        checkBox_dispSices->setText(QApplication::translate("FloorPlanDialog", "Diplay Slices", 0, QApplication::UnicodeUTF8));
        pushButton_divide_slices->setText(QApplication::translate("FloorPlanDialog", "Dividing", 0, QApplication::UnicodeUTF8));
        label_mainDirecionNum->setText(QApplication::translate("FloorPlanDialog", "Main Direction Numbers", 0, QApplication::UnicodeUTF8));
        label_dispMainDirectionNumbers->setText(QString());
        label_minmum_angle->setText(QApplication::translate("FloorPlanDialog", "Minmum Angle:", 0, QApplication::UnicodeUTF8));
        pushButton_mainDireciongs->setText(QApplication::translate("FloorPlanDialog", "Computint Main Directions", 0, QApplication::UnicodeUTF8));
        GroupBox_gettingFloorPlan->setTitle(QApplication::translate("FloorPlanDialog", "Get Floor Plan Each Layer", 0, QApplication::UnicodeUTF8));
        label_setEndingLayer->setText(QApplication::translate("FloorPlanDialog", "Ending Layer", 0, QApplication::UnicodeUTF8));
        label_setStartingLayer->setText(QApplication::translate("FloorPlanDialog", "Sarting Layer", 0, QApplication::UnicodeUTF8));
        Label_setLineWidth->setText(QApplication::translate("FloorPlanDialog", "&Line Width", 0, QApplication::UnicodeUTF8));
        Label_setStepValue_3->setText(QApplication::translate("FloorPlanDialog", "&Curve Margin", 0, QApplication::UnicodeUTF8));
        Label_setMRFValue->setText(QApplication::translate("FloorPlanDialog", "&MRF Optimization Params", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("FloorPlanDialog", "Letter Margin", 0, QApplication::UnicodeUTF8));
        checkBox_dispProcess->setText(QApplication::translate("FloorPlanDialog", "Display Reconstruction Process", 0, QApplication::UnicodeUTF8));
        checkBox_dispModel->setText(QApplication::translate("FloorPlanDialog", "Display Final Model", 0, QApplication::UnicodeUTF8));
        pushButton_floorplanRec->setText(QApplication::translate("FloorPlanDialog", "OK", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("FloorPlanDialog", "Cancel", 0, QApplication::UnicodeUTF8));
        GroupBox_modelingCeiling->setTitle(QApplication::translate("FloorPlanDialog", "Modeling Ceiling", 0, QApplication::UnicodeUTF8));
        checkBox_edit_mode->setText(QApplication::translate("FloorPlanDialog", "Edit Mode", 0, QApplication::UnicodeUTF8));
        pushButton_addCeilingPlane->setText(QApplication::translate("FloorPlanDialog", "Add Ceiling plane", 0, QApplication::UnicodeUTF8));
        pushButton_acceptFloorPlygon->setText(QApplication::translate("FloorPlanDialog", "Accept", 0, QApplication::UnicodeUTF8));
        pushButton_cancelFloorPolygon->setText(QApplication::translate("FloorPlanDialog", "Cancel", 0, QApplication::UnicodeUTF8));
        checkBox_adjustment_mode->setText(QApplication::translate("FloorPlanDialog", "Adjustment Mode", 0, QApplication::UnicodeUTF8));
        pushButton_selecPointsOnEditPlane->setText(QApplication::translate("FloorPlanDialog", "Select Points", 0, QApplication::UnicodeUTF8));
        pushButton_bundleVertices->setText(QApplication::translate("FloorPlanDialog", "Bundle Planes", 0, QApplication::UnicodeUTF8));
        pushButton_accetpAdjusment->setText(QApplication::translate("FloorPlanDialog", "Accept", 0, QApplication::UnicodeUTF8));
        pushButton_cancelAdjustment->setText(QApplication::translate("FloorPlanDialog", "Cancel", 0, QApplication::UnicodeUTF8));
        pushButton_birdView->setText(QApplication::translate("FloorPlanDialog", "Bird View", 0, QApplication::UnicodeUTF8));
        pushButton_lateralView->setText(QApplication::translate("FloorPlanDialog", "Lateral View", 0, QApplication::UnicodeUTF8));
        pushButton_DTsOnEditPlane->setText(QApplication::translate("FloorPlanDialog", "Delaunay Triangulation", 0, QApplication::UnicodeUTF8));
        GroupBox_IGM->setTitle(QApplication::translate("FloorPlanDialog", "Image Guided Modeling", 0, QApplication::UnicodeUTF8));
        pushButton_PolygonPlanes->setText(QApplication::translate("FloorPlanDialog", "Get Ploygon Planes", 0, QApplication::UnicodeUTF8));
        pushButton_beginGTSDetection_->setText(QApplication::translate("FloorPlanDialog", "GTS Detection", 0, QApplication::UnicodeUTF8));
        pushButton_backprojection->setText(QApplication::translate("FloorPlanDialog", "Back Projection", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("FloorPlanDialog", "Window Depth", 0, QApplication::UnicodeUTF8));
        pushButton_addWindowPlanes->setText(QApplication::translate("FloorPlanDialog", "Add Window Planes", 0, QApplication::UnicodeUTF8));
        pushButton_accpectWindowPlanes->setText(QApplication::translate("FloorPlanDialog", "Aceept ", 0, QApplication::UnicodeUTF8));
        pushButton_cacelWindowPlanes->setText(QApplication::translate("FloorPlanDialog", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FloorPlanDialog: public Ui_FloorPlanDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FLOORPLAN_H
