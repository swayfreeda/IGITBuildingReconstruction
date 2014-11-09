/********************************************************************************
** Form generated from reading UI file 'FloorPlan.ui'
**
** Created: Sun Nov 9 21:32:30 2014
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
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_divide_slices;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLabel *label_MainDirectionNumbers;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *pushButton_mainDireciongs;
    QGroupBox *GroupBox_gettingFloorPlan;
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout;
    QLabel *label_setEndingLayer;
    QSlider *horizontalSlider;
    QLabel *label_setStartingLayer;
    QSlider *horizontalSlider_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *Label_setLineWidth;
    QDoubleSpinBox *doubleSpinBox_lineWidth;
    QSpacerItem *horizontalSpacer_4;
    QLabel *Label_setStepValue_3;
    QDoubleSpinBox *doubleSpinBox_curveMargin;
    QHBoxLayout *horizontalLayout_2;
    QLabel *Label_setMRFValue;
    QDoubleSpinBox *doubleSpinBox_MRFLambda;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *pushButton_2;
    QGroupBox *GroupBox_modelingCeiling;
    QGroupBox *GroupBox_IGM;
    QHBoxLayout *horizontalLayout_6;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_setImageNumber;
    QSpinBox *spinBox_imageID;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *pushButton;

    void setupUi(QDialog *FloorPlanDialog)
    {
        if (FloorPlanDialog->objectName().isEmpty())
            FloorPlanDialog->setObjectName(QString::fromUtf8("FloorPlanDialog"));
        FloorPlanDialog->resize(809, 640);
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

        gridLayout_2->addWidget(doubleSpinBox_stepValue, 0, 1, 1, 1);

        Label_setStepValue = new QLabel(GroupBox_gettingSlices);
        Label_setStepValue->setObjectName(QString::fromUtf8("Label_setStepValue"));

        gridLayout_2->addWidget(Label_setStepValue, 0, 0, 1, 1);

        label_setLayerNumber = new QLabel(GroupBox_gettingSlices);
        label_setLayerNumber->setObjectName(QString::fromUtf8("label_setLayerNumber"));

        gridLayout_2->addWidget(label_setLayerNumber, 1, 0, 1, 1);

        label_displayLayerNumber = new QLabel(GroupBox_gettingSlices);
        label_displayLayerNumber->setObjectName(QString::fromUtf8("label_displayLayerNumber"));

        gridLayout_2->addWidget(label_displayLayerNumber, 1, 1, 1, 1);


        horizontalLayout_5->addLayout(gridLayout_2);

        horizontalSpacer = new QSpacerItem(331, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);

        pushButton_divide_slices = new QPushButton(GroupBox_gettingSlices);
        pushButton_divide_slices->setObjectName(QString::fromUtf8("pushButton_divide_slices"));

        horizontalLayout_5->addWidget(pushButton_divide_slices);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(GroupBox_gettingSlices);
        label->setObjectName(QString::fromUtf8("label"));
        label->setEnabled(false);

        horizontalLayout->addWidget(label);

        label_MainDirectionNumbers = new QLabel(GroupBox_gettingSlices);
        label_MainDirectionNumbers->setObjectName(QString::fromUtf8("label_MainDirectionNumbers"));

        horizontalLayout->addWidget(label_MainDirectionNumbers);

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

        horizontalSlider = new QSlider(GroupBox_gettingFloorPlan);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider, 0, 1, 1, 1);

        label_setStartingLayer = new QLabel(GroupBox_gettingFloorPlan);
        label_setStartingLayer->setObjectName(QString::fromUtf8("label_setStartingLayer"));

        gridLayout->addWidget(label_setStartingLayer, 0, 0, 1, 1);

        horizontalSlider_2 = new QSlider(GroupBox_gettingFloorPlan);
        horizontalSlider_2->setObjectName(QString::fromUtf8("horizontalSlider_2"));
        horizontalSlider_2->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_2, 1, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        Label_setLineWidth = new QLabel(GroupBox_gettingFloorPlan);
        Label_setLineWidth->setObjectName(QString::fromUtf8("Label_setLineWidth"));

        horizontalLayout_4->addWidget(Label_setLineWidth);

        doubleSpinBox_lineWidth = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_lineWidth->setObjectName(QString::fromUtf8("doubleSpinBox_lineWidth"));

        horizontalLayout_4->addWidget(doubleSpinBox_lineWidth);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_4);

        Label_setStepValue_3 = new QLabel(GroupBox_gettingFloorPlan);
        Label_setStepValue_3->setObjectName(QString::fromUtf8("Label_setStepValue_3"));

        horizontalLayout_4->addWidget(Label_setStepValue_3);

        doubleSpinBox_curveMargin = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_curveMargin->setObjectName(QString::fromUtf8("doubleSpinBox_curveMargin"));

        horizontalLayout_4->addWidget(doubleSpinBox_curveMargin);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        Label_setMRFValue = new QLabel(GroupBox_gettingFloorPlan);
        Label_setMRFValue->setObjectName(QString::fromUtf8("Label_setMRFValue"));

        horizontalLayout_2->addWidget(Label_setMRFValue);

        doubleSpinBox_MRFLambda = new QDoubleSpinBox(GroupBox_gettingFloorPlan);
        doubleSpinBox_MRFLambda->setObjectName(QString::fromUtf8("doubleSpinBox_MRFLambda"));

        horizontalLayout_2->addWidget(doubleSpinBox_MRFLambda);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        pushButton_2 = new QPushButton(GroupBox_gettingFloorPlan);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout_2->addWidget(pushButton_2);


        verticalLayout_2->addLayout(horizontalLayout_2);


        verticalLayout_3->addWidget(GroupBox_gettingFloorPlan);

        GroupBox_modelingCeiling = new QGroupBox(FloorPlanDialog);
        GroupBox_modelingCeiling->setObjectName(QString::fromUtf8("GroupBox_modelingCeiling"));
        GroupBox_modelingCeiling->setEnabled(false);
        GroupBox_modelingCeiling->setFlat(false);
        GroupBox_modelingCeiling->setCheckable(false);
        GroupBox_modelingCeiling->setChecked(false);

        verticalLayout_3->addWidget(GroupBox_modelingCeiling);

        GroupBox_IGM = new QGroupBox(FloorPlanDialog);
        GroupBox_IGM->setObjectName(QString::fromUtf8("GroupBox_IGM"));
        GroupBox_IGM->setEnabled(false);
        GroupBox_IGM->setCheckable(false);
        GroupBox_IGM->setChecked(false);
        horizontalLayout_6 = new QHBoxLayout(GroupBox_IGM);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_setImageNumber = new QLabel(GroupBox_IGM);
        label_setImageNumber->setObjectName(QString::fromUtf8("label_setImageNumber"));

        horizontalLayout_3->addWidget(label_setImageNumber);

        spinBox_imageID = new QSpinBox(GroupBox_IGM);
        spinBox_imageID->setObjectName(QString::fromUtf8("spinBox_imageID"));

        horizontalLayout_3->addWidget(spinBox_imageID);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);

        pushButton = new QPushButton(GroupBox_IGM);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_3->addWidget(pushButton);


        horizontalLayout_6->addLayout(horizontalLayout_3);


        verticalLayout_3->addWidget(GroupBox_IGM);

        GroupBox_gettingSlices->raise();
        GroupBox_gettingFloorPlan->raise();
        GroupBox_modelingCeiling->raise();
        GroupBox_IGM->raise();
        groupBox->raise();
#ifndef QT_NO_SHORTCUT
        label_Threshold->setBuddy(doubleSpinBox_threshold);
        Label_setStepValue->setBuddy(doubleSpinBox_stepValue);
        Label_setLineWidth->setBuddy(doubleSpinBox_lineWidth);
        Label_setStepValue_3->setBuddy(doubleSpinBox_curveMargin);
        Label_setMRFValue->setBuddy(doubleSpinBox_MRFLambda);
        label_setImageNumber->setBuddy(spinBox_imageID);
#endif // QT_NO_SHORTCUT
        QWidget::setTabOrder(pushButton_insistRegionDetection, pushButton_divide_slices);
        QWidget::setTabOrder(pushButton_divide_slices, pushButton_mainDireciongs);
        QWidget::setTabOrder(pushButton_mainDireciongs, horizontalSlider);
        QWidget::setTabOrder(horizontalSlider, horizontalSlider_2);
        QWidget::setTabOrder(horizontalSlider_2, pushButton_2);
        QWidget::setTabOrder(pushButton_2, pushButton);

        retranslateUi(FloorPlanDialog);

        QMetaObject::connectSlotsByName(FloorPlanDialog);
    } // setupUi

    void retranslateUi(QDialog *FloorPlanDialog)
    {
        FloorPlanDialog->setWindowTitle(QApplication::translate("FloorPlanDialog", "Floor Plan Reconstruction", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("FloorPlanDialog", " Insistent Region Detection", 0, QApplication::UnicodeUTF8));
        label_KNN->setText(QApplication::translate("FloorPlanDialog", "&Nearest Neighbours Number", 0, QApplication::UnicodeUTF8));
        label_Threshold->setText(QApplication::translate("FloorPlanDialog", "&Threshold", 0, QApplication::UnicodeUTF8));
        pushButton_insistRegionDetection->setText(QApplication::translate("FloorPlanDialog", "Detecting", 0, QApplication::UnicodeUTF8));
        GroupBox_gettingSlices->setTitle(QApplication::translate("FloorPlanDialog", "Getting Slices", 0, QApplication::UnicodeUTF8));
        Label_setStepValue->setText(QApplication::translate("FloorPlanDialog", "&Step Value", 0, QApplication::UnicodeUTF8));
        label_setLayerNumber->setText(QApplication::translate("FloorPlanDialog", "Layer Numbers", 0, QApplication::UnicodeUTF8));
        label_displayLayerNumber->setText(QString());
        pushButton_divide_slices->setText(QApplication::translate("FloorPlanDialog", "Dividing", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("FloorPlanDialog", "Main DIrection Numbers", 0, QApplication::UnicodeUTF8));
        label_MainDirectionNumbers->setText(QString());
        pushButton_mainDireciongs->setText(QApplication::translate("FloorPlanDialog", "Computint Main Directions", 0, QApplication::UnicodeUTF8));
        GroupBox_gettingFloorPlan->setTitle(QApplication::translate("FloorPlanDialog", "Get Floor Plan Each Layer", 0, QApplication::UnicodeUTF8));
        label_setEndingLayer->setText(QApplication::translate("FloorPlanDialog", "Ending Layer", 0, QApplication::UnicodeUTF8));
        label_setStartingLayer->setText(QApplication::translate("FloorPlanDialog", "Sarting Layer", 0, QApplication::UnicodeUTF8));
        Label_setLineWidth->setText(QApplication::translate("FloorPlanDialog", "&Line Width", 0, QApplication::UnicodeUTF8));
        Label_setStepValue_3->setText(QApplication::translate("FloorPlanDialog", "&Curve Margin", 0, QApplication::UnicodeUTF8));
        Label_setMRFValue->setText(QApplication::translate("FloorPlanDialog", "&MRF Optimization Params", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("FloorPlanDialog", "Begining", 0, QApplication::UnicodeUTF8));
        GroupBox_modelingCeiling->setTitle(QApplication::translate("FloorPlanDialog", "Modeling Ceiling", 0, QApplication::UnicodeUTF8));
        GroupBox_IGM->setTitle(QApplication::translate("FloorPlanDialog", "Image Guided Modeling", 0, QApplication::UnicodeUTF8));
        label_setImageNumber->setText(QApplication::translate("FloorPlanDialog", "Select An Image", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("FloorPlanDialog", "Begining", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FloorPlanDialog: public Ui_FloorPlanDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FLOORPLAN_H
