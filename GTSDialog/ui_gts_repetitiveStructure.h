/********************************************************************************
** Form generated from reading UI file 'gts_repetitiveStructure.ui'
**
** Created: Mon Nov 24 22:54:17 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GTS_REPETITIVESTRUCTURE_H
#define UI_GTS_REPETITIVESTRUCTURE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "gts_paintWidget.h"

QT_BEGIN_NAMESPACE

class Ui_GTSDetectionDialog
{
public:
    QHBoxLayout *horizontalLayout_6;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_input;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_7;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_8;
    QVBoxLayout *verticalLayout_3;
    PaintWidget *widget_rectified;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QLabel *label;
    QSpacerItem *horizontalSpacer_2;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_initialResults;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_5;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_6;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_output;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_3;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_loadImage;
    QSpacerItem *verticalSpacer_2;
    QPushButton *pushButton_quad;
    QPushButton *pushButton_rectification;
    QSpacerItem *verticalSpacer_3;
    QPushButton *pushButton_background;
    QPushButton *pushButton_foreground;
    QPushButton *pushButton_detect;
    QSpacerItem *verticalSpacer_4;
    QPushButton *pushButton_accept;
    QPushButton *pushButton_abandon;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *GTSDetectionDialog)
    {
        if (GTSDetectionDialog->objectName().isEmpty())
            GTSDetectionDialog->setObjectName(QString::fromUtf8("GTSDetectionDialog"));
        GTSDetectionDialog->resize(1405, 892);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(GTSDetectionDialog->sizePolicy().hasHeightForWidth());
        GTSDetectionDialog->setSizePolicy(sizePolicy);
        horizontalLayout_6 = new QHBoxLayout(GTSDetectionDialog);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_input = new QLabel(GTSDetectionDialog);
        label_input->setObjectName(QString::fromUtf8("label_input"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_input->sizePolicy().hasHeightForWidth());
        label_input->setSizePolicy(sizePolicy1);
        label_input->setMinimumSize(QSize(600, 400));
        label_input->setAutoFillBackground(true);
        label_input->setFrameShape(QFrame::Panel);
        label_input->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_2->addWidget(label_input);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_7);

        label_4 = new QLabel(GTSDetectionDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        sizePolicy1.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy1);
        QFont font;
        font.setPointSize(15);
        font.setBold(true);
        font.setWeight(75);
        label_4->setFont(font);

        horizontalLayout_4->addWidget(label_4);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_8);


        verticalLayout_2->addLayout(horizontalLayout_4);


        gridLayout->addLayout(verticalLayout_2, 0, 0, 1, 1);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        widget_rectified = new PaintWidget(GTSDetectionDialog);
        widget_rectified->setObjectName(QString::fromUtf8("widget_rectified"));
        sizePolicy1.setHeightForWidth(widget_rectified->sizePolicy().hasHeightForWidth());
        widget_rectified->setSizePolicy(sizePolicy1);
        widget_rectified->setMinimumSize(QSize(600, 400));
        widget_rectified->setMouseTracking(true);
        widget_rectified->setAutoFillBackground(true);

        verticalLayout_3->addWidget(widget_rectified);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        label = new QLabel(GTSDetectionDialog);
        label->setObjectName(QString::fromUtf8("label"));
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        label->setFont(font);

        horizontalLayout->addWidget(label);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);


        verticalLayout_3->addLayout(horizontalLayout);


        gridLayout->addLayout(verticalLayout_3, 0, 1, 1, 1);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_initialResults = new QLabel(GTSDetectionDialog);
        label_initialResults->setObjectName(QString::fromUtf8("label_initialResults"));
        sizePolicy1.setHeightForWidth(label_initialResults->sizePolicy().hasHeightForWidth());
        label_initialResults->setSizePolicy(sizePolicy1);
        label_initialResults->setMinimumSize(QSize(600, 400));
        label_initialResults->setAutoFillBackground(true);
        label_initialResults->setFrameShape(QFrame::Panel);
        label_initialResults->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_4->addWidget(label_initialResults);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_5);

        label_3 = new QLabel(GTSDetectionDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        sizePolicy1.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy1);
        label_3->setFont(font);

        horizontalLayout_3->addWidget(label_3);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_6);


        verticalLayout_4->addLayout(horizontalLayout_3);


        gridLayout->addLayout(verticalLayout_4, 1, 0, 1, 1);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label_output = new QLabel(GTSDetectionDialog);
        label_output->setObjectName(QString::fromUtf8("label_output"));
        sizePolicy1.setHeightForWidth(label_output->sizePolicy().hasHeightForWidth());
        label_output->setSizePolicy(sizePolicy1);
        label_output->setMinimumSize(QSize(600, 400));
        label_output->setAutoFillBackground(true);
        label_output->setFrameShape(QFrame::Panel);
        label_output->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        verticalLayout_5->addWidget(label_output);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);

        label_2 = new QLabel(GTSDetectionDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        sizePolicy1.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy1);
        label_2->setFont(font);

        horizontalLayout_2->addWidget(label_2);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_4);


        verticalLayout_5->addLayout(horizontalLayout_2);


        gridLayout->addLayout(verticalLayout_5, 1, 1, 1, 1);


        horizontalLayout_6->addLayout(gridLayout);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_loadImage = new QPushButton(GTSDetectionDialog);
        pushButton_loadImage->setObjectName(QString::fromUtf8("pushButton_loadImage"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(pushButton_loadImage->sizePolicy().hasHeightForWidth());
        pushButton_loadImage->setSizePolicy(sizePolicy2);

        verticalLayout->addWidget(pushButton_loadImage);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        pushButton_quad = new QPushButton(GTSDetectionDialog);
        pushButton_quad->setObjectName(QString::fromUtf8("pushButton_quad"));
        sizePolicy1.setHeightForWidth(pushButton_quad->sizePolicy().hasHeightForWidth());
        pushButton_quad->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(pushButton_quad);

        pushButton_rectification = new QPushButton(GTSDetectionDialog);
        pushButton_rectification->setObjectName(QString::fromUtf8("pushButton_rectification"));

        verticalLayout->addWidget(pushButton_rectification);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        pushButton_background = new QPushButton(GTSDetectionDialog);
        pushButton_background->setObjectName(QString::fromUtf8("pushButton_background"));
        pushButton_background->setAutoFillBackground(true);

        verticalLayout->addWidget(pushButton_background);

        pushButton_foreground = new QPushButton(GTSDetectionDialog);
        pushButton_foreground->setObjectName(QString::fromUtf8("pushButton_foreground"));
        pushButton_foreground->setAutoFillBackground(true);

        verticalLayout->addWidget(pushButton_foreground);

        pushButton_detect = new QPushButton(GTSDetectionDialog);
        pushButton_detect->setObjectName(QString::fromUtf8("pushButton_detect"));
        pushButton_detect->setAutoFillBackground(true);

        verticalLayout->addWidget(pushButton_detect);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_4);

        pushButton_accept = new QPushButton(GTSDetectionDialog);
        pushButton_accept->setObjectName(QString::fromUtf8("pushButton_accept"));

        verticalLayout->addWidget(pushButton_accept);

        pushButton_abandon = new QPushButton(GTSDetectionDialog);
        pushButton_abandon->setObjectName(QString::fromUtf8("pushButton_abandon"));

        verticalLayout->addWidget(pushButton_abandon);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout_5->addLayout(verticalLayout);


        horizontalLayout_6->addLayout(horizontalLayout_5);


        retranslateUi(GTSDetectionDialog);

        QMetaObject::connectSlotsByName(GTSDetectionDialog);
    } // setupUi

    void retranslateUi(QWidget *GTSDetectionDialog)
    {
        GTSDetectionDialog->setWindowTitle(QApplication::translate("GTSDetectionDialog", "Form", 0, QApplication::UnicodeUTF8));
        label_input->setText(QString());
        label_4->setText(QApplication::translate("GTSDetectionDialog", "Input Image", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("GTSDetectionDialog", "Rectified  Image", 0, QApplication::UnicodeUTF8));
        label_initialResults->setText(QString());
        label_3->setText(QApplication::translate("GTSDetectionDialog", "Initial Results", 0, QApplication::UnicodeUTF8));
        label_output->setText(QString());
        label_2->setText(QApplication::translate("GTSDetectionDialog", "Output Image", 0, QApplication::UnicodeUTF8));
        pushButton_loadImage->setText(QApplication::translate("GTSDetectionDialog", "Load Image", 0, QApplication::UnicodeUTF8));
        pushButton_quad->setText(QApplication::translate("GTSDetectionDialog", "Select A Quadrilateral", 0, QApplication::UnicodeUTF8));
        pushButton_rectification->setText(QApplication::translate("GTSDetectionDialog", "Rectification", 0, QApplication::UnicodeUTF8));
        pushButton_background->setText(QApplication::translate("GTSDetectionDialog", "Backgroud", 0, QApplication::UnicodeUTF8));
        pushButton_foreground->setText(QApplication::translate("GTSDetectionDialog", "GTS", 0, QApplication::UnicodeUTF8));
        pushButton_detect->setText(QApplication::translate("GTSDetectionDialog", "Detect", 0, QApplication::UnicodeUTF8));
        pushButton_accept->setText(QApplication::translate("GTSDetectionDialog", "Accept", 0, QApplication::UnicodeUTF8));
        pushButton_abandon->setText(QApplication::translate("GTSDetectionDialog", "PushButton", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GTSDetectionDialog: public Ui_GTSDetectionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GTS_REPETITIVESTRUCTURE_H
