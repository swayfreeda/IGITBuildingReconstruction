//================================================================================================//

// Description: This is a file containes PLYIO class for importing and exporting dense points
//              from and to PLY files.
// Author: sway
// Time:  11/04/2014
// Organization: Institute of Automation, Chinese Academy of Science

//===============================================================================================//




#ifndef SW_PLYIO_H
#define SW_PLYIO_H

#include "sw_dataType.h"

#include <QObject>
#include<QString>
#include<QVector>
#include<QVector3D>
#include<QVector2D>

typedef PointXYZRGBNormal Point;

namespace SW
{
class PLYIO : public QObject
{
    Q_OBJECT
public:
    PLYIO(){}
    PLYIO(QString name){file_name_ = name;}


    //-------------------------------------------loadPointsFromPLY----------------------------//
    //load pionts from PLY file, and each point containes 9 elements( 3D world coordinates, corlors,
    // and normals).
    // *input variables:
    // NONE

    // * return variables:
    //  vertices:  3D points containing point colors, coordinates, normals, and so on
    bool loadPointsFromPLY(QVector<Point> & vertices);





    //--------------------------------------------savePointsToPLY-----------------------------//
    //save pionts to PLY file, and each point containes 9 elements( 3D world coordinates, corlors,
    // and normals).
    // *input variables:
    // vertices:  3D points containing point colors, coordinates, normals, and so on

    // *return variables:
    //NONE
    bool savePointsToPLY(QVector<Point> & vertices);


signals:
    void statusBar(QString text);

private:
    QString file_name_;
};
}

#endif // SW_PLYIO_H
