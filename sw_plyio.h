//================================================================================================//

// Description: This is a file containes PLYIO class for importing and exporting dense points
//              from and to PLY files.
// Author: sway
// Time:  11/04/2014
// Organization: Institute of Automation, Chinese Academy of Science

//===============================================================================================//




#ifndef SW_PLYIO_H
#define SW_PLYIO_H
#include <QObject>
#include<QString>
#include<QVector>
#include<QVector3D>
#include<QVector2D>


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
    // vertices:  3D world coordinates of the points
    // colors:    colors of the points
    // normals:  normals of the points
    bool loadPointsFromPLY(QVector<QVector3D> & vertices,
                           QVector<QVector3D> & colors,
                           QVector<QVector3D> & normals);





    //--------------------------------------------savePointsToPLY-----------------------------//
    //save pionts to PLY file, and each point containes 9 elements( 3D world coordinates, corlors,
    // and normals).
    // *input variables:
    // vertices:  3D world coordinates of the points
    // colors:    colors of the points
    // normals:  normals of the points

    // *return variables:
    //NONE
    bool savePointsToPLY(QVector<QVector3D> & vertices,
                           QVector<QVector3D> & colors,
                           QVector<QVector3D> & normals);




private:
    QString file_name_;
};
}

#endif // SW_PLYIO_H
