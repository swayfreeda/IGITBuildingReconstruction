//================================================================================================//

// Description: This is a file containes PLYIO class for importing and exporting dense points
//              from and to PLY files.
// Author: sway
// Time:  11/04/2014
// Organization: Institute of Automation, Chinese Academy of Science


// Commit: The PLYIO class is renamed as DATAIO, and the class will be used for all the functions
//         related to load or save files.
//===============================================================================================//




#ifndef SW_DATAIO_H
#define SW_DATAIO_H

#include "sw_dataType.h"

#include <QObject>
#include<QString>
#include<QVector>
#include<QVector3D>
#include<QVector2D>

typedef PointXYZRGBNormal Point;

namespace SW
{
class DATAIO : public QObject
{
    Q_OBJECT
public:
    DATAIO(){}
    DATAIO(QString name){folder_name_ = name;}


    void setFolderDir(QString dir){ folder_name_ = dir;}


    //-------------------------------------------getPLYFileDirs-------------------------------//
    // the PLY Files is stored in the pmvs/models/
    QVector<QString> getPLYFileDirs();

    //-------------------------------------------getVisFileDirs-------------------------------//
    // the visibility files are stored in the pmvs/models/
    QVector<QString> getVisFileDirs();

    //-------------------------------------------getImageDirs---------------------------------//
    // images are stored in the pmvs/visualize/
    QVector<QString> getImageDirs();

    //-------------------------------------------getCameraDirs--------------------------------//
    // project matrixs are stored in pmvs/txt/
    QVector<QString> getCameraDirs();


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
    // vertices:  3D points containing point colors, coordinates, normals, and so on.
    // file_name: name of file to be saved.

    // *return variables:
    //NONE
    bool savePointsToPLY(QVector<Point> & vertices, QString file_name);



    //----------------------------------------------loadImages--------------------------------//
    // load images from the visulize/ folder, and both the name of each image and the image are
    // stored.
    // *input variables
    //  None

    // * output variables
    // images: containes all the images and their names
    bool loadImages(QMap<QString, QImage> & images);



    //---------------------------------------------load cameras-------------------------------//
    // load all the projection matrixs from the txt files, and both the names for each camera and
    // the names are stored.
    // *input variables
    // NONE
    // *output variables
    // cameras: containes all the cameras and their names.
    bool loadCameras(QMap<QString, Camera> & cameras);


    //---------------------------------------------load visibility-----------------------------//
    // load all the visibility of the points
    // *input variable
    // NONE
    // *output variable
    // vertices: attach the visibility information of each vertex to each point
    bool loadVisiblities(QVector<Point> & vertices);




signals:
    void statusBar(QString text);

private:

    // folder is defaulted as pmvs/ , there are two subfolders and two files included: txt/ contianes
    // all the prjection matrixs of the camera; visualize/ containes all the images; .*ply contains all
    // the points and *.patch containes the visibility infomation
    QString folder_name_;
};
}

#endif // SW_PLYIO_H
