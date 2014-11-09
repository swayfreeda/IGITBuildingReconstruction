#include "sw_dataIO.h"

#include <QFile>
#include <QTextStream>
#include <QProgressDialog>
#include<QApplication>
#include<QMessageBox>
#include<QDir>


//-------------------------------------------getPLYFileDir-------------------------------//
QVector<QString> SW::DATAIO::getPLYFileDirs()
{
    QVector<QString> ply_file_names;
    QDir dir;
    if(!dir.exists(folder_name_ + tr("/models"))) return ply_file_names;

    QString path = folder_name_ + tr("/models");
    dir.setPath(path);
    dir.setFilter(QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot);
    QStringList filter;
    filter<<"*.ply";

    // ply files
    QFileInfoList fileList = dir.entryInfoList(filter);
    int nFiles = fileList.size();

    for(int i=0; i< nFiles; i++)
    {
        ply_file_names.append( fileList.at(i).filePath());
    }

    return ply_file_names;
}
//-------------------------------------------getVisFileDir-------------------------------//
QVector<QString> SW::DATAIO::getVisFileDirs()
{
    QVector<QString> patch_file_names;
    QDir dir;
    if(!dir.exists(folder_name_ + tr("/models"))) return patch_file_names;

    QString path = folder_name_ + tr("/models");
    dir.setPath(path);
    dir.setFilter(QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot);
    QStringList filter;
    filter<<"*.patch";

    // ply files
    QFileInfoList fileList = dir.entryInfoList(filter);
    int nFiles = fileList.size();

    for(int i=0; i< nFiles; i++)
    {
        patch_file_names.append( fileList.at(i).filePath());
    }

    return patch_file_names;
}
//-------------------------------------------getImageFolderDir---------------------------//
QVector<QString> SW::DATAIO::getImageDirs()
{
    QVector<QString> img_file_names;
    QDir dir;
    if(!dir.exists(folder_name_ + tr("/visualize"))) return img_file_names;

    QString path = folder_name_ + tr("/visualize");
    dir.setPath(path);
    dir.setFilter(QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot);
    QStringList filter;
    filter<<QString("*.jpg")<<QString("*.jpeg")<<QString("*.png")<<QString("*.JPG");

    // ply files
    QFileInfoList fileList = dir.entryInfoList(filter);
    int nFiles = fileList.size();

    for(int i=0; i< nFiles; i++)
    {
        img_file_names.append( fileList.at(i).filePath());
    }

    return img_file_names;

}
//-------------------------------------------getCameraFolderDir-------------------------//
QVector<QString> SW::DATAIO::getCameraDirs()
{
    QVector<QString> cam_file_names;
    QDir dir;
    if(!dir.exists(folder_name_ + tr("/txt"))) return cam_file_names;

    QString path = folder_name_ + tr("/txt");
    dir.setPath(path);
    dir.setFilter(QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot);
    QStringList filter;
    filter<<"*.txt";

    // ply files
    QFileInfoList fileList = dir.entryInfoList(filter);
    int nFiles = fileList.size();

    for(int i=0; i< nFiles; i++)
    {
        cam_file_names.append( fileList.at(i).filePath());
    }

    return cam_file_names;

}
//-------------------------------------------loadPointsFromPLY----------------------------//
bool SW::DATAIO::loadPointsFromPLY(QVector<Point> & vertices)
{

    QVector< QString>  file_names = getPLYFileDirs();

    int counter = 0;
    foreach(QString file_name, file_names)
    {

        QFile file(file_name);
        if(!file.open(QIODevice::ReadOnly))
        {
            emit statusBar("Fail to load points from " + file_name);
            return false;
        }

        // create a progress dialog
        QProgressDialog progress;
        progress.setLabelText(QString("%1 / %2").arg(counter+1).arg(file_names.size()));
        progress.setWindowModality(Qt::WindowModal);

        bool start_read = false;
        QTextStream in(&file);
        int counter = 0;

        QVector<Point> sub_cluster;
        while(!in.atEnd())
        {
            // get the line data
            QString line = in.readLine();
            QStringList fields = line.split(" ");

            // line date begins with "ply", "comment" or "format" will be ignored, because these lines
            // containes no useful information
            if (line.startsWith("ply") || line.startsWith("comment") || line.startsWith("format"))
            {
                continue;
            }

            // nummber of vetices
            if (line.startsWith("element vertex"))
            {
                fields.takeFirst();
                fields.takeFirst();
                int vertex_num = fields.takeFirst().toInt();
                sub_cluster.resize(vertex_num);

                progress.setRange(0, vertex_num);
                continue;
            }

            // end of header
            if (line.startsWith("end_header"))
            {
                start_read = true;
                counter =0;
                continue;
            }

            // read vertices
            if (start_read == true )
            {
                // 3D world coordinate
                sub_cluster[counter].x = fields.takeFirst().toFloat();
                sub_cluster[counter].y = fields.takeFirst().toFloat();
                sub_cluster[counter].z = fields.takeFirst().toFloat();

                // normal
                sub_cluster[counter].normal_x = fields.takeFirst().toFloat();
                sub_cluster[counter].normal_y = fields.takeFirst().toFloat();
                sub_cluster[counter].normal_z = fields.takeFirst().toFloat();

                // color
                sub_cluster[counter].r = fields.takeFirst().toFloat();
                sub_cluster[counter].g = fields.takeFirst().toFloat();
                sub_cluster[counter].b = fields.takeFirst().toFloat();

                counter ++;
                // set the value of progress diaog
                progress.setValue(counter);
                qApp->processEvents();
                if (progress.wasCanceled())
                {
                    vertices.clear();
                    return false;
                }
            }
        }

        foreach(Point pt, sub_cluster)
        {
            vertices.append(pt);
        }
        sub_cluster.clear();
        counter ++;
    }

    return true;
}
//--------------------------------------------savePointsToPLY-----------------------------//
bool SW::DATAIO::savePointsToPLY(QVector<Point> & vertices, QString file_name)
{
    QFile file(file_name);
    if (!file.open(QIODevice::WriteOnly))
    {
        emit statusBar(tr("Fail to Save PLY!"));
        return false;
    }

    // create a progress dialog
    QProgressDialog progress;
    progress.setLabelText(tr("Saving dense points..."));
    progress.setWindowModality(Qt::WindowModal);
    progress.setRange(0, vertices.size());

    QTextStream out(&file);
    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << vertices.size() << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property float nx" << endl;
    out << "property float ny" << endl;
    out << "property float nz" << endl;
    out <<"property uchar diffuse_red"<<endl;
    out <<"property uchar diffuse_green"<<endl;
    out <<"property uchar diffuse_blue"<<endl;
    out << "end_header" << endl;

    for(int i=0; i< vertices.size(); i++)
    {
        out << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << " ";
        out << vertices[i].normal_x << " " << vertices[i].normal_y << " " << vertices[i].normal_z << " ";
        out << vertices[i].r << " " << vertices[i].g << " " << vertices[i].b <<endl;

        progress.setValue(i);
        qApp->processEvents();
        if (progress.wasCanceled())
        {
            return false;
        }
    }

    QMessageBox::warning(NULL, tr("Information"), tr("Save Completed!"));

    return true;
}
//--------------------------------------------load images----------------------------------//
bool SW::DATAIO::loadImages(QMap<QString, QImage> & images)
{
    QVector<QString> img_dirs = getImageDirs();
    QProgressDialog progress;
    progress.setLabelText(tr("Loading Images..."));
    progress.setRange(0, img_dirs.size());
    progress.setWindowModality(Qt::WindowModal);

    int nSteps = 0;

    QImage img;
    //load images
    foreach(QString img_dir, img_dirs)
    {
        // set the value of progress diaog
        progress.setValue(nSteps);
        qApp->processEvents();
        if (progress.wasCanceled())
        {
            images.clear();
            return false;
        }
        if (!img_dir.isEmpty())
        {
            img.load(img_dir);

            QStringList fields = img_dir.split("/");
            QString name = fields.takeLast();
            images.insert(name, img);
        }
        nSteps++;
    }
    return true;
}
//---------------------------------------------load cameras-------------------------------//
bool SW::DATAIO::loadCameras(QMap<QString, Camera> & cameras)
{
    QVector<QString> cam_dirs = getCameraDirs();

    QProgressDialog progress;
    progress.setLabelText(tr("Load Cameras..."));
    progress.setRange(0, cam_dirs.size());
    progress.setWindowModality(Qt::WindowModal);

    int counter =0;
    foreach(QString cam_dir, cam_dirs)
    {
          // set the value of progress diaog
        // set the value of progress diaog
        progress.setValue(counter);
        qApp->processEvents();
        if (progress.wasCanceled())
        {
            cameras.clear();
            return false;
        }

        //  cout<<txt_name.toStdString()<<endl;
        QFile file(cam_dir);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

        Camera cam;
        QTextStream in(&file);
        int line_num = 0;
        while (!in.atEnd())
        {
            QString line = in.readLine();
            // cout<<line.toStdString()<<endl;
            if (line_num > 0)
            {
                QStringList fields = line.split(" ");
                for (int i = 0; i < 4; i++)
                {
                    cam.project_.at<float>(line_num - 1, i) = fields.takeFirst().toFloat();
                }
            }
            line_num++;
        }

        // get the rotation matrix, translation vector, focal, and camera axises of the
        // cameras
        cam.decomposeProjMats();

        // assign a color to the camera for displaying
        cam.color_= QColor((int)rand()&255, (int)rand()&255,(int)rand()&255);

        // get the name of the image which the camera the is correspoding to
        QStringList fields = cam_dir.split("/");
        QString name = fields.takeLast();
        name = name.replace(".txt", "jpg");

        cameras.insert(name, cam);

        counter++;
    }
    return true;
}
//---------------------------------------------load visibility-----------------------------//
bool SW::DATAIO::loadVisiblities(QVector<Point> & vertices)
{
    if(vertices.size() == 0)
    {
        QMessageBox::warning(0, tr("Warning"), tr("Loading Points First!"));
        return false;
    }
    // get the dirs of the patch files
    QVector<QString> vis_dirs = getVisFileDirs();

    QVector<QVector<uint> > all_vis;
    QVector<QVector<uint> > sub_vis;
    QVector<uint> tmp;
    int file_id = 0;

    foreach(QString vis_dir, vis_dirs)
    {
        sub_vis.clear();

        // create a progress dialog
        QProgressDialog progress;
        progress.setLabelText(QString("%1 / %2").arg(file_id+1).arg(vis_dirs.size()));
        progress.setWindowModality(Qt::WindowModal);

        //  cout<<txt_name.toStdString()<<endl;
        // load vis from each file
        QFile file(vis_dir);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

        QTextStream in(&file);
        int line_num = 0;
        int counter = 0;

        while (!in.atEnd())
        {
            QString line = in.readLine();
            QStringList fields = line.split(" ");
            if (line_num == 1)
            {
                uint points_num = fields.takeFirst().toUInt();
                progress.setRange(0, points_num);
                //vis.resize(points_num);
            }
            if (line_num > 1)
            {
                if (line.startsWith("PATCH"))
                {
                    counter = 0;
                    continue;
                }
                if (counter == 5){

                    while (fields.size() != 0)
                    {
                        tmp.append(fields.takeFirst().toUInt());
                    }
                    tmp.pop_back();
                    sub_vis.append(tmp);
                }
                counter++;
            }

            // if progress dialog is canceled ....
            progress.setValue(sub_vis.size());
            qApp->processEvents();
            if (progress.wasCanceled())
            {
                all_vis.clear();
                return false;
            }

            line_num++;
        }

        // collect all the visibility data
        foreach(QVector<uint> vis, sub_vis)
        {
            all_vis.append(vis);
        }

        file_id++;
    }

    if(all_vis.size()!= vertices.size())
    {
        QMessageBox:: warning(0, tr("Error"), tr("Number of Visibility Colud not Match Vertices!"));
        return false;
    }

    int pt_id = 0;
    foreach(QVector<uint> vis, all_vis)
    {
        foreach(uint id, vis)
        {
            vertices[pt_id].vis.append(id);
        }
        pt_id ++;
    }

}
