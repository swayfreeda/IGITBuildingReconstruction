#include "sw_plyio.h"

#include <QFile>
#include <QTextStream>
#include <QProgressDialog>
#include<QApplication>
#include<QMessageBox>

//-------------------------------------------loadPointsFromPLY----------------------------//
bool SW::PLYIO::loadPointsFromPLY(QVector<Point> & vertices)
{

    QFile file(file_name_);
    if(!file.open(QIODevice::ReadOnly))
    {
        emit statusBar("Fail to load points from " + file_name_);
        return false;
    }

    // create a progress dialog
    QProgressDialog progress;
    progress.setLabelText(tr("Loading dense points..."));
    progress.setWindowModality(Qt::WindowModal);

    bool start_read = false;
    QTextStream in(&file);
    int counter = 0;

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
            vertices.resize(vertex_num);

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
            vertices[counter].x = fields.takeFirst().toFloat();
            vertices[counter].y = fields.takeFirst().toFloat();
            vertices[counter].z = fields.takeFirst().toFloat();

            // normal
            vertices[counter].normal_x = fields.takeFirst().toFloat();
            vertices[counter].normal_y = fields.takeFirst().toFloat();
            vertices[counter].normal_z = fields.takeFirst().toFloat();

            // color
            vertices[counter].r = fields.takeFirst().toFloat();
            vertices[counter].g = fields.takeFirst().toFloat();
            vertices[counter].b = fields.takeFirst().toFloat();

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
    return true;
}

//--------------------------------------------savePointsToPLY-----------------------------//

bool SW::PLYIO::savePointsToPLY(QVector<Point> & vertices)
{
    QFile file(file_name_);
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
