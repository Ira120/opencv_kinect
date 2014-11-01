#include "EdgeModel.h"

//=======================================================================================//

EdgeModel::EdgeModel() {}

//=======================================================================================//

EdgeModel::~EdgeModel() {}

//=======================================================================================//

void EdgeModel::createOBJproFrame(int frame_nr) {
    Line3D line3D = Line3D::Line3D();

    char filename_lines[200];
    sprintf(filename_lines,"/Users/irina/Develop/workspace/bachelor_1/data/3Dlines%.2d.obj",frame_nr++);
    QFile file (filename_lines);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream (&file);
        stream << "# 3Dlines pro frame" << endl;
        stream << "# OBJ File: '3Dlines.blend'" << endl;
        stream << "# www.blender.org" << endl;
        stream << "o EdgeModel" << endl;

        //in blender -> mit 1 = Front Ortho
        for (int i=0; i<(int)lines3DproFrame.size();i++) {

            line3D = lines3DproFrame.at(i);
            stream << "v " << line3D.getStartPointOfLine3D().x << " " << line3D.getStartPointOfLine3D().y << " " << line3D.getStartPointOfLine3D().z << endl;
            stream << "v " << line3D.getEndPointOfLine3D().x << " " << line3D.getEndPointOfLine3D().y << " " << line3D.getEndPointOfLine3D().z << endl;
        }

        for (int i=0; i<(int)lines3DproFrame.size();i++) {

            stream << "l " << i*2+1 << " " << i*2+2 << endl;
        }
        file.close();
    }

    log = SSTR("[DEBUG]: ...OBJ file is created...\n");
    Log(log);

}

//=======================================================================================//

void EdgeModel::createOBJfinal() {
    Line3D line3D = Line3D::Line3D();

    char filename_lines[200];
    sprintf(filename_lines,"/Users/irina/Develop/workspace/bachelor_1/data/3Dlines_final.obj");
    QFile file (filename_lines);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream (&file);
        stream << "# 3Dlines pro frame" << endl;
        stream << "# OBJ File: '3Dlines.blend'" << endl;
        stream << "# www.blender.org" << endl;
        stream << "o EdgeModel" << endl;

        //in blender -> mit 1 = Front Ortho
        for (int i=0; i<(int)line3Dall.size();i++) {
            for (int j=0; j<(int)line3Dall.at(i).size();j++){

            line3D = line3Dall.at(i).at(j);
            stream << "v " << line3D.getStartPointOfLine3D().x << " " << line3D.getStartPointOfLine3D().y << " " << line3D.getStartPointOfLine3D().z << endl;
            stream << "v " << line3D.getEndPointOfLine3D().x << " " << line3D.getEndPointOfLine3D().y << " " << line3D.getEndPointOfLine3D().z << endl;
        }
        }

        //for l enumeration in .obj
        int tempVal = 0;
        int *pTempVal;
        pTempVal = &tempVal;
        for (int i=0; i<(int)line3Dall.size();i++) {
            for (int j=0; j<(int)line3Dall.at(i).size();j++){
                //stream << "l " << j*2+1 << " " << j*2+2 << endl;
                *pTempVal+=1;
            }
        }
        for (int i=1; i<=tempVal*2; i+=2){
            stream << "l " << i << " " << i+1 << endl;
        }
        file.close();
    }

    log = SSTR("[DEBUG]: ...OBJ file is created...\n");
    Log(log);
}

//=======================================================================================//

void EdgeModel::createOBJfinal(vector<vector<Line3D> > lines_vector) {
    Line3D line3D = Line3D::Line3D();

    char filename_lines[200];
    sprintf(filename_lines,"/Users/irina/Develop/workspace/bachelor_1/data/3Dlines_group.obj");
    QFile file (filename_lines);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream (&file);
        stream << "# 3Dlines pro frame" << endl;
        stream << "# OBJ File: '3Dlines.blend'" << endl;
        stream << "# www.blender.org" << endl;
        stream << "o EdgeModel" << endl;

        //in blender -> mit 1 = Front Ortho
        for (int i=0; i<(int)lines_vector.size();i++) {
            for (int j=0; j<(int)lines_vector.at(i).size();j++){

            line3D = lines_vector.at(i).at(j);
            stream << "v " << line3D.getStartPointOfLine3D().x << " " << line3D.getStartPointOfLine3D().y << " " << line3D.getStartPointOfLine3D().z << endl;
            stream << "v " << line3D.getEndPointOfLine3D().x << " " << line3D.getEndPointOfLine3D().y << " " << line3D.getEndPointOfLine3D().z << endl;
        }
        }

        //for l enumeration in .obj
        int tempVal = 0;
        int *pTempVal;
        pTempVal = &tempVal;
        for (int i=0; i<(int)lines_vector.size();i++) {
            for (int j=0; j<(int)lines_vector.at(i).size();j++){
                //stream << "l " << j*2+1 << " " << j*2+2 << endl;
                *pTempVal+=1;
            }
        }
        for (int i=1; i<=tempVal*2; i+=2){
            stream << "l " << i << " " << i+1 << endl;
        }
        file.close();
    }

    log = SSTR("[DEBUG]: ...OBJ file is created...\n");
    Log(log);
}

//=======================================================================================//

void EdgeModel::createOBJgrouped(vector<Line3D> grouped_lines) {
    Line3D line3D = Line3D::Line3D();

    char filename_lines[200];
    sprintf(filename_lines,"/Users/irina/Develop/workspace/bachelor_1/data/3Dlines_fertig.obj");
    QFile file (filename_lines);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream (&file);
        stream << "# 3Dlines pro frame" << endl;
        stream << "# OBJ File: '3Dlines.blend'" << endl;
        stream << "# www.blender.org" << endl;
        stream << "o EdgeModel" << endl;

        //in blender -> mit 1 = Front Ortho
        for (int i=0; i<(int)grouped_lines.size();i++) {

            line3D = grouped_lines.at(i);
            stream << "v " << line3D.getStartPointOfLine3D().x << " " << line3D.getStartPointOfLine3D().y << " " << line3D.getStartPointOfLine3D().z << endl;
            stream << "v " << line3D.getEndPointOfLine3D().x << " " << line3D.getEndPointOfLine3D().y << " " << line3D.getEndPointOfLine3D().z << endl;
        }

        for (int i=0; i<(int)grouped_lines.size();i++) {

            stream << "l " << i*2+1 << " " << i*2+2 << endl;
        }
        file.close();
    }

    log = SSTR("[DEBUG]: ...OBJ file is created...\n");
    Log(log);

}
