#include "EdgeModel.h"

//=======================================================================================//

EdgeModel::EdgeModel() {}

//=======================================================================================//

EdgeModel::~EdgeModel() {}

//=======================================================================================//

void EdgeModel::createOBJ(int frame_nr) {
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


