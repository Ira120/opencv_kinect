#include "GroupLines3D.h"

//=======================================================================================//

GroupLines3D::GroupLines3D() {}

//=======================================================================================//

GroupLines3D::~GroupLines3D() {}

//=======================================================================================//

float GroupLines3D::cosineSimilarity(Line3D first, Line3D second) {
    //https://en.wikipedia.org/wiki/Cosine_similarity
    /*
     * Cosine similarity is a measure of similarity between two vectors
     * of an inner product space that measures the cosine of the angle
     * between them. The cosine of 0° is 1, and it is less than 1 for any
     * other angle.
    */

    float similarity;
    float skalar;
    float betrag_first;
    float betrag_second;
    Vec3f temp_first;
    Vec3f temp_second;

    temp_first.val[0] = first.getEndPointOfLine3D().x - first.getStartPointOfLine3D().x;
    temp_first.val[1] = first.getEndPointOfLine3D().y - first.getStartPointOfLine3D().y;
    temp_first.val[2] = first.getEndPointOfLine3D().z - first.getStartPointOfLine3D().z;

    temp_second.val[0] = second.getEndPointOfLine3D().x - second.getStartPointOfLine3D().x;
    temp_second.val[1] = second.getEndPointOfLine3D().y - second.getStartPointOfLine3D().y;
    temp_second.val[2] = second.getEndPointOfLine3D().z - second.getStartPointOfLine3D().z;

    skalar = temp_first.dot(temp_second);

    betrag_first = (float)sqrt(pow(temp_first.val[0],2) + pow(temp_first.val[1],2) + pow(temp_first.val[2],2));
    betrag_second = (float)sqrt(pow(temp_second.val[0],2) + pow(temp_second.val[1],2) + pow(temp_second.val[2],2));

    similarity = skalar / (betrag_first * betrag_second);

    return similarity;
}

//=======================================================================================//

float GroupLines3D::euclideanDistance(Line3D first, Line3D second) {
    //Kochbuch C++ (Kapitel 11.12 - Den Abstand zwischen zwei Vektoren berechnen; S.447)
    float distance;
    std::vector<float> temp_first;
    std::vector<float> temp_second;

    temp_first.push_back((first.getEndPointOfLine3D().x + first.getStartPointOfLine3D().x)/2.0f);
    temp_first.push_back((first.getEndPointOfLine3D().y + first.getStartPointOfLine3D().y)/2.0f);
    temp_first.push_back((first.getEndPointOfLine3D().z + first.getStartPointOfLine3D().z)/2.0f);

    temp_second.push_back((second.getEndPointOfLine3D().x + second.getStartPointOfLine3D().x)/2.0f);
    temp_second.push_back((second.getEndPointOfLine3D().y + second.getStartPointOfLine3D().y)/2.0f);
    temp_second.push_back((second.getEndPointOfLine3D().z + second.getStartPointOfLine3D().z)/2.0f);

    distance = (float)vectorDistance(temp_first.begin(), temp_first.end(), temp_second.begin());

    return distance;

}

//=======================================================================================//

int GroupLines3D::findSimilarLines(vector<vector<Line3D> > lines_vector, int frame_nr) {
    log = SSTR("[DEBUG]: ...GROUP 3DLINES...\n");
    Log(log);

    vector<Line3D> temp_lines_vector;
    Line3D temp_line3D;
    vector<SimilarityMeasure> euklead_vector;
    vector<Line3D> cosine_vector;

    //create one 1D-vector with all detected 3DLines in the object
    for (int i=0; i<(int)lines_vector.size();i++){
        for (int j=0; j<(int)lines_vector.at(i).size();j++){
            temp_lines_vector.push_back(lines_vector.at(i).at(j));
        }
    }

    //if no 3DLines found
    if (temp_lines_vector.empty()){
        log = SSTR("[DEBUG]: ...vector with 3DLines was empty!...\n");
        Log(log);
        return 0;
    }

    //if the 'global' vector includes just one 3Dline
    else if (temp_lines_vector.size() == 1) {
        log = SSTR("[DEBUG]: ...vector includes just 1 3DLine...\n");
        Log(log);

        //take the only 3DLine...
        temp_line3D = temp_lines_vector.at(0);

        //...and fill final vector with this line
        cosine_vector.push_back(temp_line3D);

        final_lines_vector.push_back(cosine_vector);
        log = SSTR("[DEBUG]: ...push back 3DLine to final vector...\n");
        Log(log);

        EdgeModel::EdgeModel model;
        model.createOBJfinal(final_lines_vector);
        cout << "... create GROUPED .obj-File with one edge..."<< endl;

        return 0;
    }

    //if the 'global' vector includes more than one 3Dline
    else {
        log = SSTR("[DEBUG]: ...GROUP EDGES...\n");
        Log(log);

        //while 'global' vector includes 3DLines do the following
        while (!temp_lines_vector.empty()) {

            log = SSTR("[DEBUG]: ........................\n");
            Log(log);

            //used vector should be empty
            euklead_vector.clear();
            cosine_vector.clear();

            //take the first 3DLine in 'gloabal' vector...
            temp_line3D = temp_lines_vector.at(0);

            //...and fill the final vector with this 3DLine
            cosine_vector.push_back(temp_line3D);

            //if just one 3DLine is left
            if (temp_lines_vector.size()==1) {
                log = SSTR("[DEBUG]: 'global' vector includes just one edge -> reject it!\n");
                Log(log);

                EdgeModel::EdgeModel model;
                model.createOBJfinal(final_lines_vector);
                model.createOBJfinal(final_lines_vector);
                log = SSTR("[DEBUG]: ... create GROUPED .obj-File with: " << final_lines_vector.size() << " edges..."<< endl);
                Log(log);

                groupSimilarLines(frame_nr);

                return 0;
            }
            else {

                log = SSTR("[DEBUG]: ...there are " << temp_lines_vector.size() << " left..." << endl);
                Log(log);

                //compare the first edge in 'global' vector with the rest edges - euclidean distance
                for (int i=1 ; i<(int)temp_lines_vector.size(); i++) {
                    float dist;

                    dist = euclideanDistance(temp_line3D,temp_lines_vector.at(i));

                    //Schwellwert für Euklidische Distanz festlegen
                    //falls die Distanz unter dem Schwellwert ist, dann das Linien-Paar weiter übernehmen
                    if (dist < 30.0f) {
                        SimilarityMeasure::SimilarityMeasure simMeas_eukl(temp_line3D,0,temp_lines_vector.at(i),i,dist);
                        euklead_vector.push_back(simMeas_eukl);

                        log = SSTR(dist <<"[DEBUG]: ...edge found with minor euclidean distance at nr.: "<< i << endl);
                        Log(log);
                    } else {

                        log = SSTR(dist <<"[DEBUG]: ...edge NOT found with minor euclidean distance at nr.: "<< i << endl);
                        Log(log);
                    }

                }

                //if some edges with minor euclidean distance were found
                if (!euklead_vector.empty()) {
                    int num=0;

                    //für alle Kanten mit geringer eukl. Distanz eine ähnliche Kante nach Kosinus-Maß suchen
                    for (int i=0; i<(int)euklead_vector.size();i++) {

                        float sim;

                        sim = cosineSimilarity(temp_line3D,euklead_vector.at(i).second);

                        //Schwellwert für Kosinus-Ähnlichkeit festlegen
                        if (fabsf(sim) > 0.9f) {
                            //store similar edge into final vector
                            cosine_vector.push_back(euklead_vector.at(i).second);

                            log = SSTR(sim <<"[DEBUG]: ...edge found with high cosine similarity at x: "<< euklead_vector.at(i).second.getStartPointOfLine3D().x << endl);
                            Log(log);

                            //delete similar egde from 'global' vector
                            temp_lines_vector.erase(temp_lines_vector.begin()+euklead_vector.at(i).i_second-num);
                            num++;

                        } else {
                            cout << "Keine Kosinus-Ähnlichkeit" << endl;


                            // BUG: wenn bei 2 Kanten die vorherige keine kosinus-Ähnlichkeit aufweist, dann wird sie nochmals geprüft
                            // und zwei Kanten werden einfach enfernt, ohnde dass die zweite auch auf die Ähnlichkeit geprüft wird
                            if (euklead_vector.size()==2) {
                                cout << "Fall mit 2 Kanten?" << endl;

                                cout << "i: " << i << endl;
                                sim = cosineSimilarity(temp_line3D,euklead_vector.at(i).second);
                                cout << "sim: " << sim << endl;
                                //Schwellwert für Kosinus-Ähnlichkeit festlegen
                                if (fabsf(sim) > 0.9f) {
                                    cosine_vector.push_back(euklead_vector.at(i).second);

                                    cout << sim << ": eine ähnliche Kante nach Kosinus-Maß im 2er Fall gefunden bei " << euklead_vector.at(i).second.getStartPointOfLine3D().x <<endl;

                                    temp_lines_vector.erase(temp_lines_vector.begin()+euklead_vector.at(i).i_second-num);
                                    num++;
                                    cout <<"ich lösche danach - num: " << num <<endl;
                                }

                                euklead_vector.clear();
                                cout << "Fall mit 2 Kanten erledigt" << endl;

                            }
                        }
                    }
                    final_lines_vector.push_back(cosine_vector);
                    cout << "füge eine Kante in den Hauptvector\n";
                    temp_lines_vector.erase(temp_lines_vector.begin()+0);
                } else {

                    log = SSTR("[DEBUG]: ...no edges with minor euclidean distance were found! -> delete the first line in 'gloabal' vector..."<< endl);
                    Log(log);

                    temp_lines_vector.erase(temp_lines_vector.begin()+0);
                }
            }
        }
    }
    EdgeModel::EdgeModel model;
    model.createOBJfinal(final_lines_vector);

    groupSimilarLines(frame_nr);
    return 0;
}

//=======================================================================================//

int GroupLines3D::groupSimilarLines(int frame_nr) {

    for (int i=0; i<(int)final_lines_vector.size(); i++) {
        float x_sum_start=0;
        float y_sum_start=0;
        float z_sum_start=0;

        float x_sum_end=0;
        float y_sum_end=0;
        float z_sum_end=0;
        Line3D temp;
        int size = final_lines_vector.at(i).size();

        if ((int)final_lines_vector.at(i).size() >= (int)frame_nr*(1/4)) {
        for (int j=0; j<(int)final_lines_vector.at(i).size(); j++) {

            x_sum_start += final_lines_vector.at(i).at(j).getStartPointOfLine3D().x;
            y_sum_start += final_lines_vector.at(i).at(j).getStartPointOfLine3D().y;
            z_sum_start += final_lines_vector.at(i).at(j).getStartPointOfLine3D().z;

            x_sum_end += final_lines_vector.at(i).at(j).getEndPointOfLine3D().x;
            y_sum_end += final_lines_vector.at(i).at(j).getEndPointOfLine3D().y;
            z_sum_end += final_lines_vector.at(i).at(j).getEndPointOfLine3D().z;
        }

        temp.storeLine3D(x_sum_start/(float)size,   y_sum_start/(float)size,    z_sum_start/(float)size,
                         x_sum_end/(float)size,     y_sum_end/(float)size,      z_sum_end/(float)size);
        }

        groped_lines.push_back(temp);
    }

    EdgeModel::EdgeModel model;
    model.createOBJgrouped(groped_lines);
    cout << "CREATE final OBJ" << endl;

    return 1;

}

