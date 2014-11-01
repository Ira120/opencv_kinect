#include "GroupLines3D.h"

//=======================================================================================//

GroupLines3D::GroupLines3D() {}

//=======================================================================================//

GroupLines3D::~GroupLines3D() {}

//=======================================================================================//

float GroupLines3D::cosineSimilarity(Line3D first, Line3D second) {
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

int GroupLines3D::findSimilarLines(vector<vector<Line3D> > lines_vector) {
    vector<Line3D> temp_lines_vector;
    Line3D temp_line3D;
    vector<SimilarityMeasure> euklead_vector;
    vector<Line3D> cosine_vector;

    //aus mehrdimensionalen vector einen dimensionalen mit allen gefundenen Kanten erstellen
    for (int i=0; i<(int)lines_vector.size();i++){
        for (int j=0; j<(int)lines_vector.at(i).size();j++){
            temp_lines_vector.push_back(lines_vector.at(i).at(j));
        }
    }

    //Absicherung, wenn keine 3DLinien für das Model gefunden wurden
    if (temp_lines_vector.empty()){
        cout << "Vector mit Linien ist leer!" << endl;
        return 0;
    }
    else {

        cout << "...Gruppiere gefunden Kanten..." << endl;

        //solange vector mit 3DLinien befüllt ist
        while (!temp_lines_vector.empty()) {

            cout << "Neue Gruppierung" <<endl;
            //Absicherung, dass die verwendeten vectoren leer sind
            euklead_vector.clear();
            cosine_vector.clear();

            //nehme die erste Linie aus dem vector raus
            temp_line3D = temp_lines_vector.at(0);

            //vector ist immer mit einer der ersten Linie befüllt
            cosine_vector.push_back(temp_line3D);

            //Absicherung, wenn nur eine Linie insgesamt gefunden wurde bzw. geblieben ist
            if (temp_lines_vector.size()==1) {
                cout << "Vektor hat nur (noch) eine Line3D" << endl;

                final_lines_vector.push_back(cosine_vector);

                EdgeModel::EdgeModel model;
                model.createOBJfinal(final_lines_vector);
                 cout << "tu was mit: " << final_lines_vector.size() << endl;

                groupSimilarLines();

                return 0;
            }
            else {

                cout << "...Es sind " << temp_lines_vector.size() << " Kanten im Model..." << endl;

                //vergleiche die erste Kante mit allen anderen Kanten im vector auf die Euklidische Distanz
                for (int i=1 ; i<(int)temp_lines_vector.size(); i++) {
                    float dist;

                    dist = euclideanDistance(temp_line3D,temp_lines_vector.at(i));

                    cout << i << ") Distanz: " << dist << ". Zwischen " << temp_line3D.getStartPointOfLine3D().x << " und  " << temp_lines_vector.at(i).getStartPointOfLine3D().x << endl;

                    //Schwellwert für Euklidische Distanz festlegen
                    //falls die Distanz unter dem Schwellwert ist, dann das Linien-Paar weiter übernehmen
                    if (dist < 70.0f) {
                        SimilarityMeasure::SimilarityMeasure simMeas_eukl (temp_line3D,0,temp_lines_vector.at(i),i,dist);
                        euklead_vector.push_back(simMeas_eukl);

                        cout << dist << ": eine Kante mit geringer euklid. Distanz gefunden" << "und nr.: " << i << endl;
                    }

                }
                    //wenn Kanten mit geringer Euklidischer Distanz gefunden wurden
                    if (!euklead_vector.empty()) {

                        int del=0;
                        //für alle Kanten mit geringer eukl. Distanz eine ähnliche Kante nach Kosinus-Maß suchen
                        for (int i=0; i<(int)euklead_vector.size();i++) {

                            float sim;

                            sim = cosineSimilarity(temp_line3D,euklead_vector.at(i).second);

                            //Schwellwert für Kosinus-Ähnlichkeit festlegen
                            if (fabsf(sim) > 0.8f) {
                                cosine_vector.push_back(euklead_vector.at(i).second);

                                cout << sim << ": eine ähnliche Kante nach Kosinus-Maß gefunden bei " << euklead_vector.at(i).second.getStartPointOfLine3D().x <<endl;

                                temp_lines_vector.erase(temp_lines_vector.begin()+euklead_vector.at(i).i_second-del);
                                del++;
                                  cout <<"ich lösche danach - del: " << del <<endl;


                            }
                            else {
                                cout << "Keine Kosinus-Ähnlichkeit" << endl;
                                //final_lines_vector.push_back(cosine_vector);


                                // BUG: wenn bei 2 Kanten die vorherige keine kosinus-Ähnlichkeit aufweist, dann wird sie nochmals geprüft
                                // und zwei Kanten werden einfach enfernt, ohnde dass die zweite auch auf die Ähnlichkeit geprüft wird
                                if (euklead_vector.size()==2) {
                                    cout << "Fall mit 2 Kanten?" << endl;

                                    cout << "i: " << i << endl;
                                    sim = cosineSimilarity(temp_line3D,euklead_vector.at(i).second);
                                    cout << "sim: " << sim << endl;
                                    //Schwellwert für Kosinus-Ähnlichkeit festlegen
                                    if (fabsf(sim) > 0.8f) {
                                        cosine_vector.push_back(euklead_vector.at(i).second);

                                        cout << sim << ": eine ähnliche Kante nach Kosinus-Maß im 2er Fall gefunden bei " << euklead_vector.at(i).second.getStartPointOfLine3D().x <<endl;

                                        temp_lines_vector.erase(temp_lines_vector.begin()+euklead_vector.at(i).i_second-del);
                                                                          del++;
                                          cout <<"ich lösche danach - del: " << del <<endl;
                                    }

                                    euklead_vector.clear();
                                    cout << "Fall mit 2 Kanten erledigt" << endl;

                                }
                            }
                        }
                    final_lines_vector.push_back(cosine_vector);
                    temp_lines_vector.erase(temp_lines_vector.begin()+0);
                    }
                    else {

                        cout  << "keine ähnliche Kante nach Kosinus-Maß gefunden" << endl;

                        final_lines_vector.push_back(cosine_vector);
                        temp_lines_vector.erase(temp_lines_vector.begin()+0);
                    }
                }
            }
        }
    EdgeModel::EdgeModel model;
    model.createOBJfinal(final_lines_vector);
    cout << "tu was mit: " << final_lines_vector.size() << endl;

    groupSimilarLines();
    return 0;
}

//=======================================================================================//

int GroupLines3D::groupSimilarLines() {

    for (int i=0; i<final_lines_vector.size(); i++) {
        float x_sum_start=0;
        float y_sum_start=0;
        float z_sum_start=0;

        float x_sum_end=0;
        float y_sum_end=0;
        float z_sum_end=0;
        Line3D temp;
        int size = final_lines_vector.at(i).size();

        for (int j=0; j<final_lines_vector.at(i).size(); j++) {

            x_sum_start += final_lines_vector.at(i).at(j).getStartPointOfLine3D().x;
            y_sum_start += final_lines_vector.at(i).at(j).getStartPointOfLine3D().y;
            z_sum_start += final_lines_vector.at(i).at(j).getStartPointOfLine3D().z;

            x_sum_end += final_lines_vector.at(i).at(j).getEndPointOfLine3D().x;
            y_sum_end += final_lines_vector.at(i).at(j).getEndPointOfLine3D().y;
            z_sum_end += final_lines_vector.at(i).at(j).getEndPointOfLine3D().z;


        }

        temp.storeLine3D(x_sum_start/(float)size,   y_sum_start/(float)size,    z_sum_start/(float)size,
                         x_sum_end/(float)size,     y_sum_end/(float)size,      z_sum_end/(float)size);
        groped_lines.push_back(temp);
    }

    EdgeModel::EdgeModel model;
    model.createOBJgrouped(groped_lines);
    cout << "CREATE" << endl;

}

