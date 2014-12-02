#include "Tools.h"

//=======================================================================================//

namespace Tools {

    int saveFramesFromKinect(VideoCapture &capture){

        int n = 0;
        char filename[200];
        string window_name = "video | q or esc to quit";
        cout << "press space to save a picture. q or esc to quit" << endl;
        namedWindow(window_name, WINDOW_KEEPRATIO); //resizable window;
        Mat frame;

        for (;;) {
                capture.grab();
                capture.retrieve(frame,CAP_OPENNI_BGR_IMAGE);
                if (frame.empty())
                    break;

                imshow(window_name, frame);
                char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input

                switch (key) {
                    case 'q':
                    case 'Q':
                    case 27: //escape key
                        return 0;
                    case ' ': //Save an image
                        sprintf(filename,"/Users/irina/Develop/workspace/bachelor_1/calibrate_image%.2d.jpg",n++);
                        imwrite(filename,frame);
                        cout << "Saved " << filename << endl;
                        break;
                    default:
                        break;
                }
            }
     return 0;
    }

    //=======================================================================================//

    float modalValue(vector<float> values) {
        //finde Modalwert in der nähren Nachbarschaft des "fehlerhaften" Pixels
        sort(values.begin(),values.end());
        Mat valuesMat= Mat::zeros(2,values.size(),CV_32F);
        int matCounter=0;

        for(int i=0;i<values.size();i++) {
            if(i==0){
                valuesMat.at<float>(0,0)=values.at(0);
                valuesMat.at<float>(1,0)=1;
            }
            else {
                if(values.at(i)==values.at(i-1)) {
                    valuesMat.at<float>(1,matCounter) ++;
                } else {
                    matCounter++;
                    valuesMat.at<float>(0,matCounter) = values.at(i);
                    valuesMat.at<float>(1,matCounter) = 1;
                }
            }
        }

        float result = valuesMat.at<float>(0,0);
        float compare = valuesMat.at<float>(1,0);
        for(int u=1; u<valuesMat.cols; u++) {
            if(valuesMat.at<float>(1,u) > compare) {
                result = valuesMat.at<float>(0,u);
                compare = valuesMat.at<float>(1,u);
            }
        }

        return result;
    }
}
