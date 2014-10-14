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
}
