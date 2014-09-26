#include "Application.h"

Application::Application()
{
    frame_nr = 0;
    detector = EdgeDetection::EdgeDetection();
    projection = BackProjection::BackProjection();
}

Application::~Application(){


}

int Application::initKinect(){

    int return_value;

    VideoCapture capture(CAP_OPENNI);


    if (!capture.isOpened()) {
        cerr << "Failed to open the video device, video file or image sequence!\n" << endl;
        return 1;
    }

    else {


        return_value = Application::frameLoop(capture);

        return return_value;
    }
}

void Application::showModel()
{

}


int Application::frameLoop(VideoCapture capture){

    Mat rgbImage, depthMap;

       vector<Vec4i> lines_hough_frame;
       /*
        vector<Vec4i> lines_lsd_frame;
        vector<vector<Vec4i> > lines_hogh;
        vector<vector<Vec4i> > lines_lsd;
        int i;
        Vec4i xy;
        float z;
        Point2f nowyj;
        */

    for (;;) {
            capture.grab();
            capture.retrieve(rgbImage,CAP_OPENNI_BGR_IMAGE);
            capture.retrieve(depthMap,CAP_OPENNI_DEPTH_MAP);
            capture.set( CAP_PROP_OPENNI_REGISTRATION , 0);
            if (rgbImage.empty() | depthMap.empty())
                break;

            imshow("show RGB input", rgbImage);
            char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input

            switch (key) {
                case 27: //escape key
                    return 0;
                case ' ': //Save an image


                // TODO: nach Tasten Auswahl treffen!!!


                    frame_nr += 1;

                    lines_hough_frame = detector.applyHoughTransformation(rgbImage,frame_nr);
                    projection.calculateBackProejction(lines_hough_frame,depthMap);

                    break;

                case 'h': //Hough
                    frame_nr +=1;


                default:
                    break;
            }
        }

    return 0;
}
