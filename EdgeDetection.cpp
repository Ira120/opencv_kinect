#include "EdgeDetection.h"

//=======================================================================================//

EdgeDetection::EdgeDetection() {}

//=======================================================================================//

EdgeDetection::~EdgeDetection() {}

//=======================================================================================//

Mat EdgeDetection::smoothGauss (Mat image, int value) {

    //mask size 'dynamic'
    value = (value/2)*2+1;

    Mat imageGauss;
    imageGauss = image.clone();
    GaussianBlur(image, imageGauss, Size (value, value), 0, 0);

    log = SSTR("[DEBUG]: ...gaussian blur... with mask size: " << value << endl);
    Log(log);

    return imageGauss;
}

//=======================================================================================//

Mat EdgeDetection::smoothMedian (Mat image, int value) {

    //mask size 'dynamic'
    value = (value/2)*2+1;

    Mat imageMedian;
    medianBlur(image,imageMedian,value);

    log = SSTR("[DEBUG]: ....median blur... with mask size: " << value << endl);
    Log(log);

    return imageMedian;
}

//=======================================================================================//

Mat EdgeDetection::smoothBilateral (Mat image, int value) {

    //mask size 'dynamic'
    value = (value/2)*2+1;

    Mat imageBilateral;
    imageBilateral = image.clone();
    bilateralFilter(image, imageBilateral, value, value*2, value/2);

    log = SSTR("[DEBUG]: ....bilateral blur... with mask size: " << value << endl);
    Log(log);

    return imageBilateral;
}

//=======================================================================================//

vector<Vec4i> EdgeDetection::applyHoughTransformation(Mat imageOriginal, int frame_nr) {
    lines.clear();

    cout<<"breite: "<<imageOriginal.cols<<", höhe: "<<imageOriginal.rows<<endl;
    //10px smaller for search algo in 'BackProjection::findZInDepthMap'
    Rect mask(10,10,545,397);
    Mat rgbROI = imageOriginal(mask);

    Mat smooth;
    smooth = smoothBilateral(rgbROI,5);

    char filename_lines[200];
    sprintf(filename_lines,"/Users/irina/Develop/workspace/bachelor_1/data/hough_lines%.2d.txt",frame_nr);
    QFile file (filename_lines);

    Canny(smooth, imageCanny, 50, 200, 3);

    HoughLinesP(imageCanny, lines, 1, CV_PI/180, 20, 20, 5);

    //write an output file
    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream (&file);
        stream << "Detected hough lines pro frame" << endl;
        stream << "===========================================" << endl;
        stream << "Found about "<<lines.size()<<" lines with hough"<<endl;
        stream << "===========================================" << endl;

    for(size_t i = 0; i < lines.size(); i++) {

        lines[i].val[0] += 10;
        lines[i].val[1] += 10;
        lines[i].val[2] += 10;
        lines[i].val[3] += 10;

        Vec4i l = lines[i];

        line(imageOriginal, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 1, LINE_AA);
        stream << i+1 << ") start_pixel ( " << l.val[0] << "|" << l.val[1] << " ) --------> end_pixel ( " << l[2] << "|" << l[3] << " )" <<  endl;
    }

    file.close();
    }

    //show and save dected lines per frame
    imshow("Detected lines with hough",imageOriginal);
    sprintf(filename,"/Users/irina/Develop/workspace/bachelor_1/data/hough_lines%.2d.jpg",frame_nr++);
    imwrite(filename,imageOriginal);

    log = SSTR("========================================================\n"
               << "[DEBUG]: Saved " << filename << endl
               << "========================================================\n");
    Log(log);

    log = SSTR("[DEBUG]: ...hough lines detected...\n");
    Log(log);

    return lines;
}

//=======================================================================================//
