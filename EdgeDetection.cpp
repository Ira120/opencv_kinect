#include "EdgeDetection.h"

#include <iostream>

EdgeDetection::EdgeDetection()
{

}

EdgeDetection::~EdgeDetection()
{

}

Mat EdgeDetection::smoothGauss (Mat image, int value){
    Mat imageGauss;
    imageGauss = image.clone();
    GaussianBlur(image, imageGauss, Size (value, value), 0, 0);
    cout<<"...gaussian blur..."<<endl;
    return imageGauss;
}

Mat EdgeDetection::smoothMedian (Mat image, int value){
    Mat imageMedian;
    medianBlur(image,imageMedian,value);
    cout<<"...median blur..."<<endl;
    return imageMedian;
}

Mat EdgeDetection::smoothBilateral (Mat image, int value){
    Mat imageBilateral;
    imageBilateral = image.clone();
    bilateralFilter(image, imageBilateral, value, value*2, value/2);
    cout<<"...bilateral blur..."<<endl;
    return imageBilateral;
}

vector<Vec4i> EdgeDetection::applyHoughTransformation(Mat imageOriginal, int frame_nr){

    Mat smooth;
    smooth = smoothGauss(imageOriginal,15);
    QString filename_lines = "/Users/irina/Develop/workspace/bachelor_1/hough_lines.txt";
    QFile file (filename_lines);

    Canny(smooth, imageCanny, 50, 200, 3);

    HoughLinesP(imageCanny, lines, 1, CV_PI/180, 50, 50, 10 );
    if (file.open(QIODevice::ReadWrite))
    {
        QTextStream stream (&file);
        stream << "Detected hough lines pro frame" << endl;
        stream << "===========================================" << endl;
        stream << "Found about "<<lines.size()<<" lines with hough"<<endl;
        stream << "===========================================" << endl;

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( imageOriginal, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
        stream << i+1 << "start_pixel ( " << l.val[0] << "|" << l.val[1] << " ) --------> end_pixel ( " << l[2] << "|" << l[3] << " )" <<  endl;
    }

    file.close();
    }
    imshow("Detected lines with hough",imageOriginal);
    sprintf(filename,"/Users/irina/Develop/workspace/bachelor_1/hough_lines%.2d.jpg",frame_nr++);
    imwrite(filename, imageOriginal);
    cout << "Saved " << filename << endl;

    cout<<"...hough lines..."<<endl;
    return lines;
}

vector <Vec4i> EdgeDetection::applyLSD(Mat imageOriginal, int frame_nr){

     // TODO: detect segments with start and end ponits with LSD

     Ptr<LineSegmentDetector> ls;

     Mat smooth;
     smooth = smoothGauss(imageOriginal,35);
   //  smooth = smoothBilateral(imageOriginal,85);

     cvtColor(smooth,imageGray,COLOR_BGR2GRAY);

     ls = createLineSegmentDetector(LSD_REFINE_STD);
     // Detect the lines
     ls->detect(imageGray, lines);

     cout<<"Found about "<<lines.size()<<" lines with LSD"<<endl;

     Mat drawnLines(imageGray);
     ls->drawSegments(drawnLines, lines);
     imshow("Detected segment with LSD", drawnLines);

     sprintf(filename,"/Users/irina/Develop/workspace/bachelor_1/lsd_lines%.2d.jpg",frame_nr++);
     imwrite(filename, drawnLines);
     cout << "Saved " << filename << endl;

     return lines;
}
