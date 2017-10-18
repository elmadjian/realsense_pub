#ifndef MARKERDETECTOR_H
#define MARKERDETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <map>

#include <iostream>

using namespace cv;
using namespace std;

class MarkerDetector
{
public:
    vector<Mat> markers;
    vector<vector<Point>> contoursF;
    Point centroidF;
    bool detected;

    MarkerDetector();
    ~MarkerDetector();
    Mat detect(string code, Mat img);

private:
    Mat preprocess(Mat img);
    vector<vector<Point>> findMarkerContours(Mat img);
    vector<vector<Point>> findCandidates(vector<vector<Point>> contours);
    bool isMinLength(vector<Point> approx, double minLength) ;
    void transformMarker(vector<vector<Point>> candidates, Mat img);
    bool getMarkerCode(string code, Mat img);
    int matSum(Mat mat);
    Mat makeItOne(Mat mat, int thresh);
    bool checkCode(string code, Mat matrix);
    string rotate90(string code);
    Point findCentroid(vector<Point> points);
};

#endif // MARKERDETECTOR_H
