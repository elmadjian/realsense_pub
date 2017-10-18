#include "markerdetector.h"

MarkerDetector::MarkerDetector() {

}

MarkerDetector::~MarkerDetector() {

}

Mat MarkerDetector::detect(string code, Mat img) {
    Mat out;
    out = preprocess(img);
    vector<vector<Point>> contours    = findMarkerContours(out);
    vector<vector<Point>> candidates  = findCandidates(contours);
    transformMarker(candidates, img);
    if (getMarkerCode(code, img))
        detected = true;
    else
        detected = false;
    markers.clear();
    contoursF.clear();
    return img;
}

//FUNC: preprocess
//Converts colored img to grayscale and perform an adaptive threshold on it
//======================================
Mat MarkerDetector::preprocess(Mat img) {
    Mat out;
    cvtColor(img, out, COLOR_BGR2GRAY);
    medianBlur(out, out, 5);
    adaptiveThreshold(out, out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 9, 9);
    return out;
}


//FUNC: findMarkerContours
//Find all the external contours on an image and return
//them as a contourList
//=======================================
vector<vector<Point>> MarkerDetector::findMarkerContours(Mat img) {
    vector<vector<Point>> contours, contourList;
    vector<Vec4i> hierarchy;
    findContours(img, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++) {
        if (contours[i].size() > 5)
            contourList.push_back(contours[i]);
    }
    return contourList;
}

//FUNC: findCandidates
//Find possible marker candidates on an image
//========================================
vector<vector<Point>> MarkerDetector::findCandidates(vector<vector<Point>> contours) {
    vector<vector<Point>> possibleMarkers;
    for (int i = 0; i < contours.size(); i++) {
        vector<Point> approx;
        double eps = arcLength(contours[i], true) * 0.05;
        approxPolyDP(contours[i], approx, eps, true);
        if (approx.size() != 4)
            continue;
        if (!isContourConvex(approx))
            continue;
        if (isMinLength(approx, 1000))
            continue;
        Point v1 = approx[1] - approx[0];
        Point v2 = approx[2] - approx[0];
        double lr = (v1.x * v2.y) - (v1.y * v2.x);
        if (lr < 0) {
            Point temp = approx[1];
            approx[1] = approx[3];
            approx[3] = temp;
        }
        possibleMarkers.push_back(approx);
    }
    return possibleMarkers;
}

//FUNC: isMinLength
//Find whether the contour obeys the "min distance rule"
//between points (we're looking for sort of a square, right)
//=========================================
bool MarkerDetector::isMinLength(vector<Point> approx, double minLength) {
    double minDist = INT32_MAX;
    for (int i = 0; i < 4; i++) {
        Point p = approx[i] - approx[(i+1)%4];
        double squaredLength = p.ddot(p);
        minDist = min(minDist, squaredLength);
    }
    if (minDist < minLength)
        return true;
    return false;
}

//FUNC: transformMaker
//Finds the homography of a marker so that you can warp its
//perspective and process its code
//=====================================
void MarkerDetector::transformMarker(vector<vector<Point>> candidates, Mat img) {
    for (int i = 0; i < candidates.size(); i++) {
        Mat out;
        Point m0, m1, m2, m3;
        m0 = candidates[i][0];
        m1 = candidates[i][1];
        m2 = candidates[i][2];
        m3 = candidates[i][3];
        vector<Point2f> newM = {m0, m1, m2, m3};
        vector<double> sides;
        sides.push_back(sqrt(pow(m0.x-m1.x,2) - pow(m0.y-m1.y,2)));
        sides.push_back(sqrt(pow(m1.x-m2.x,2) - pow(m1.y-m2.y,2)));
        sides.push_back(sqrt(pow(m2.x-m3.x,2) - pow(m2.y-m3.y,2)));
        sides.push_back(sqrt(pow(m3.x-m0.x,2) - pow(m3.y-m0.y,2)));
        double side = 0;
        for (int j = 0; j < 4; j++) {
            if (side < sides[j])
                side = sides[j];
        }
        vector<Point2f> dst{Point(0,0),Point(side-1, 0),Point(side-1,side-1),Point(0,side-1)};
        Mat M = getPerspectiveTransform(newM, dst);
        warpPerspective(img, out, M, Size(side, side));
        markers.push_back(out);
        contoursF.push_back(candidates[i]);
    }
}

//FUNC: getMarkerCode
//Looks for code in a warped marker to see if the code
//found matches the provided one
//========================================
bool MarkerDetector::getMarkerCode(string code, Mat img) {
    for (int i = 0; i < markers.size(); i++) {
        double side = markers[i].rows;
        int step = (int) side/5;
        Mat bitMatrix = Mat::zeros(5,5, CV_8UC1);
        Mat gray, thresh;
        cvtColor(markers[i], gray, CV_BGR2GRAY);
        threshold(gray, thresh, 0, 255, THRESH_OTSU);
        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++) {
                int ys = j*step;
                int xs = k*step;
                Mat cut = thresh(Range(ys,ys+step), Range(xs,xs+step));
                cut = makeItOne(cut, 240);
                if (matSum(cut) > (cut.rows*cut.cols)*0.55)
                    bitMatrix.at<uchar>(j,k) = 1;
            }
        }
        if (checkCode(code, bitMatrix)) {
            polylines(img, contoursF[i], true, Scalar(0,0,255), 2);
            centroidF = findCentroid(contoursF[i]);
            circle(img, centroidF, 2, Scalar(0,255,0), 2);
            return true;
        }
    }
    return false;
}

//FUNC: makeItOne
//Puts a "1" where intensity is > threshold
//otherwise leaves it with a "0"
//=========================================
Mat MarkerDetector::makeItOne(Mat mat, int thresh) {
    Mat ones = Mat::zeros(mat.rows, mat.cols, CV_8UC1);
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            if (mat.at<uchar>(i,j) > thresh)
                ones.at<uchar>(i,j) = 1;
        }
    }
    return ones;
}

//FUNC: matSum
//Counts the number of "1"s found in a submatrix mat
//========================================
int MarkerDetector::matSum(Mat mat) {
    int sum=0;
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++)
            sum += (int) mat.at<uchar>(i,j);
    }
    return sum;
}

//FUNC: checkCode
//Tries to match the provided code with the bit matrix
//===========================================
bool MarkerDetector::checkCode(string code, Mat matrix) {
    string matCode = "";
    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            char c = '0' + matrix.at<uchar>(i,j);
            matCode += c;
        }
    }
    string rotCode = code;
    for (int i = 0; i < 4; i++) {
        if (matCode.compare(rotCode) == 0)
            return true;
        rotCode = rotate90(rotCode);
    }
    return false;
}

//FUNC: rotate90
//Rotates CCW 90 degrees a matrix represented as a string
//===========================================
string MarkerDetector::rotate90(string code) {
    string newCode = "00000";
    for (int i = 8; i > 5; i--) {
        newCode += '0';
        newCode += code[i];
        newCode += code[i+5];
        newCode += code[i+10];
        newCode += '0';
    }
    newCode += "00000";
    return newCode;
}

//FUNC: findCentroid
//Finds the centroid of four points
//====================================
Point MarkerDetector::findCentroid(vector<Point> points) {
    int sumX = 0;
    int sumY = 0;
    for (int i = 0; i < 4; i++) {
        sumX += points[i].x;
        sumY += points[i].y;
    }
    Point centroid = Point(sumX/4, sumY/4);
    return centroid;
}
