#include <librealsense/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>
#include <zmq.hpp>
#include <cmath>
#include "markerdetector.h"


#define WIDTH 320
#define HEIGHT 240

using namespace std;
using namespace cv;
using namespace rs;
using namespace zmq;


float3 filterCoords(vector<vector<double>>*, const float3, float3);
double findValidNeighborDepth(double, const float, const uint16_t*, int, int);
void sendCoords(const float3, socket_t*);
float filterZOutlier(float, float);


int main()
{
    //create context
    context ctx;

    //access device
    device *dev = ctx.get_device(0);

    //configure IR stream
    dev->enable_stream(stream::depth, WIDTH, HEIGHT, format::z16, 60);
    dev->enable_stream(stream::color, WIDTH, HEIGHT, format::bgr8, 60);
    dev->enable_stream(stream::infrared, WIDTH, HEIGHT, format::any, 60);
    try { dev->enable_stream(stream::infrared2, WIDTH, HEIGHT, format::any, 60); } catch(...) {}
    dev->set_option(option::r200_depth_units, 100);

    //start streaming
    dev->start();

    //warming up
    for (int i = 0; i < 30; i++)
        dev->wait_for_frames();

    //get depth parameter
    const float scale = dev->get_depth_scale();

    //some placeholders
    Mat color, output, depth;
    int k = 0;
    double prevDepth = 0;

    //detecting marker on scene
    MarkerDetector *md = new MarkerDetector();
    string code = "0000000110010100001000000";

    //starting network
    context_t context(1);
    socket_t socket (context, ZMQ_PUB);
    socket.bind("tcp://127.0.0.1:7777");

    //preparing coords vector
    vector<vector<double>> xyz;
    for (int i = 0; i < 3; i++) {
        vector<double> v = {0.0, 0.0, 0.0, 0.0, 0.0};
        xyz.push_back(v);
    }
    float3 filtered;
    filtered.x = 0;
    filtered.y = 0;
    filtered.z = 0;


    for (;;) {

        if (dev->poll_for_frames()) {
            const uint16_t *colorImg = (const uint16_t*) dev->get_frame_data(stream::rectified_color);
            const uint16_t *depthImg = (const uint16_t*) dev->get_frame_data(stream::depth);
            color = Mat(Size(WIDTH,HEIGHT), CV_8UC3, (void*) colorImg);
            depth = Mat(Size(WIDTH,HEIGHT), CV_8UC1, (void*) depthImg);

            //imshow("RGB", color);
            output = md->detect(code, color);
            imshow("processed", output);
            imshow("depth", depth);

            double depthVal = 0;
            if (md->detected){ //&& stream != NULL) {
                int x = md->centroidF.x;
                int y = md->centroidF.y;
                depthVal = scale * depthImg[y*(WIDTH-1)+x];
                double diff = abs(depthVal - prevDepth);
                if (depthVal == 0 || (diff > 0.15 && prevDepth != 0))
                    depthVal = findValidNeighborDepth(prevDepth, scale, depthImg, x, y);
                prevDepth = depthVal;
                intrinsics intr = dev->get_stream_intrinsics(stream::rectified_color);

                //cout <<"centroid: " << x << "," << y << "  Depth value: " << depthVal << endl;
                const float2 pixel = {x = (float)x, y = (float)y};
                const float3 coord = intr.deproject(pixel, depthVal);
                filtered = filterCoords(&xyz, coord, filtered);
                //sendCoords(coord, &socket);
                string coords = to_string(filtered.x) + ";"\
                        + to_string(filtered.y)       + ";"\
                        + to_string(filtered.z)       + "\r\n";
                message_t msg(coords.length());

                snprintf((char*) msg.data(), coords.length(), coords.data());
                k = (k+1)%2;
                if (k == 0) {
                    socket.send(msg);
                    cout << filtered.z << endl;
                }

            }
            if ('q' == waitKey(10))
                break;
        }
    }
}

double findValidNeighborDepth(double prevDepth, const float scale, const uint16_t *depthImg, int x, int y) {
    double d0, d1, d2, d3;
    double thresh = 0.2;
    for (int i = 1; i < 75; i++) {
        d0 = scale * depthImg[y*(WIDTH-1)+x+i];
        d1 = scale * depthImg[y*(WIDTH-1)+x-i];
        d2 = scale * depthImg[(y+i)*(WIDTH-1)+x];
        d3 = scale * depthImg[(y-i)*(WIDTH-1)+x];
        if (d0 != 0 && abs(d0 - prevDepth) < thresh) return d0;
        if (d1 != 0 && abs(d1 - prevDepth) < thresh) return d1;
        if (d2 != 0 && abs(d2 - prevDepth) < thresh) return d2;
        if (d3 != 0 && abs(d3 - prevDepth) < thresh) return d3;
    }
    return 0;
}

float3 filterCoords(vector<vector<double>> *xyz, const float3 coord, float3 prev) {
    vector<double> sum = {0, 0, 0};
    float3 filtered;
    xyz->at(0).push_back(coord.x);
    xyz->at(1).push_back(coord.y*(-1));
    float z = filterZOutlier(coord.z, prev.z);
    xyz->at(2).push_back(z);

    for (int i = 0; i < 3; i++) {
        xyz->at(i).erase(xyz->at(i).begin());
        for (int j = 0; j < 5; j++)
            sum.at(i) += xyz->at(i).at(j);
    }
    filtered.x = sum.at(0)/5.0;
    filtered.y = sum.at(1)/5.0;
    filtered.z = sum.at(2)/5.0;
    //cout << coord.z << "___" << filtered.z << endl;
    return filtered;
}

float filterZOutlier(float coordZ, float filteredZ) {
    if (coordZ > filteredZ + 0.3)
        return filteredZ + 0.005;
    else if (coordZ < filteredZ - 0.3)
        return filteredZ - 0.005;
    else
        return coordZ;
}

void sendCoords(const float3 coord, socket_t *socket) {
    string coords = to_string(coord.x) + ";"\
            + to_string(coord.y)       + ";"\
            + to_string(coord.z)       + "\r\n";
    message_t msg(coords.length());

    snprintf((char*) msg.data(), coords.length(), coords.data());
    socket->send(msg);
}


