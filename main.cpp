#include <librealsense/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>
#include "markerdetector.h"
#include "tcpacceptor.h"
#include "tcpstream.h"

#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace cv;
using namespace rs;



float3 filterCoords(vector<vector<double>>*, const float3);
double findValidNeighborDepth(const float, const uint16_t*, int, int);
void sendMessage(const float3, TCPStream*);


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

    //detecting marker on scene
    MarkerDetector *md = new MarkerDetector();
    string code = "0000000110010100001000000";

    //starting network
    TCPStream* stream;
    TCPAcceptor* acceptor = new TCPAcceptor(7777, "127.0.0.1");
    if (acceptor->start() != 0) {
        perror("Could not start network");
        exit(-1);
    }
    stream = acceptor->accept(); //blocks!!!
    cout << "client connected!" << endl;

    //preparing coords vector
    vector<vector<double>> xyz;
    for (int i = 0; i < 3; i++) {
        vector<double> v = {0.0, 0.0, 0.0, 0.0, 0.0};
        xyz.push_back(v);
    }
    int change = 0;


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
                if (depthVal == 0)
                    depthVal = findValidNeighborDepth(scale, depthImg, x, y);
                intrinsics intr = dev->get_stream_intrinsics(stream::rectified_color);

                //cout <<"centroid: " << x << "," << y << "  Depth value: " << depthVal << endl;
                const float2 pixel = {x = (float)x, y = (float)y};
                const float3 coord = intr.deproject(pixel, depthVal);
                filterCoords(&xyz, coord);
                if (change == 0)
                    sendMessage(coord, stream);
                change = (change+1)%3;
            }
            if ('q' == waitKey(10))
                break;
        }
    }
    delete stream;
}

double findValidNeighborDepth(const float scale, const uint16_t *depthImg, int x, int y) {
    double d0, d1, d2, d3;
    for (int i = 1; i < 100; i++) {
        d0 = depthImg[y*(WIDTH-1)+x+i];
        d1 = depthImg[y*(WIDTH-1)+x-i];
        d2 = depthImg[(y+i)*(WIDTH-1)+x];
        d3 = depthImg[(y-i)*(WIDTH-1)+x];
        if (d0 != 0) return d0*scale;
        if (d1 != 0) return d1*scale;
        if (d2 != 0) return d2*scale;
        if (d3 != 0) return d3*scale;
    }
    return 0;
}

float3 filterCoords(vector<vector<double>> *xyz, const float3 coord) {
    vector<double> sum = {0, 0, 0};
    float3 filtered;
    xyz->at(0).push_back(coord.x);
    xyz->at(1).push_back(coord.y);
    xyz->at(2).push_back(coord.z);

    for (int i = 0; i < 3; i++) {
        xyz->at(i).erase(xyz->at(i).begin());
        for (int j = 0; j < 5; j++)
            sum.at(i) += xyz->at(i).at(j);
    }
    filtered.x = sum.at(0)/5.0;
    filtered.y = sum.at(1)/5.0;
    filtered.z = sum.at(2)/5.0;
    cout << filtered.z << endl;
    return filtered;
}

void sendMessage(const float3 coord, TCPStream* stream) {
    char msg[1024];
    string coords = to_string(coord.x) +";"\
            + to_string(coord.y)       + ";"\
            + to_string(coord.z)       + "\r\n";

    msg[coords.length()] = 0;
    snprintf(msg, coords.length(), coords.data());
    stream->send(msg, coords.length());
}


