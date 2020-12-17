#include <iostream>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "OpticalFlowPyrLK.h"
#include "of_drawer.h"

using namespace std;
using namespace cv;

// KITTI1_1 KITTI1_2
string file_1 = "../EUROC1.png";  // first image
string file_2 = "../EUROC2.png";  // second image

int main(int argc, char **argv) 
{
    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    if(img1.empty() || img2.empty())
        cout<<"read image error! \n"<<file_1<<"\n"<<file_2<<endl;
    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(300, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // double t1,t2,time_used;
    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto kp: kp1) 
        pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    auto t1 = chrono::steady_clock::now();
    // cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
    CalcOpticalFlowPyr(img1, img2, pt1, pt2, status, error);
    // CalcOpticalFlowOneLevel(img1, img2, pt1, pt2, status, error);
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv: " << time_used.count() << endl;

    cv::Mat show_points;
    cv::cvtColor(img1, show_points, CV_GRAY2BGR);
    for (int i = 0; i < kp1.size(); i++)
    {
        if (status[i])
        cv::circle(show_points, kp1[i].pt, 2, cv::Scalar(0, 250, 0), 1);
    }

    ReduceVector(pt1,status);
    ReduceVector(pt2,status);
    vector<uchar> outlier_status;
    cv::findFundamentalMat(pt1,pt2, cv::FM_RANSAC, 1, 0.99, outlier_status);
    ReduceVector(pt1,outlier_status);
    ReduceVector(pt2,outlier_status);
    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) 
    {
        if (status[i]) 
        {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(250, 0 ,0 ), 1);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    
    //Draw with Munsell Color System
    ShowMotionColor(img2,0,pt1,pt2);
    cv::imshow("original points", show_points);
    cv::imshow("tracked ", img2_CV);
    cv::waitKey(0);

    return 0;
}

// https://blog.csdn.net/findgeneralgirl/article/details/107919541 
// Lucas B D, Kanade T. An iterative image registration technique with an application to stereo vision
// https://blog.csdn.net/banyao2006/article/details/39484113