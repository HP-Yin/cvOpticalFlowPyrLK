#pragma once
#ifndef DRAWER_H
#define DRAWER_H

#include <iostream>  
#include <opencv2/opencv.hpp>  
      
using namespace cv;  
using namespace std;  
      
#define UNKNOWN_FLOW_THRESH 1e9  

//  Color encoding of flow vectors from  
//  httpmembers.shaw.caquadiblocothercolint.htm  
//  This code is modified from  
//  httpvision.middlebury.eduflowdata  
void makecolorwheel(vector<cv::Scalar> &colorwheel)  
{
    int RY = 15;  
    int YG = 6;  
    int GC = 4;  
    int CB = 11;  
    int BM = 13;  
    int MR = 6;  
      
    int i;  
      
    for (i = 0; i < RY; i++) 
        colorwheel.push_back(Scalar(255,       255*i/RY,     0));  
    for (i = 0; i < YG; i++) 
        colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));  
    for (i = 0; i < GC; i++) 
        colorwheel.push_back(Scalar(0,         255,      255*i/GC));  
    for (i = 0; i < CB; i++) 
        colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));  
    for (i = 0; i < BM; i++) 
        colorwheel.push_back(Scalar(255*i/BM,      0,        255));  
    for (i = 0; i < MR; i++) 
        colorwheel.push_back(Scalar(255,       0,        255-255*i/MR));  
}  
      
void Motions2Color(Mat flow, Mat &color)  
{  
    if (color.empty())  
        color.create(flow.rows, flow.cols, CV_8UC3);  
    
    static vector<Scalar> colorwheel; //Scalar r,g,b  
    if (colorwheel.empty())  
        makecolorwheel(colorwheel);  
    
    // determine motion range:  
    float maxrad = -1;  
    
    // Find max flow to normalize fx and fy  
    for (int i= 0; i < flow.rows; ++i)   
    {  
        for (int j = 0; j < flow.cols; ++j)   
        {  
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
            float fx = flow_at_point[0];  
            float fy = flow_at_point[1];  
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
                continue;  
            float rad = sqrt(fx * fx + fy * fy);  
            maxrad = maxrad > rad ? maxrad : rad;  
        }  
    }  
    
    for (int i= 0; i < flow.rows; ++i)   
    {  
        for (int j = 0; j < flow.cols; ++j)   
        {
            uchar *data = color.data + color.step[0] * i + color.step[1] * j;  
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
    
            float fx = flow_at_point[0] / maxrad;  
            float fy = flow_at_point[1] / maxrad;  
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
            {  
                data[0] = data[1] = data[2] = 0;  
                continue;  
            }  
            float rad = sqrt(fx * fx + fy * fy);  
    
            float angle = atan2(-fy, -fx) / CV_PI;  
            float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);  
            int k0 = (int)fk;  
            int k1 = (k0 + 1) % colorwheel.size();  
            float f = fk - k0;  
            //f = 0; // uncomment to see original color wheel  
    
            for (int b = 0; b < 3; b++)   
            {  
                float col0 = colorwheel[k0][b] / 255.0;  
                float col1 = colorwheel[k1][b] / 255.0;  
                float col = (1 - f) * col0 + f * col1;  
                if (rad <= 1)  
                    col = 1 - rad * (1 - col); // increase saturation with radius  
                else  
                    col *= .75; // out of range  
                data[2 - b] = (int)(255.0 * col);  
            }  
        }  
    }  
}

void ShowMotionColor(cv::Mat img,int level,vector<cv::Point2f> pts_prev,vector<cv::Point2f> pts_next)
{
    vector<cv::Scalar> result_color;
    assert(pts_prev.size()==pts_next.size());
    static vector<cv::Scalar> colorwheel; //Scalar r,g,b  
    if (colorwheel.empty())  
        makecolorwheel(colorwheel);  

    // determine motion range:  
    float maxrad = -1;    
    float avg_rad=0,sum_rad=0;
    for (int i= 0; i < pts_prev.size(); ++i)   
    {
        float dx = pow(2,level)*(pts_next[i].x-pts_prev[i].x);
        float dy = pow(2,level)*(pts_next[i].y-pts_prev[i].y);
        // if(dx>UNKNOWN_FLOW_THRESH||dy>UNKNOWN_FLOW_THRESH)
            // return cv::Scalar(255,255,255);
        /*
        if(y < 10)
        { var = 30;}
        else
        {var = 40;}
        可以写成:
        var = (y < 10) ? 30 : 40;
        */
        float rad = sqrt(dx*dx + dy*dy);  
        maxrad = maxrad > rad ? maxrad : rad;   
        sum_rad+=rad;
    }
    avg_rad = sum_rad/pts_prev.size();
    cout<<"maxrad "<<maxrad<<endl;

    for (int i= 0; i < pts_prev.size(); ++i)
    {
        float dx = pow(2,level)*(pts_next[i].x-pts_prev[i].x);
        float dy = pow(2,level)*(pts_next[i].y-pts_prev[i].y);
        float fx = dx/avg_rad;
        float fy = dy/avg_rad;
        float rad = sqrt(fx * fx + fy * fy);
        float angle = atan2(-fy, -fx) / CV_PI;
        float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1); // colorwheel.size 55
        int k0 = (int)fk;  
        int k1 = (k0 + 1) % colorwheel.size();  
        float f = fk - k0;

        int data[3];
        for (int b = 0; b < 3; b++)   
        {
            float col0 = colorwheel[k0][b] / 255.0;  
            float col1 = colorwheel[k1][b] / 255.0;  
            float col = (1 - f) * col0 + f * col1;  
            if (rad <= 1)  
                col = 1 - rad * (1 - col); // increase saturation with radius  
            else  
                col *= .75; // out of range  
            data[2 - b] = (int)(255.0 * col);
        }
        cv::Scalar color(data[0],data[1],data[2]);
        result_color.push_back(color);
    }

    if(img.channels()<3)
        cv::cvtColor(img, img, CV_GRAY2BGR);
    cv::Mat show = img.clone();
    cv::resize(show,show,cv::Size(),pow(2,level),pow(2,level),cv::INTER_NEAREST);
    int side = 5,half_side = 2;
    //cv::Rect(int a,int b,int c,int d)a,b为矩形的左上角坐标,c,d为矩形的长和宽
    for(int j = 0; j < pts_next.size(); j++)
    {
        cv::Point2f centre = pts_next[j]*pow(2,level);
        // cv::circle(show,pts_next[j]*pow(2,level),2,result_color[j],0);
        cv::rectangle(show,cv::Rect(centre.x-half_side,centre.y-half_side,side,side),result_color[j],-1,1,0);
    }
    cv::imshow("MotionColor",show);
}


#endif

// https://bbs.elecfans.com/jishu_485979_1_1.html    
// Munsell颜色系统来显示