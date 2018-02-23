#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "ORBextractor.h"
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace ORB_SLAM2;

int main()
{
    cv::Mat img(cv::imread("/home/vm-ubuntu/work_space/extract_feature_to_OctTree/src/lena.jpg",0));
    assert(img.type() == CV_8UC1 && !img.empty());

//    cvNamedWindow("img", 1);
    cv::imshow("img", img);
    vector<cv::KeyPoint> cv_feats;
    cv::Mat cv_descs;
    ORBextractor extractor;

    double t = cv::getTickCount();
    extractor(img, cv::Mat(), cv_feats, cv_descs);
    t = (double)cv::getTickCount() - t;
    cout<< "Running time:"<<t/cv::getTickFrequency()<<"s"<<endl;

    cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
    cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
    for_each(cv_feats.begin(), cv_feats.end(), [&](cv::KeyPoint i){
        cv::circle(img_rgb, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
    });
    cv::imshow("orb_slam_img", img_rgb);
    cvWaitKey(0);


//    cv::Ptr<cv::ORB> orb = cv::ORB::create(400,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,20);
//   // cv::Mat roi(img.size(), CV_8UC1, cv::Scalar(0));
//    orb->detect(img, cv_feats, img);
//    orb->compute(img, cv_feats, cv_descs);

////    orb->detectAndCompute(img, cv::Mat(), cv_feats, cv_descs);

//    cv::Mat img_opencv = cv::Mat(img.size(), CV_8UC3);
//    //cv::cvtColor(img, img_opencv, CV_GRAY2RGB);

//    std::for_each(cv_feats.begin(), cv_feats.end(), [&](cv::KeyPoint i){
//        cv::circle(img_opencv, i.pt, 4 * (i.octave + 1), cv::Scalar(255, 255, 255), 1);
//    });

//    cvNamedWindow("opencv_img");
//    cv::imshow("opencv_img", img_opencv);
//    cvWaitKey(0);
}
