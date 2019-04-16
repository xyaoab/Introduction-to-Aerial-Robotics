#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "visualization_msgs/Marker.h"

using namespace std;
using namespace cv;
double height, last_height;
double vicon_time, height_time, last_vicon_time, last_height_time, image_time, last_image_time;
Eigen::Vector3d velocity, position, last_position, velocity_gt;
Eigen::Quaterniond q;
ros::Publisher pub_vel, pub_vel_gt;
cv::Mat tmp, image, prev_image;
double fx, fy, cx, cy;
cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;
Mat gx, gy, gt, gxgy, gygt, gx2, gy2, gxgt, sumgx2, sumgy2, sumgxgy, sumgxgt, sumgygt;
Mat u,v;
Mat prev_points;
Mat points;
double RMS_vx, RMS_vy;
int frame_count;
double RMS_vx_sum=0.;
double RMS_vy_sum=0.;
double sumX=0.;
double sumY=0.;

void visualizeVelocity(Eigen::Vector3d position, Eigen::Vector3d velocity,
                       int id, Eigen::Vector3d color, ros::Publisher pub_vel) {
    double scale = 10;
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.2;
    m.scale.y = 0.5;
    m.scale.z = 0;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.color.a = 1.0;
    m.color.r = color.x();
    m.color.g = color.y();
    m.color.b = color.z();
    m.points.clear();
    geometry_msgs::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    m.points.push_back(point);
    point.x = position.x() + velocity.x() * scale;
    point.y = position.y() + velocity.y() * scale;
    point.z = position.z() + velocity.z() * scale;
    m.points.push_back(point);
    pub_vel.publish(m);
}
Mat get_fx(Mat &src1){

    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel.at<double>(0, 0) = -1.0;
    kernel.at<double>(1, 0) = -1.0;

    Mat dst1 = Mat::zeros(src1.size(),src1.type());
    filter2D(src1, dst1, -1, kernel);
    return dst1;
}

Mat get_fy(Mat &src1){

    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel.at<double>(0, 0) = -1.0;
    kernel.at<double>(0, 1) = -1.0;

    Mat dst1 = Mat::zeros(src1.size(),src1.type());
    filter2D(src1, dst1, -1, kernel);

    return dst1;
}


Mat  get_Sum(Mat &m){

      Mat kernel = Mat::ones(31, 31, CV_64FC1);
      Mat dst1 = Mat::zeros(m.size(),m.type());
      filter2D(m, dst1, -1, kernel);

      return dst1;


}
vector<Mat> getGaussianPyramid(Mat &img, int nLevels){
    vector<Mat> pyr;
    pyr.push_back(img);
    for(int i = 0; i < nLevels - 1; i++){
        Mat tmp;
        pyrDown(pyr[pyr.size() - 1], tmp);
        pyr.push_back(tmp);
    }
    return pyr;
}
void lk(Mat &img1, Mat &img2, Mat &u, Mat &v,Mat &prev_points){

    gt=img2-img1;
    gx = get_fx(img2);
    gy = get_fy(img2);

    gx2 = gx.mul(gx);
    gy2 = gy.mul(gy);
    gxgy = gx.mul(gy);
    gxgt = gx.mul(gt);
    gygt = gy.mul(gt);
    //30*30 windown size for summization
    sumgx2 =  get_Sum(gx2);
    sumgy2 =  get_Sum(gy2);
    sumgxgy =  get_Sum(gxgy);
    sumgxgt =  get_Sum(gxgt);
    sumgygt =  get_Sum(gygt);
    Mat tmp = sumgx2.mul(sumgy2) - sumgxgy.mul(sumgxgy);
    u = (sumgxgy.mul(sumgygt) - sumgy2.mul(sumgxgt));
    v = (sumgxgt.mul(sumgxgy) - sumgx2.mul(sumgygt));

    divide(u, tmp, u);
    divide(v, tmp, v);

     u = u.mul(prev_points);
     v = v.mul(prev_points);
     Mat nzu, nzv;

     compare(u,0,nzu,5);
     compare(v,0,nzv,5);
     findNonZero(nzu,nzu);
     findNonZero(nzv,nzv);
     vector<cv::Point3f> goodu(nzu.rows*nzu.cols);
     vector<cv::Point3f> goodv(nzv.rows*nzv.cols);
     for (int i=0;i<nzu.rows*nzu.cols;i++)
     {
        Point pnt = nzu.at<Point>(i);
        goodu[i].x = pnt.x;
        goodu[i].y = pnt.y;
        goodu[i].z = u.at<double>(pnt.y, pnt.x);
     }
     for (int i=0;i<nzv.rows*nzv.cols;i++)
     {
        Point pnt = nzv.at<Point>(i);
        goodv[i].x = pnt.x;
        goodv[i].y = pnt.y;
        goodv[i].z = v.at<double>(pnt.y, pnt.x);
     }
     Vec6f uline, vline;
     float sumX=0;
     float sumY=0;
     fitLine(goodu, uline, cv::DIST_L1, 1, 0.01, 0.01);
     for (int i=0;i<goodu.size();i++)
     {
         sumX+=(goodu[i].x - uline[3])/ uline[0] * uline[2] + uline[5];
     }

     fitLine(goodv, vline, cv::DIST_L2, 1, 0.01, 0.01);
     for (int i=0;i<goodv.size();i++)
     {
         sumY+=(goodv[i].x - vline[3])/ vline[0] * vline[2] + vline[5];
     }


     u = Mat::ones(u.rows, u.cols, CV_64FC1)*sumX/goodu.size();
     v = Mat::ones(v.rows, v.cols, CV_64FC1)*sumY/goodv.size();
}


void lkIterative(Mat &img1, Mat &img2, Mat &u, Mat &v,Mat &prev_points){

    Mat u_tmp = Mat::zeros(img1.size(),CV_64FC1);
    Mat v_tmp = Mat::zeros(img1.size(),CV_64FC1);
    lk(img1, img2, u_tmp, v_tmp, prev_points);
    u = u+u_tmp;
    v = v+v_tmp;
    for(int k=0;k<2;k++){

         double du = u_tmp.at<double>(0,0);
         double dv = v_tmp.at<double>(0,0);
         if (abs(du)<0.5 && abs(dv)<0.5) break;
         Mat image_tmp = Mat::zeros(img1.size(),img1.type());
         for (int i=0;i<img2.cols;i++){
             for(int j=0;j<img2.rows;j++){
                 if(cvRound(j+dv)<img2.rows && cvRound(i+du)<img2.cols
                         && cvRound(j+dv)>=0 && cvRound(i+du)>=0)
                 image_tmp.at<double>(j,i) =img2.at<double>(cvRound(j+dv),cvRound(i+du));
             }
         }

        lk(img1, image_tmp, u_tmp, v_tmp, prev_points);
        u = u+u_tmp;
        v = v+v_tmp;

    }
  //  cout<<"iterative u(0,0)"<<u.at<double>(40,50)<<endl;

}



void coarseToFine(Mat &img1, Mat &img2, Mat &u, Mat &v, Mat & prev_points, int nLevels=2){

    vector<Mat> pyr1 = getGaussianPyramid(img1, nLevels);

    vector<Mat> pyr2 = getGaussianPyramid(img2, nLevels);
    vector<Mat> pyr3 = getGaussianPyramid(prev_points, nLevels);
    Mat upu, upv;
    for(int i = nLevels - 1; i >= 0; i--){

        Mat tmpu = Mat::zeros(pyr1[i].rows, pyr1[i].cols, CV_64FC1);
        Mat tmpv = Mat::zeros(pyr2[i].rows, pyr2[i].cols, CV_64FC1);
        lkIterative(pyr1[i], pyr2[i], tmpu, tmpv, pyr3[i]);
        if(i != nLevels - 1){
            tmpu += upu;
            tmpv += upv;
        }
        if(i == 0){
            u = tmpu;
            v = tmpv;
            return;
        }
        pyrUp(tmpu, upu);
        pyrUp(tmpv, upv);

        Mat map1(upu.size(), CV_32FC2);
        Mat map2(upu.size(), CV_32FC2);
        for (int y = 0; y < map1.rows; ++y){
            for (int x = 0; x < map1.cols; ++x){
                Point2f f = Point2f((float)(upu.at<double>(y, x)), (float)(upv.at<double>(y, x)));
                map1.at<Point2f>(y, x) = Point2f(x + f.x / 2, y + f.y / 2);
                map2.at<Point2f>(y, x) = Point2f(x - f.x / 2, y - f.y / 2);
            }
        }

        Mat warped1(map1.size(),pyr1[i-1].type());
        Mat warped2(map2.size(),pyr2[i-1].type());

        remap(pyr1[i - 1], warped1, map1, cv::Mat(), INTER_LINEAR);
        remap(pyr2[i - 1], warped2, map2, cv::Mat(), INTER_LINEAR);

        warped1.copyTo(pyr1[i - 1]);
        warped2.copyTo(pyr2[i - 1]);

    }
}




void heightCallback(const sensor_msgs::Range::ConstPtr &height_msg) {
    height = height_msg->range;
    height_time = height_msg->header.stamp.toSec();
    if(last_height_time!=0)
    {
        velocity.z()=(height-last_height)/(height_time-last_height_time);
    }
    last_height=height;
    last_height_time=height_time;
}

void viconCallback(const nav_msgs::Odometry::ConstPtr &vicon_msg) {
    position.x() = vicon_msg->pose.pose.position.x;
    position.y() = vicon_msg->pose.pose.position.y;
    position.z() = vicon_msg->pose.pose.position.z;
    q = Eigen::Quaterniond(vicon_msg->pose.pose.orientation.w,
                           vicon_msg->pose.pose.orientation.x,
                           vicon_msg->pose.pose.orientation.y,
                           vicon_msg->pose.pose.orientation.z);
    vicon_time = vicon_msg->header.stamp.toSec();
    velocity_gt.x() = vicon_msg->twist.twist.linear.x;
    velocity_gt.y() = vicon_msg->twist.twist.linear.y;
    velocity_gt.z() = vicon_msg->twist.twist.linear.z;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &image_msg) {
    image_time = image_msg->header.stamp.toSec();
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    cv_ptr->image.copyTo(tmp);
    cv::undistort(tmp, image, cameraMatrix, distCoeffs);
   // cv::imshow("optical_flow", image);
    cv::waitKey(10);
    // TODO: 1. Calculate velocity by LK Optical Flow Algorithm
    // TODO: You can use height as the z value and q(from VICON) as the orientation.
    // TODO: For this part, you can assume the UAV is flying slowly,
    // TODO: which means height changes slowly and q seldom changes.

        // TODO: Actually, velocity.z() is not that important for this homework
    u = Mat::zeros(image.size(),CV_64FC1);
    v = Mat::zeros(image.size(),CV_64FC1);
    if(prev_image.empty())
    {
       frame_count=-1;
       prev_points = Mat::zeros(image.size(),CV_64FC1);
       cv_ptr->image.copyTo(prev_image);
       last_image_time = image_time-0.0002f;
       prev_image.convertTo(prev_image, CV_64FC1, 1.0/255, 0);

       for (int i=0;i<26;i++)
       {
           for(int j=0;j<21;j++)
           {
               prev_points.at<double>(40+20*j,50+i*25)=1;
               circle(prev_image,Point2f(50+i*25,40+20*j), 5, cv::Scalar(0,0,255), 1, 8, 0 );
           }
       }

    }
    image.convertTo(image, CV_64FC1, 1.0/255, 0);

    coarseToFine(prev_image, image, u, v, prev_points);
    //lk(prev_image, image, u, v, prev_points);



    for (int i=0;i<26;i++)
    {
        for(int j=0;j<21;j++)
        {

           arrowedLine(prev_image, Point2f(50+i*25,40+20*j), cv::Point2f(cvRound(50+i*25+u.at<double>(40+20*j, 50+i*25)*3), cvRound(40+20*j+v.at<double>(40+20*j, 50+i*25)*3)),cv::Scalar(255,0,255),3);

        }
    }
    sumX = sum(u)[0];
     sumY = sum(v)[0];
    velocity = Eigen::Vector3d(0, 0, 0);
    velocity.x()=sumX*height/((image_time-last_image_time)*fx*u.rows*u.cols*0.1);
    velocity.y()=sumY*height/((image_time-last_image_time)*fy*v.rows*v.cols*0.1);
    cout<<"speed"<<velocity.x()<<" , "<<velocity.y()<<endl;
    prev_image.convertTo(prev_image, CV_8UC1, 255, 0);
    imshow("optical_flow", prev_image);
    // Visualize in RViz
    visualizeVelocity(position, velocity, 0, Eigen::Vector3d(1, 0, 0), pub_vel);

    //velocity_gt = (position - last_position) / (vicon_time - last_vicon_time);
    visualizeVelocity(position, velocity_gt, 0, Eigen::Vector3d(0, 1, 0), pub_vel_gt);

    last_position = position;
    last_vicon_time = vicon_time;
    last_image_time = image_time;
    cv::swap(prev_image, image);
    frame_count++;

     cout<<"JACKU"<<endl;
    // TODO: 2. Analyze the RMS Error here
     if(frame_count)
     {
        cout<<"VICON speed"<<velocity_gt.x()<<" , "<<velocity_gt.y()<<endl;
        RMS_vx_sum += pow((velocity.x()-velocity_gt.x()), 2.0);
        RMS_vy_sum += pow((velocity.y()-velocity_gt.y()), 2.0);
        RMS_vx = sqrt(RMS_vx_sum)/frame_count;
        RMS_vy = sqrt(RMS_vy_sum)/frame_count;

        cout<<"SUm RMS"<<RMS_vx_sum<<", "<<RMS_vy_sum<<endl;
        cout<<"Averge RMS vx: "<<RMS_vx<<", vy: "<<RMS_vy<<endl;
     }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "opticalflow_node");
    ros::NodeHandle node;

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    fx = cameraMatrix.at<double>(0, 0) = 362.565;
    fy = cameraMatrix.at<double>(1, 1) = 363.082;
    cx = cameraMatrix.at<double>(0, 2) = 365.486;
    cy = cameraMatrix.at<double>(1, 2) = 234.889;

    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.278765;
    distCoeffs.at<double>(1, 0) = 0.0694761;
    distCoeffs.at<double>(2, 0) = -2.86553e-05;
    distCoeffs.at<double>(3, 0) = 0.000242845;


    imageSize.height = 480;
    imageSize.width = 752;

    ros::Subscriber sub_height = node.subscribe("/tfmini_ros_node/TFmini", 10, heightCallback);
    ros::Subscriber sub_image = node.subscribe("/camera/image_raw", 10, imageCallback);
    ros::Subscriber sub_vicon = node.subscribe("/uwb_vicon_odom", 10, viconCallback);
    pub_vel = node.advertise<visualization_msgs::Marker>("/optical_flow/velocity", 1, true);
    pub_vel_gt = node.advertise<visualization_msgs::Marker>("/optical_flow/velocity_gt", 1, true);

    ros::spin();
    return 0;
}
