#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "visualization_msgs/Marker.h"
#include <iomanip>
using namespace std;
double height, last_height;
double vicon_time, height_time, last_vicon_time, last_height_time, image_time, last_image_time;
Eigen::Vector3d velocity, position, last_position, velocity_gt;
Eigen::Quaterniond q;
ros::Publisher pub_vel, pub_vel_gt;
cv::Mat tmp, image,prev_image;
double fx, fy, cx, cy;
cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;
vector<cv::Point2f> points,prev_points(11*14),good_pts, good_prev_pts;
vector<uchar> status;
vector<float> err;
cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.03);
cv::Size winSize(31,31);
cv::Mat mask;
double RMS_vx, RMS_vy;
int frame_count;
double RMS_vx_sum=0.;
double RMS_vy_sum=0.;
ros::Publisher odom_pub;

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
    //cv::imshow("optical_flow", image);
    cv::waitKey(10);
    // TODO: 1. Calculate velocity by LK Optical Flow Algorithm
    // TODO: You can use height as the z value and q(from VICON) as the orientation.
    // TODO: For this part, you can assume the UAV is flying slowly,
    // TODO: which means height changes slowly and q seldom changes.
    // start

    //cv::cornerSubPix(image, points, winSize, zeroZone, criteria);

    if(prev_image.empty())
    {
       cv_ptr->image.copyTo(prev_image);
       last_image_time = image_time-0.0002f;
       frame_count=-1;

       int k=0;
//       for (int i=0;i<26;i++)
//       {
//           for(int j=0;j<21;j++)
//           {
//               prev_points[k++]=cv::Point2f(50.f+i*25.f,40.f+20.f*j);
//           }
//       }
       for (int i=0;i<14;i++)
       {
          for(int j=0;j<11;j++)
          {
              prev_points.push_back(cv::Point2f(50.f+i*50.f,40.f+40.f*j));
          }
       }
    }

    cv::calcOpticalFlowPyrLK(prev_image, image, prev_points, points, status, err, winSize,
                        3, termcrit, 0, 0.002);
    if (points.empty()) return;
     cv::Point2f diff_XY ;
     int numOfSelectedPoint = 0 ;
     float sumX = 0.0 ;
     float sumY = 0.0 ;
     cout << fixed << setprecision(6);
    cout<<"image time"<<image_time<<endl;
    for( int i = 0; i<points.size(); i++ )
    {
            cv::circle(prev_image, prev_points[i], 5, cv::Scalar(0,0,255), 1, 8, 0 );
        if (status[i]){
            good_pts.push_back(points[i]);
            good_prev_pts.push_back(prev_points[i]);
            cv::circle(prev_image, prev_points[i], 5, cv::Scalar(255,0,255), 2, 8, 0 );

        }
    }
    if (good_pts.empty() || good_prev_pts.empty()) return;
    cv::Mat H = cv::findHomography(good_prev_pts, good_pts, 8 ,3 ,mask);

    // TODO: Actually, velocity.z() is not that important for this homework

    if(!mask.empty())
    {
       for (int i=0;i<good_pts.size();i++)
        {   double yy=pow((good_prev_pts[i].y-good_pts[i].y),2.0);
            double xx=pow((good_prev_pts[i].x-good_pts[i].x),2.0);
            if(mask.at<double>(0,i) && sqrt(xx+yy) <25.0)
            {
                diff_XY = (good_pts[i]-good_prev_pts[i]);
                //if (max_x_dis<abs(diff_XY.x)) max_x_dis=abs(diff_XY.x);
                //if (max_y_dis<abs(diff_XY.y)) max_y_dis=abs(diff_XY.y);
                sumX += diff_XY.x;
                sumY += diff_XY.y;
                numOfSelectedPoint++;
                cv::arrowedLine(prev_image, cv::Point2f(good_prev_pts[i].x,good_prev_pts[i].y), cv::Point2f(cvRound(good_prev_pts[i].x+diff_XY.x*2.0), cvRound(good_prev_pts[i].y+diff_XY.y*2.0)),cv::Scalar(255,0,255),3);
               }
       }
       if(numOfSelectedPoint>=5)
       {
        //cout<<"inliners size"<<numOfSelectedPoint<<endl;
        velocity = Eigen::Vector3d(0, 0, 0);
        velocity.x()=sumX*height/((image_time-last_image_time)*fx*numOfSelectedPoint);
        velocity.y()=sumY*height/((image_time-last_image_time)*fy*numOfSelectedPoint);
       }

     }
    else return;
    cv::imshow("prev_test", prev_image);

    good_pts.clear();
    good_prev_pts.clear();
    status.clear();
    cv::swap(prev_image, image);
    frame_count++;

    Eigen::Matrix3d Rr = q.normalized().toRotationMatrix();

    //velocity = Rr*velocity;

    // Visualize in RViz
    visualizeVelocity(position, velocity, 0, Eigen::Vector3d(1, 0, 0), pub_vel);

    //velocity_gt = (position - last_position) / (vicon_time - last_vicon_time);
    visualizeVelocity(position, velocity_gt, 0, Eigen::Vector3d(0, 1, 0), pub_vel_gt);

    last_position = position;
    last_vicon_time = vicon_time;
    last_image_time = image_time;

    // TODO: 2. Analyze the RMS Error here
    // havn't considered time sychornization
    if(frame_count>0)
    {
       cout<<"VICON speed"<<velocity_gt.x()<<" , "<<velocity_gt.y()<<endl;
       cout<<"computed velocity"<<velocity.x()<<" , "<<velocity.y()<<endl;
       RMS_vx_sum += pow((velocity.x()-velocity_gt.x()), 2.0);
       RMS_vy_sum += pow((velocity.y()-velocity_gt.y()), 2.0);
       RMS_vx = sqrt(RMS_vx_sum)/frame_count;
       RMS_vy = sqrt(RMS_vy_sum)/frame_count;

       cout<<"SUm RMS"<<RMS_vx_sum<<", "<<RMS_vy_sum<<endl;
       cout<<"Averge RMS vx: "<<RMS_vx<<", vy: "<<RMS_vy<<endl;
    }
    velocity_gt = Rr.transpose()*velocity_gt;
    nav_msgs::Odometry optical_flow_odom;
    optical_flow_odom.header.frame_id ="camera";
    optical_flow_odom.header.stamp = image_msg->header.stamp;
    optical_flow_odom.twist.twist.linear.x = -velocity.x();
    optical_flow_odom.twist.twist.linear.y = -velocity.y();
    optical_flow_odom.twist.twist.linear.z = -velocity.z();

    optical_flow_odom.pose.pose.position.x = 0;
    optical_flow_odom.pose.pose.position.y = 0;
    optical_flow_odom.pose.pose.position.z = height;
    odom_pub.publish(optical_flow_odom);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "opticalflow_node");
    ros::NodeHandle node;
    cout<<"Using OpenCV version " << CV_VERSION <<endl;
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
    odom_pub = node.advertise<nav_msgs::Odometry>("/opticalflow_node/opticalflow_odom", 100);

    ros::spin();
    return 0;
}
