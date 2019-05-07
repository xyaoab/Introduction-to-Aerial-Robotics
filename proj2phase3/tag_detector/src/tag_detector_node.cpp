#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;
double se_x=0;
double se_y=0;
double se_z=0;
double se_roll=0;
double se_pitch=0;
double se_yaw=0;
double rmse_x, rmse_y, rmse_z, rmse_roll, rmse_pitch, rmse_yaw;
int frame=0;

// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    ROS_INFO("write your code here!");
    //...
    frame++;
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    MatrixXd A(2*pts_2.size(),9);
    for (int i=0;i<pts_2.size();i++)
    {
        A.block<2,9>(2*i,0) << pts_3[i].x, pts_3[i].y, 1, 0,0,0, -un_pts_2[i].x*pts_3[i].x,  -un_pts_2[i].x*pts_3[i].y, -un_pts_2[i].x,
                 0,0,0, pts_3[i].x, pts_3[i].y, 1, -un_pts_2[i].y*pts_3[i].x,  -un_pts_2[i].y*pts_3[i].y, -un_pts_2[i].y;

    }

    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    //right singular vector ->null space
    VectorXd x = svd.matrixV().rightCols(1);
    //cout<<"singual values \n"<<svd.singularValues()<<endl;
    Matrix3d H_hat, H_orthogonal;
   // MatrixXd wrap_K;
    //cv::cv2eigen(K,wrap_K);

    H_hat << x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8];

   // H_hat = wrap_K.inverse()*H_hat;
    if (H_hat(2,2)<0)
        H_hat = -H_hat;
    H_orthogonal << H_hat.col(0), H_hat.col(1), H_hat.col(0).cross(H_hat.col(1));
   // cout<<"H_orthogonal \n"<<H_orthogonal<<endl;
    // cout<<"H_hat \n"<<H_hat<<endl;
    JacobiSVD<MatrixXd> svd_hat(H_orthogonal, ComputeThinU | ComputeThinV);
    R = svd_hat.matrixU()*(svd_hat.matrixV().transpose());
    T = H_hat.col(2)/(H_hat.col(0).norm());
    //cout<<"rotation matrix"<<R<<endl;
    //cout<<"translation"<<T<<endl;

    //...
    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);

    //RMSE  0.236613, 0.188604, 0.589393, 0.122965, 0.204613, 0.550431
    se_x += pow(odom_yourwork.pose.pose.position.x - odom_ref.pose.pose.position.x, 2);
    se_y += pow(odom_yourwork.pose.pose.position.y - odom_ref.pose.pose.position.y, 2);
    se_z += pow(odom_yourwork.pose.pose.position.z - odom_ref.pose.pose.position.z, 2);

    Eigen::Vector3d eulerAngle=Q_ref.matrix().eulerAngles(2,1,0);
    Eigen::Vector3d eulerAngle_yourwork=Q_yourwork.matrix().eulerAngles(2,1,0);

    se_roll += pow(eulerAngle(0) - eulerAngle_yourwork(0),2);
    se_pitch += pow(eulerAngle(1) - eulerAngle_yourwork(1),2);
    se_yaw += pow(eulerAngle(2) - eulerAngle_yourwork(2),2);
    rmse_x = sqrt(se_x/frame);
    rmse_y = sqrt(se_y/frame);
    rmse_z = sqrt(se_z/frame);
    rmse_roll = sqrt(se_roll/frame);
    rmse_pitch = sqrt(se_pitch/frame);
    rmse_yaw = sqrt(se_yaw/frame);
    ROS_INFO("RMSE X, Y, Z, roll, pitch, yaw: \n %f, %f, %f, %f, %f, %f",
             rmse_x, rmse_y, rmse_z, rmse_roll, rmse_pitch,rmse_yaw);

}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
