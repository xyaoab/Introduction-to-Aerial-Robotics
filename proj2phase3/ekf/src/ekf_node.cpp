#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include "visualization_msgs/Marker.h"
using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
ros::Publisher pub_vel_imu;
ros::Publisher pub_vel_opt;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt_pnp = MatrixXd::Identity(6,6);
MatrixXd Rt_opt = MatrixXd::Identity(3,3);
// state and covariance
VectorXd x = VectorXd::Zero(15);
MatrixXd cov = MatrixXd::Identity(15,15);
Vector3d gravity(0,0,-9.8);
double t_last = 0;
bool update = false;


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


void publish(const VectorXd &x, const sensor_msgs::Imu::ConstPtr &msg)
{

    Matrix3d Rxx;
    Rxx << cos(x(5))*cos(x(4))- sin(x(3))*sin(x(5))*sin(x(4)), -cos(x(3))*sin(x(5)), cos(x(5))*sin(x(4))+cos(x(4))*sin(x(3))*sin(x(5)),
          cos(x(4))*sin(x(5))+cos(x(5))*(sin(x(3)))*sin(x(4)), cos(x(3))*cos(x(5)), sin(x(5))*sin(x(4))-cos(x(5))*sin(x(3))*cos(x(4)),
          -cos(x(3))*sin(x(4)), sin(x(3)), cos(x(3))*cos(x(4));

    Quaterniond q_predict(Rxx);
    nav_msgs::Odometry ekf_odom;
    ekf_odom.header.frame_id ="world"; //world is vicon
    ekf_odom.header.stamp = msg->header.stamp;
    ekf_odom.pose.pose.position.x = x(0);
    ekf_odom.pose.pose.position.y = x(1);
    ekf_odom.pose.pose.position.z = x(2);
    ekf_odom.twist.twist.linear.x = x(6);
    ekf_odom.twist.twist.linear.y = x(7);
    ekf_odom.twist.twist.linear.z = x(8);
    ekf_odom.pose.pose.orientation.x = q_predict.x();
    ekf_odom.pose.pose.orientation.y = q_predict.y();
    ekf_odom.pose.pose.orientation.z = q_predict.z();
    ekf_odom.pose.pose.orientation.w = q_predict.w();

    odom_pub.publish(ekf_odom);
}
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    double dt = msg->header.stamp.toSec() - t_last;
    if(!update || dt>1){
        t_last = msg->header.stamp.toSec();
        //update=true;
        return;
    }
    //accerlation
    Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    //angular velocity
    Vector3d ang_vel(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    Matrix3d G, R, G_inv_dot, R_dot;
    double phi =x(3);
    double theta=x(4);
    double psi=x(5);
    G << cos(theta), 0, -cos(phi)*sin(theta),
         0, 1, sin(phi),
         sin(theta), 0, cos(theta)*cos(phi);

    R << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
        cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
            -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);

    Matrix3d G_inv = G.inverse();

    G_inv_dot << 0, ang_vel(2)*cos(theta) - ang_vel(0)*sin(theta), 0,
            ang_vel(0)*sin(theta) - ang_vel(2)*cos(theta) - (ang_vel(2)*cos(theta)*sin(phi)*sin(phi))/(cos(phi)*cos(phi)) + (ang_vel(0)*sin(phi)*sin(phi)*sin(theta))/(cos(phi)*cos(phi)), (ang_vel(0)*cos(theta)*sin(phi))/cos(phi) + (ang_vel(2)*sin(phi)*sin(theta))/cos(phi), 0,
            (ang_vel(2)*cos(theta)*sin(phi))/(cos(phi)*cos(phi)) - (ang_vel(0)*sin(phi)*sin(theta))/(cos(phi)*cos(phi)), - (ang_vel(0)*cos(theta))/cos(phi) - (ang_vel(2)*sin(theta))/cos(phi), 0;
    R_dot << accel(1)*sin(phi)*sin(psi) + accel(2)*cos(phi)*cos(theta)*sin(psi) - accel(0)*cos(phi)*sin(theta)*sin(psi), accel(2)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) - accel(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), - accel(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - accel(2)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)) - accel(1)*cos(phi)*cos(psi),
           accel(0)*cos(phi)*cos(psi)*sin(theta) - accel(2)*cos(phi)*cos(theta)*cos(psi) - accel(1)*cos(psi)*sin(phi), accel(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - accel(0)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)),   accel(0)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) + accel(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - accel(1)*cos(phi)*sin(psi),
           accel(1)*cos(phi) - accel(2)*cos(theta)*sin(phi) + accel(0)*sin(phi)*sin(theta), - accel(0)*cos(phi)*cos(theta) - accel(2)*cos(phi)*sin(theta), 0;

    MatrixXd At = MatrixXd::Zero(15,15);
    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);
    At.block<3,3>(3,3) << G_inv_dot;
    At.block<3,3>(6,3) << R_dot;
    At.block<3,3>(3,9) << -G_inv;
    At.block<3,3>(6,12) << -R;

    MatrixXd Ut = MatrixXd::Zero(15,12);
    Ut.block<3,3>(3,0) << -G_inv;
    Ut.block<3,3>(6,3) << -R;
    Ut.block<6,6>(9,6) << MatrixXd::Identity(6,6);

    MatrixXd Ft = MatrixXd::Identity(15,15) + dt*At;
    MatrixXd Vt = dt*Ut;
    cov = Ft * cov *(Ft.transpose()) + Vt * Q * (Vt.transpose());
    VectorXd f = VectorXd::Zero(15);
    f.block<3,1>(0,0) = x.block<3,1>(6,0);
    f.block<3,1>(3,0) = G_inv * (ang_vel-Vector3d(x(9),x(10),x(11)));
    f.block<3,1>(6,0) = gravity + R*(accel-Vector3d(x(12),x(13),x(14)));

    x += dt*f;
    //cout<<"imu callback x" <<x<<endl;
    t_last = msg->header.stamp.toSec();
    publish(x, msg);
    visualizeVelocity(Vector3d(x(0),x(1),x(2)), Vector3d(x(6),x(7),x(8)), 0, Eigen::Vector3d(0, 0, 1), pub_vel_imu);


}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
Eigen::Matrix3d Rvm;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) // pnp transfrom to vicon frame instead of mat
{
    //your code for update
    //For part 1
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;

        Quaterniond q(msg-> pose.pose.orientation.w, msg-> pose.pose.orientation.x, msg-> pose.pose.orientation.y, msg-> pose.pose.orientation.z);
        Matrix3d Rcm = q.toRotationMatrix();
        Vector3d Tcm = Vector3d(msg-> pose.pose.position.x, msg-> pose.pose.position.y, msg-> pose.pose.position.z);
        Matrix3d Rmi = Rcm.transpose() * Rcam.transpose();// Rmi=Rmc*Rci
        Vector3d Tic = Vector3d(-0.1, 0, -0.03);
        Vector3d Tmi = -Rcm.transpose() * (Rcam.transpose()*Tic + Tcm); // Rmc*Tci+Tmc

        Rvm << 0,1,0,
                1,0,0,
                0,0,-1; //mat in vicon frame

        Matrix3d Rwi = Rvm * Rmi;
        Vector3d Twi =  Rvm * Tmi;

        MatrixXd Ct = MatrixXd::Zero(6,15);
        Ct.block<6,6>(0,0) = MatrixXd::Identity(6,6);
        MatrixXd Wt = MatrixXd::Identity(6,6);
        MatrixXd Kt = cov*Ct.transpose()*(Ct*cov*Ct.transpose()+Wt*Rt_pnp*Wt.transpose()).inverse();

        double phi = asin(Rwi(2,1));
        double theta = atan2(-Rwi(2,0)/cos(phi), Rwi(2,2)/cos(phi));
        double psi = atan2(-Rwi(0,1)/cos(phi), Rwi(1,1)/cos(phi));

        VectorXd imu_pose(6), imu_pose_diff(6);
        imu_pose << Twi, phi, theta, psi;
        imu_pose_diff = imu_pose - Ct*x;
        if(imu_pose_diff(3)>M_PI)
            imu_pose_diff(3) -= 2*M_PI;
        else if(imu_pose_diff(3)<-M_PI)
            imu_pose_diff(3) += 2*M_PI;

        if(imu_pose_diff(4)>M_PI)
            imu_pose_diff(4) -= 2*M_PI;
        else if(imu_pose_diff(4)<-M_PI)
            imu_pose_diff(4) += 2*M_PI;

        if(imu_pose_diff(5)>M_PI)
            imu_pose_diff(5) -= 2*M_PI;
        else if(imu_pose_diff(5)<-M_PI)
            imu_pose_diff(5) += 2*M_PI;

        x += Kt * imu_pose_diff;
        cov -= Kt*Ct*cov;

        cout<<"odom callback x: " <<x<<endl;
        update = true;

     // For part 2 & 3
    // camera position in the IMU frame = (0.1, 0, 0.03)
    // camera orientaion in the IMU frame = Quaternion(0, 0, 0, 1); w x y z, respectively
    //                     RotationMatrix << -1, 0, 0,
    //                                       0, -1, 0,
    //                                       0, 0, 1;


}

void optical_flow_odom_callback(const nav_msgs::Odometry::ConstPtr &msg){ // in vicon frame
    if (!update)
        return;
    //Rcam Ric
    //cout << "opt flow R_cam:  " << Rcam << endl;


    Vector3d Tic = Vector3d(-0.1, 0, -0.03);
    Vector3d Tvi(x(0),x(1),x(2));
    Vector3d Vv = Vector3d(x(6), x(7), x(8));

    double phi =x(3);
    double theta=x(4);
    double psi=x(5);
    double vx=x(6);
    double vy=x(7);
    double vz=x(8);

    Matrix3d Rvi;
    Rvi << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
            cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
                -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);

    Vector3d Pc = -Rcam.transpose()*Rvi.transpose()*Tvi - Rcam.transpose()*Tic;
    Vector3d Vc = Rcam.transpose() * Rvi.transpose() * Vv;

    MatrixXd Ct = MatrixXd::Zero(3,15); //z,vx,vy
    //d vx vy dx3
    Ct.block<2,3>(1,6) <<cos(phi)*sin(psi),                                   -cos(phi)*cos(psi),            -sin(phi),
                    sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta), - cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta),  cos(phi)*sin(theta);
// d vx vy dx2
    Ct.block<2,3>(1,3) <<vy*cos(psi)*sin(phi) - vz*cos(phi) - vx*sin(phi)*sin(psi),                                                                                                                                          0,                                                                       vx*cos(phi)*cos(psi) + vy*cos(phi)*sin(psi),
                         vx*cos(phi)*sin(psi)*sin(theta) - vy*cos(phi)*cos(psi)*sin(theta) - vz*sin(phi)*sin(theta), vx*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + vy*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + vz*cos(phi)*cos(theta), vx*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - vy*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta));
    //z
    Ct.block<1,3>(0,3) << x(0)*cos(phi)*cos(theta)*sin(psi) - x(1)*cos(phi)*cos(psi)*cos(theta) - x(2)*cos(theta)*sin(phi),   x(0)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + x(1)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - x(2)*cos(phi)*sin(theta), x(1)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - x(0)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi));

    Ct.block<1,3>(0,0)<< cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),  cos(phi)*cos(theta);

    MatrixXd Wt = MatrixXd::Identity(3,3);
    MatrixXd Kt = cov*Ct.transpose()*(Ct*cov*Ct.transpose()+Wt*Rt_opt*Wt.transpose()).inverse();

    Vector3d ztg((msg-> pose.pose.position.z-Pc(2)),
            (msg->twist.twist.linear.x-Vc(0)),
            (msg->twist.twist.linear.y-Vc(1)));//z,vx,vy


    x += Kt * ztg;
    cov -= Kt*Ct*cov;
    Vector3d velocity=Rvi*Rcam*Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);// Rvi*Ric
    visualizeVelocity(Vector3d(x(0),x(1),x(2)), velocity, 0, Eigen::Vector3d(1, 0, 0), pub_vel_opt);
    cout<<"optical flow odom callback x" <<x<<endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");

    Rt_pnp.topLeftCorner(3, 3) = 0.01 * Rt_pnp.topLeftCorner(3, 3);
    Rt_pnp.bottomRightCorner(3, 3) = 0.01 * Rt_pnp.bottomRightCorner(3, 3);
    Rt_pnp.bottomRightCorner(1, 1) = 0.01 * Rt_pnp.bottomRightCorner(1, 1);

    Rt_opt = 0.05 * Rt_opt;



    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    ros::Subscriber s3 = n.subscribe("optical_flow_odom", 1000, optical_flow_odom_callback);

    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    pub_vel_imu = n.advertise<visualization_msgs::Marker>("ekf_imu_velocity", 1, true);
    pub_vel_opt = n.advertise<visualization_msgs::Marker>("ekf_opt_velocity", 1, true);
    Rcam << 0,-1,0,
            -1,0,0,
            0,0,-1;
    //cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.1 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.1 * Q.bottomRightCorner(6, 6);


    ros::spin();
}
