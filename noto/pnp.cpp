#include "armor.h"
#include "/home/liufy/CLionProjects/untitled2/noto/EKF.h"
#include "/home/liufy/CLionProjects/untitled2/noto/tracker.h"
#include "geometry_msgs/msg/quaternion.hpp"  // ROS 2四元数消息类型
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // tf2与ROS消息转换
#include "angles/angles.h"
#include "tf2/convert.h"


void convertRvecToQuaternion(const cv::Mat& rvec, tf2::Quaternion& q) {
    double theta = cv::norm(rvec);
    if (theta < 1e-12) {
        q.setRPY(0, 0, 0);
        return;
    }
    double axis[3];
    for (int i = 0; i < 3; ++i) {
        axis[i] = rvec.at<double>(i) / theta;
    }
    q = tf2::Quaternion(axis[0], axis[1], axis[2], cos(theta / 2));
}

void pnp(Mat camera_matrix,Mat distortion_coeffs, ArmorBox &armor,Mat show) {
    // void pnp(Mat camera_matrix,Mat distortion_coeffs, ArmorBox &armor,Mat show,int fd) {
    vector<Point3f> object_points;
    object_points.push_back(Point3f(-6.15f, -6.25f, 0.f));//左下
    object_points.push_back(Point3f(-6.15f, 6.25f, 0.f));//左上
    object_points.push_back(Point3f(6.15f, 6.25f, 0.f));//右上
    object_points.push_back(Point3f(6.15f, -6.25f, 0.f));//右下

    // 作用：存储装甲板在图像中的像素坐标，通常通过目标检测算法（如轮廓分析）获取。
    // 数据来源：vertices数组应为检测算法输出的角点坐标，需确保与3D点一一对应
    vector<Point2f> image_points;
    image_points.push_back(armor.armorVertices[0]);//左下
    image_points.push_back(armor.armorVertices[1]);//左上
    image_points.push_back(armor.armorVertices[2]);//右上
    image_points.push_back(armor.armorVertices[3]);//右下

    Mat rvec, tvec;
    solvePnP(object_points, image_points, camera_matrix, distortion_coeffs, rvec, tvec,0, SOLVEPNP_ITERATIVE);

    Mat rotation_matrix;
    Rodrigues(rvec, rotation_matrix);  // 转换旋转向量为矩阵

    Mat P_oc = -rotation_matrix.inv() * tvec;// 相机到装甲板

    double GUN_CAM_DISTANCE_X=0;// 枪口到相机的距离
    double GUN_CAM_DISTANCE_Y=0;// 枪口到相机的距离
    tvec.at<double>(0, 0) -= GUN_CAM_DISTANCE_X;  // 修正X轴平移量
    tvec.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y;  // 修正Y轴平移量
    double distance = cv::norm(tvec); // 计算 tvec 的 L2 范数（即距离）

    float x_pitch, y_yaw,pitch,yaw,evaluateDistance;


    if (distance > 5000)
    {
        double x_pos = tvec.at<double>(0, 0);// 相机到装甲板X轴距离
        double y_pos = tvec.at<double>(1, 0);
        double z_pos = tvec.at<double>(2, 0);
        double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
        double tan_yaw = x_pos / z_pos;
        x_pitch = -atan(tan_pitch) * 180 / CV_PI;// 弧度转角度
        y_yaw = atan(tan_yaw) * 180 / CV_PI;// 弧度转角度
    }
    else
    {
        double fx = camera_matrix.at<double>(0, 0);// 相机内参矩阵fx
        double fy = camera_matrix.at<double>(1, 1);// 相机内参矩阵fy
        double cx = camera_matrix.at<double>(0, 2);// 相机内参矩阵cx
        double cy = camera_matrix.at<double>(1, 2);// 相机内参矩阵cy
        Point2f pnt;
        vector<cv::Point2f> in;
        vector<cv::Point2f> out;
        in.push_back(armor.center);// 装甲板中心点

        undistortPoints(in, out, camera_matrix, distortion_coeffs, noArray(), camera_matrix);//  去畸变
        pnt = out.front();

        double rxNew = (pnt.x - cx) / fx;// 相机内参矩阵fx
        double ryNew = (pnt.y - cy) / fy;
        y_yaw = atan(rxNew) * 180 / CV_PI;
        x_pitch= -atan(ryNew) * 180 / CV_PI;
    }
    float camera_target_height = distance * sin(x_pitch / 180 * CV_PI);//  相机到装甲板高度
    float gun_target_height = camera_target_height + GUN_CAM_DISTANCE_Y;//  枪口到装甲板高度
    float gun_pitch_tan = gun_target_height / (distance * cos(x_pitch / 180 * CV_PI));// 枪口到装甲板高度
    x_pitch = atan(gun_pitch_tan) * 180 / CV_PI;

    // 这一块需要射速
    //              float compensateGravity_pitch_tan = tan(x_pitch / 180 * CV_PI) +
    //        (0.5 * 9.788 * (distance / BULLET_SPEED) * (distance / BULLET_SPEED) / cos(x_pitch / 180 * CV_PI));
    //              x_pitch = atan(compensateGravity_pitch_tan) * 180 / CV_PI;
    yaw = y_yaw;
    pitch = x_pitch;
    evaluateDistance = distance;

    string distance1="Distance: " + to_string(distance);
    string pitch1="Pitch: " + to_string(x_pitch);
    string yaw1="Yaw: " + to_string(y_yaw);
    string X="X: " + to_string(tvec.at<double>(0));
    string Y="Y: " + to_string(tvec.at<double>(1));
    string Z="Z: " + to_string(tvec.at<double>(2));

    putText(show,  yaw1, Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
    putText(show,pitch1,Point(50,100),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);
    putText(show,distance1,Point(50,150),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);
    putText(show,X,Point(50,200),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);
    putText(show,Y,Point(50,250),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);
    putText(show,Z,Point(50,300),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);


    tf2::Quaternion q;
    convertRvecToQuaternion(rvec, q);    // 或者使用其他方法转换旋转向量到四元数
    // 直接调用 tf2::toMsg 转换为 ROS 2 的 Quaternion 消息
    geometry_msgs::msg::Quaternion ros_q = tf2::toMsg(q);  // ✅ 正确调用

    // 创建Tracker实例并调用initEKF方法
    rm_auto_aim::Tracker tracker(0, 0);
    tracker.init(tvec, ros_q,armor);




    // write(fd, (const char*)&pitch, sizeof(float));  // 发送4字节的二进制数据
    // write(fd,(const char*)&yaw,sizeof(float));
    //
    // uint8_t buffer[10];
    // buffer[0] = 0xAA;  // 帧头
    //
    // // 将float转为字节数组(考虑字节序)
    // memcpy(&buffer[1], &pitch, sizeof(float));
    // memcpy(&buffer[5], &yaw, sizeof(float));
    //
    // // 计算校验和
    // buffer[9] = 0;
    // for(int i = 1; i < 9; i++) {
    //     buffer[9] ^= buffer[i];
    // }
    //
    // int written = write(fd, buffer, sizeof(buffer));// 发送数据
    // if(written != sizeof(buffer)) {
    //     perror("Serial write incomplete");
    // }
}
