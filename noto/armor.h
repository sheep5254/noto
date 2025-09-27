#ifndef ARMOR_H
#define ARMOR_H
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/msg/quaternion.hpp"  // ROS 2四元数消息类型
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // tf2与ROS消息转换
#include "angles/angles.h"
#include "/home/liufy/CLionProjects/untitled2/noto/EKF.h"


using namespace cv;
using namespace std;
using namespace ml;

#define UART_DEVICE "/dev/ttyUSB0"//默认的串口名称

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class ArmorBox; // 前向声明


class LightBar
{
public:
    LightBar();
    LightBar(const RotatedRect &light);
    ~LightBar();

    RotatedRect lightRect;
    float length;
    Point2f center;
    float angle;
};

class ArmorBox
{
public:
    ArmorBox();
    ArmorBox(const LightBar &l_light,const LightBar &r_light);
    ~ArmorBox();

    // angle difference between two light bars
    float getAngleDiff() const;

    // horizon angle of the line of centers of lights
    float getDeviationAngle() const;

    // distance ratio between the centers of lights in X direction
    float getDislocationX() const;

    // distance ratio between the centers of lights in Y direction
    float getDislocationY() const;

    // length ratio difference between two light bars
    float getLengthRation() const;

    // judge whether this armor is suitable or not
    bool isSuitableArmor() const;
    // double orientationToYaw(const geometry_msgs::msg::Quaternion & q);


public:
    LightBar l_light, r_light;
    int l_index, r_index;
    int armorNum;
    vector<Point2f> armorVertices;
    int type;//0 小 1 大
    Point2f center;
    Rect armorRect;
    float armorAngle;
    Mat armorImg;


    // enum State {
    //     LOST,
    //     DETECTING,
    //     TRACKING,
    //     TEMP_LOST,
    //     CHANGE_TARGET,
    //   } tracker_state;
    // std::string tracked_id;
    // double dz, another_r;
    // int change_thres;
    // double distance_to_image_center; // 距离图像中心的距离
    // geometry_msgs::msg::Pose armor_pose; // 位姿（位置 + 姿态）
    // double last_yaw_;                  // 上一次的偏航角（用于角度连续性处理）
    // Eigen::VectorXd target_state;      // 目标状态向量（如卡尔曼滤波器状态）

};

struct ArmorParam
{
    int color_threshold;
    int bright_threshold;

    int max_light_y_variance;   // light average variance in Y distance
    int max_light_length;    // max light length

    float min_area; // light bar
    float max_angle; // light bar

    float max_angle_diff;   // max angle difference between two lights bars
    float max_lengthDiff_ratio; // max length ratio difference between two light bars
    float max_deviation_angle;  // max horizon angle of the line of centers of lights

    float max_x_diff_ratio; // max distance ratio in X direction
    float max_y_diff_ratio; // max distance ratio in Y direction

    // default values
    ArmorParam()
    {
        color_threshold = 100 - 20;
        bright_threshold = 60;

        max_light_y_variance = 200;
        max_light_length = 50;

        min_area = 50;
        max_angle = 45;

        max_angle_diff = 6;
        max_lengthDiff_ratio = 0.5;
        max_deviation_angle = 50;

        max_x_diff_ratio = 4.5;
        max_y_diff_ratio = 0.5;
    }
};
extern ArmorParam armorParam;


float getPointsDistance(const Point2f &a, const Point2f &b);

void setNumScore(const int &armorNum, const int &targetNum, float &armorScore);

bool armorCompare(const ArmorBox &a_armor, const ArmorBox &b_armor, const ArmorBox &lastArmor, const int &targetNum);

// void pnp(Mat camera_matrix,Mat distortion_coeffs, ArmorBox &armor,Mat show,int fd);
void pnp(Mat camera_matrix,Mat distortion_coeffs, ArmorBox &armor,Mat show);
// double orientationToYaw(const geometry_msgs::msg::Quaternion & q);
// void convertRvecToQuaternion(const cv::Mat& rvec, tf2::Quaternion& q);

void ArmorNum(ArmorBox &armor,Mat warpPerspective_src,Mat showArmors);

Point kalman(KalmanFilter KF,Mat run,  ArmorBox armor,Mat measurement);
ArmorBox armorDetect(RotatedRect predict_rect);


//
// class Tracker
// {
// public:
//     void pnp(cv::Mat camera_matrix, cv::Mat distortion_coeffs, ArmorBox &armor, cv::Mat show);
//     Tracker(double max_match_distance, double max_match_yaw_diff_);
//     // void pnp(Mat camera_matrix, Mat distortion_coeffs, ArmorBox &armor, Mat show);
//
//     // using Armors = auto_aim_interfaces::msg::Armors;
//     // using Armor = auto_aim_interfaces::msg::Armor;
//     //
//     // void init(const Armors::SharedPtr & armors_msg);
//     //
//     // void update(const Armors::SharedPtr & armors_msg);
//
//     // void adaptAngularVelocity(const double & duration);
//
//     rm_auto_aim::ExtendedKalmanFilter ekf;
//
//     int tracking_thres;
//     int lost_thres;
//     int change_thres;
//
//     enum State {
//         LOST,
//         DETECTING,
//         TRACKING,
//         TEMP_LOST,
//         CHANGE_TARGET,
//       } tracker_state;
//
//     std::string tracked_id;
//     std::string last_tracked_id;
//     // Armor tracked_armor;
//     ArmorsNum tracked_armors_num;
//
//     double info_position_diff;
//     double info_yaw_diff;
//
//     Eigen::VectorXd measurement;
//
//     Eigen::VectorXd target_state;
//
//     // To store another pair of armors message
//     double dz, another_r;
//
// private:
//     // void initEKF(const Armor & a);
//     //
//     // void initChange(const Armor & armor_msg);
//     //
//     // void updateArmorsNum(const Armor & a);
//     //
//     // void handleArmorJump(const Armor & a);
//
//     double orientationToYaw(const geometry_msgs::msg::Quaternion & q);
//
//     Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);
//
//     double max_match_distance_;
//
//     double max_match_yaw_diff_;
//
//     int detect_count_;
//     int lost_count_;
//     int change_count_;
//
//     double last_yaw_;
// };


#endif //ARMOR_H
