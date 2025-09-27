
#ifndef UNTITLED2_TRACKER_H
#define UNTITLED2_TRACKER_H
// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>
#include "/home/liufy/CLionProjects/untitled2/noto/armor.h"
#include "/home/liufy/CLionProjects/untitled2/noto/EKF.h"
// #include "armor_tracker/extended_kalman_filter.hpp"
// #include "auto_aim_interfaces/msg/armors.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

    enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

    class Tracker
    {
    public:
        Tracker(double max_match_distance, double max_match_yaw_diff_);

        // using Armors = auto_aim_interfaces::msg::Armors;
        // using Armor = auto_aim_interfaces::msg::Armor;

        // void init(const Armors::SharedPtr & armors_msg);
        void init(const Mat tvec,geometry_msgs::msg::Quaternion ros_q,ArmorBox &armor);
        void initEKF(const Mat tvec,geometry_msgs::msg::Quaternion ros_q);
        //
        // void update(const Armors::SharedPtr & armors_msg);

        void adaptAngularVelocity(const double & duration);

        ExtendedKalmanFilter ekf;

        int tracking_thres;
        int lost_thres;
        int change_thres;

        enum State {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
            CHANGE_TARGET,
          } tracker_state;

        std::string tracked_id;
        std::string last_tracked_id;
        // Armor tracked_armor;
        ArmorsNum tracked_armors_num;

        double info_position_diff;
        double info_yaw_diff;

        Eigen::VectorXd measurement;

        Eigen::VectorXd target_state;

        // To store another pair of armors message
        double dz, another_r;

        // 将 orientationToYaw 移到 public 部分以供外部访问
        double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

    private:
        // void initEKF(const Armor & a);
        //
        // void initChange(const Armor & armor_msg);
        //
        void updateArmorsNum(const ArmorBox & a);
        //
        // void handleArmorJump(const Armor & a);

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

        double max_match_distance_;

        double max_match_yaw_diff_;

        int detect_count_;
        int lost_count_;
        int change_count_;

        double last_yaw_;
};

}  // namespace rm_auto_aim
#endif //UNTITLED2_TRACKER_H