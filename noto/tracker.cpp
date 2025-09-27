#include "tracker.h"
#include "armor.h"

namespace rm_auto_aim {

    Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
        : max_match_distance_(max_match_distance),
          max_match_yaw_diff_(max_match_yaw_diff) {
        // 初始化成员变量
        tracking_thres = 0;
        lost_thres = 0;
        change_thres = 0;

        tracker_state = LOST;

        tracked_id = "";
        last_tracked_id = "";

        info_position_diff = 0.0;
        info_yaw_diff = 0.0;

        target_state = Eigen::VectorXd::Zero(9);
        measurement = Eigen::VectorXd::Zero(4);

        dz = 0.0;
        another_r = 0.0;

        detect_count_ = 0;
        lost_count_ = 0;
        change_count_ = 0;

        last_yaw_ = 0.0;
    }

    void Tracker::init(cv::Mat tvec, geometry_msgs::msg::Quaternion ros_q, ArmorBox& armor) {
        double xa = tvec.at<double>(0, 0);
        double ya = tvec.at<double>(1, 0);
        double za = tvec.at<double>(2, 0);
        last_yaw_ = 0;

        double yaw_y = orientationToYaw(ros_q);

        // Set initial position at 0.2m behind the target
        target_state = Eigen::VectorXd::Zero(9);
        double r = 0.2;
        double xc = xa + r * cos(yaw_y);
        double yc = ya + r * sin(yaw_y);
        dz = 0;
        another_r = r;
        target_state << xc, 0, yc, 0, za, 0, yaw_y, 0, r;

        ekf.setState(target_state);

        tracked_id = std::to_string(armor.armorNum);
        tracker_state = DETECTING;// 初始化状态为检测中

        change_count_ = 0;// 初始化连续丢失计数器
        change_thres = 20;// 连续丢失阈值
        updateArmorsNum(armor);

    }
    void Tracker::updateArmorsNum(const ArmorBox & armor)
    {
        if (armor.type == 1 && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
            tracked_armors_num = ArmorsNum::BALANCE_2;
        } else if (tracked_id == "1") {  //哨兵为1
            tracked_armors_num = ArmorsNum::OUTPOST_3;
        } else {
            tracked_armors_num = ArmorsNum::NORMAL_4;
        }
    }

} // namespace rm_auto_aim