#include "armor.h"

Point kalman(KalmanFilter KF,Mat run,  ArmorBox armor,Mat measurement)
{
    Point statePt = Point((int) KF.statePost.at<float>(0), (int) KF.statePost.at<float>(1));
    Mat prediction = KF.predict();
    Point predictPt = Point((int) prediction.at<float>(0), (int) prediction.at<float>(1));

    measurement.at<float>(0) = (float) armor.center.x;
    measurement.at<float>(1) = (float) armor.center.y;

    KF.correct(measurement);

    resize(run,run,Size(640,480));
    circle(run, statePt, 5, Scalar(0,0,255), 2);
    circle(run, predictPt, 5, Scalar(255, 255, 0), 2);
    return predictPt;
}

ArmorBox armorDetect(RotatedRect predict_rect) {
    std::vector<cv::Point2f> vertices(4);
    predict_rect.points(vertices.data());

    // 生成左右灯条的旋转矩形
    cv::RotatedRect left_light = cv::minAreaRect(std::vector<cv::Point2f>{vertices[0], vertices[1]});
    cv::RotatedRect right_light = cv::minAreaRect(std::vector<cv::Point2f>{vertices[2], vertices[3]});

    // 构造 LightBar 对象（需根据实际 LightBar 类调整）
    LightBar l_light(left_light);
    LightBar r_light(right_light);

    // 最终生成 ArmorBox
    ArmorBox predicted_armor(l_light, r_light);
    return predicted_armor;

}

