#include "CameraApi.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "armor.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <Eigen/Dense>
#include "EKF.h"

#include <Eigen/Geometry>


using namespace std;
using namespace cv;
using namespace rm_auto_aim;
using namespace Eigen;

unsigned char* g_pRgbBuffer;

// 状态转移函数 x_k = f(x_{k-1})
Eigen::VectorXd f(const Eigen::VectorXd& x) {
    Eigen::VectorXd x_new = x;
    double dt = 0.033; // 假设30fps，时间间隔33ms

    // 匀速模型（9维状态：[x, vx, y, vy, z, vz, yaw, yaw_rate, r]）
    x_new(0) += x(1) * dt; // x = x + vx*dt
    x_new(2) += x(3) * dt; // y = y + vy*dt
    x_new(4) += x(5) * dt; // z = z + vz*dt
    x_new(6) += x(7) * dt; // yaw = yaw + yaw_rate*dt

    return x_new;
}

// 观测函数 z_k = h(x_k)
Eigen::VectorXd h(const Eigen::VectorXd& x) {
    Eigen::VectorXd z(4); // 测量值为 [x, y, z, yaw]
    z << x(0), x(2), x(4), x(6); // 提取位置和角度
    return z;
}

// 状态转移Jacobian F = ∂f/∂x
Eigen::MatrixXd jacobian_f(const Eigen::VectorXd& x) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    double dt = 0.033;
    F(0, 1) = dt; // ∂f0/∂x1 = dt (vx对x的影响)
    F(2, 3) = dt; // ∂f2/∂x3 = dt (vy对y的影响)
    F(4, 5) = dt; // ∂f4/∂x5 = dt (vz对z的影响)
    F(6, 7) = dt; // ∂f6/∂x7 = dt (yaw_rate对yaw的影响)
    return F;
}

// 观测Jacobian H = ∂h/∂x
Eigen::MatrixXd jacobian_h(const Eigen::VectorXd& x) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 9);
    H(0, 0) = 1; // ∂h0/∂x0 = 1 (x对x的观测)
    H(1, 2) = 1; // ∂h1/∂x2 = 1 (y对y的观测)
    H(2, 4) = 1; // ∂h2/∂x4 = 1 (z对z的观测)
    H(3, 6) = 1; // ∂h3/∂x6 = 1 (yaw对yaw的观测)
    return H;
}

// 过程噪声协方差 Q
Eigen::MatrixXd update_Q() {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9);
    // 调整噪声系数（需要根据实际场景调参）
    Q(0,0) = Q(2,2) = Q(4,4) = 0.1;  // 位置噪声
    Q(1,1) = Q(3,3) = Q(5,5) = 0.5;  // 速度噪声
    Q(6,6) = 0.05; // 角度噪声
    Q(7,7) = 0.1;  // 角速度噪声
    Q(8,8) = 0.01; // 半径噪声
    return Q;
}

// 观测噪声协方差 R
Eigen::MatrixXd update_R(const Eigen::VectorXd& z) {
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    // 调整噪声系数（需要根据实际场景调参）
    R(0,0) = R(1,1) = 10.0; // 像素坐标噪声
    R(2,2) = 1.0;   // z轴噪声
    R(3,3) = 0.1;   // 角度噪声
    return R;
}


int armor_state=0;//0 未发现灯条 1 发现灯条 2 未发现装甲板 3发现装甲板
int enemyColor=0;//0 红 1 蓝
int targetNum=0;

int main() {


    // 初始化EKF
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(9, 9);
    rm_auto_aim::ExtendedKalmanFilter ekf(f, h, jacobian_f, jacobian_h, update_Q, update_R, P0);

    // // 跟踪状态变量
    // bool is_tracking = false;
    // int lost_count = 0;
    // const int max_lost_frames = 5; // 最大丢失帧数
    //
    //  //串口初始化
    // int fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    // if (fd == -1) {
    //     perror("open failed");
    //     return -1;
    // }
    // struct termios options;
    // tcgetattr(fd, &options);  // 获取当前配置
    //
    // // 设置波特率（例如 115200）
    // cfsetispeed(&options, B115200);
    //
    //
    // // 数据位（8位）、无校验、1位停止位
    // options.c_cflag &= ~PARENB;  // 禁用校验
    // options.c_cflag &= ~CSTOPB;  // 1位停止位
    // options.c_cflag &= ~CSIZE;   // 清除数据位掩码
    // options.c_cflag |= CS8;      // 8位数据位
    //
    // // 应用配置
    // tcsetattr(fd, TCSANOW, &options);



    // int stateNum=4,  measureNum = 2;
    // KalmanFilter KF = KalmanFilter(stateNum, measureNum, 0);
    // Mat state = Mat(stateNum, 1, CV_32FC1);
    // Mat processNoise = Mat(stateNum, 1, CV_32F);
    // Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    //
    // randn(state, Scalar::all(0), Scalar::all(0.1));
    //
    // //!< state transition matrix (A)
    // KF.transitionMatrix = (Mat_<float>(4, 4) <<
    //                     1, 0, 1, 0,
    //                     0, 1, 0, 1,
    //                     0, 0, 1, 0,
    //                     0, 0, 0, 1);
    //
    // //!< measurement matrix (H)
    // setIdentity(KF.measurementMatrix);
    //
    // //!< process noise covariance matrix (Q)
    // KF.processNoiseCov *= 1;
    // setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
    //
    // //!< measurement noise covariance matrix (R)
    // KF.measurementNoiseCov *= 1;
    // setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    //
    // //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
    // KF.errorCovPost *= 1;
    // setIdentity(KF.errorCovPost, Scalar::all(1));
    //
    // //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    // Mat control = -KF.gain * (measurement - KF.measurementMatrix * KF.statePost);
    //
    // // !< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    // randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

    int iCameraCounts = 1;// 相机数量
    int iStatus = -1;// 相机状态
    tSdkCameraDevInfo tCameraEnumList;//`
    int hCamera;
    tSdkCameraCapbility tCapability;
    tSdkFrameHead sFrameInfo;
    BYTE* pbyBuffer;
    int iDisplayFrames = 10000;
    IplImage* iplImage = NULL;
    int channel = 3;

    CameraSdkInit(1);// 初始化SDK库

    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    if (iCameraCounts == 0) return -1;

    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    if (iStatus != CAMERA_STATUS_SUCCESS) return -1;

    CameraGetCapability(hCamera, &tCapability);
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    // 关键修复1：设置合理的曝光时间和白平衡防止过曝
    CameraSetAeState(hCamera, TRUE);      // 开启自动曝光
    //CameraSetAeExposureRange(hCamera, 500, 3000); // 设置曝光上下限（μs）
    CameraSetExposureTime(hCamera, 1000);  // 设置曝光时间1000μs（根据环境调整）
    //CameraSetWbMode(hCamera, CAMERA_WB_MODE_AUTO); // 自动白平衡
    CameraPlay(hCamera);

    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8); // 确保输出BGR格式
    }

    Mat camera_matrix = (Mat_<double>(3, 3) << 712.9215287852671,0.0,307.8615217208748,
        0.0,714.6431520641138, 256.8790022083954 ,0.0,0.0,  1.0);
    Mat distortion_coeffs = (Mat_<double>(5, 1) <<  -0.447569535375274, -0.000446903384808, 0.000278897924377, 0.001563208302172);
    while (iDisplayFrames--) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            // 关键修复2：使用正确的图像尺寸创建Mat对象
            cv::Mat matImage(
                cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                (sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8) ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
            );
            Mat srcImg,srcImg_binary,showLight,showArmors,show,run;
            Mat warpPerspective_src;
            vector<ArmorBox> armors;
            ArmorBox targetArmor,lastArmor;
            vector<LightBar> lights;
            vector<vector<Point>> lightContours;
            LightBar light;


            show=cv::Mat::zeros(400,400,CV_8UC3);//  显示图片
            matImage.copyTo(showLight);
            matImage.copyTo(showArmors);
            matImage.copyTo(srcImg);
            matImage.copyTo(warpPerspective_src);
            matImage.copyTo(run);

            cvtColor(srcImg,srcImg_binary,CV_BGR2GRAY);
            uchar *pdata = (uchar *) srcImg.data;
            uchar *qdata = (uchar *) srcImg_binary.data;
            int srcData = srcImg.rows * srcImg.cols;
            if (enemyColor == 1)
            {
                for (int i = 0; i < srcData; i++)
                {
                    if (*(pdata + 2) - *pdata > (100-20))
                        *qdata = 255;
                    pdata += 3;
                    qdata++;
                }
            }
            else if (enemyColor == 0)
            {
                for (int i = 0; i < srcData; i++)
                {
                    if (*pdata - *(pdata + 2) > (100-20))
                        *qdata = 255;
                    pdata += 3;
                    qdata++;
                }
            }

            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
            threshold(srcImg_binary,srcImg_binary,192,255,THRESH_BINARY);
            dilate(srcImg_binary, srcImg_binary, kernel);

            findContours(srcImg_binary,lightContours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
            RotatedRect  lightRect;

            for (const auto &lightContour : lightContours) {
                if (lightContour.size()<5) continue;
                if (contourArea(lightContour)<50) continue;
                lightRect = fitEllipse(lightContour);// 输入轮廓 输出旋转矩形（中心点xy，宽高，角度）
                light = LightBar(lightRect);
                if (abs(light.angle)>45) continue;

                if (!lights.empty()) {
                    int sum_y=0;
                    for (const auto light : lights) {
                        sum_y+=light.center.y;
                    }
                    int mean=sum_y/lights.size();// 计算平均值

                    int variance=0;//  方差
                    for (const auto light : lights) {
                        int deviation=light.center.y-mean;// 偏差
                        variance+=deviation*deviation;// 方差
                    }
                    variance/=lights.size();// 方差

                    if (variance>200) {
                        auto it = lights.begin();
                        while (it != lights.end()) {
                            if (it->center.y < mean - 100&&  it->length<50) {
                                it=lights.erase(it);
                            }
                            else {
                                ++it;
                            }
                        }
                    }
                }
                lights.push_back(light);


                if (lights.size()<2) {
                    continue;
                }

                if (!lights.empty()) {
                    for (auto light: lights)
                    {
                        Point2f lightVertices[4];//  创建一个数组，用于存储矩形的四个顶点
                        light.lightRect.points(lightVertices);// 获取矩形的四个顶点
                        for (size_t i = 0; i < 4; i++)
                        {
                            line(showLight, lightVertices[i], lightVertices[(i + 1) % 4], Scalar(255, 0, 255), 2, 8, 0);
                        }
                    }

                    sort(lights.begin(), lights.end(), [](const LightBar& a, const LightBar& b) {
                        return a.center.x < b.center.x;// 按x坐标升序排序
                    });
                    armor_state=1;
                }

                for (int i = 0; i < lights.size() - 1; i++) // todo time complexity problem
                {
                    for (int j = i + 1; j < lights.size(); j++)
                    {
                        ArmorBox armor = ArmorBox(lights[i], lights[j]);
                        if (armor.isSuitableArmor())
                        {
                            armor.l_index = i;//  记录灯条索引
                            armor.r_index = j;
                            ArmorNum(armor,warpPerspective_src,showArmors);
                            armors.emplace_back(armor);
                        }
                    }
                    int length = armors.size(); // todo: size_t
                    vector<ArmorBox>::iterator it = armors.begin();
                    for (size_t i = 0; i < length; i++)
                    {
                        for (size_t j = i + 1; j < length; j++)
                        {
                            if (armors[i].l_index == armors[j].l_index ||
                            armors[i].l_index == armors[j].r_index ||
                            armors[i].r_index == armors[j].r_index ||
                            armors[i].r_index == armors[j].l_index)
                            {
                                armors[i].getDeviationAngle() > armors[j].getDeviationAngle() ?
                                it = armors.erase(it + i) : it = armors.erase(it + j);//  按角度排序
                            }
                        }
                    }
                }
                if (armors.empty())
                {
                    armor_state = 2;
                } else
                {
                    armor_state = 3;
                }

                if (armor_state == 2) {
                    targetArmor = ArmorBox();// 清空装甲板
                }
                else if (armor_state == 3)
                {
                    ArmorBox mva = armors[0];
                    for (int i = 1; i < armors.size(); i++)
                    {
                        if (armorCompare(armors[i], mva, lastArmor, targetNum)) mva = armors[i];//  选择装甲板
                    }
                    targetArmor = mva;// 将得分最高的装甲板设置为目标装甲板

                    // // 准备测量值 [x, y, z, yaw]
                    // Eigen::Vector4d measurement;
                    // measurement << targetArmor.center.x, targetArmor.center.y,
                    //                /* z */ 0, /* yaw */ targetArmor.armorAngle;
                    //
                    //
                    // if (is_tracking) {// 如果正在跟踪
                    //     // 1. EKF预测步骤
                    //     Eigen::VectorXd predicted_state = ekf.predict();
                    //
                    //     // 2. EKF更新步骤
                    //     Eigen::VectorXd updated_state = ekf.update(measurement);
                    //
                    //     // 3. 使用滤波结果
                    //     Point2f filtered_pos(updated_state(0), updated_state(2));
                    //     float filtered_yaw = updated_state(6);
                    //
                    //     // 绘制滤波结果
                    //     circle(run, filtered_pos, 5, Scalar(0, 0, 255), 2); // 红色表示滤波结果
                    //
                    //     // 重置丢失计数器
                    //     lost_count = 0;
                    //
                    // } else {// 如果未跟踪
                    //     // 初始化EKF状态
                    //     Eigen::VectorXd x0(9);
                    //     x0 << targetArmor.center.x, 0, targetArmor.center.y, 0,
                    //           0, 0, targetArmor.armorAngle, 0, 0.2;
                    //     ekf.setState(x0);
                    //     is_tracking = true;
                    // }

                    lastArmor = targetArmor;
                }

                if (!armors.empty()) {
                        for (auto armor : armors) {
                            //circle(showArmors, armor.center, 2, Scalar(0, 255, 0), 2);// 画出装甲板中心点
                            for (size_t i = 0; i < 4; i++)
                            {
                                line(showArmors, armor.armorVertices[i], armor.armorVertices[(i + 1) % 4],
                                Scalar(255, 255, 0), 1, 8, 0);

                            }
                            string text = "ID: " + to_string(armor.armorNum);
                            putText(showArmors, text, armor.center,FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);// 显示装甲板ID


                            circle(showArmors,targetArmor.center,5,Scalar(255,255,0),2);
                        }
                            // //卡尔曼滤波
                            // Point2f predict=kalman(KF,run,targetArmor,measurement);
                            //
                            // cv::Size2f armor_size(static_cast<float>(targetArmor.armorRect.width), static_cast<float>(targetArmor.armorRect.height));
                            // RotatedRect predict_rect(predict,armor_size,targetArmor.armorAngle);
                            // ArmorBox predicted_armor= armorDetect(predict_rect);


                            // pnp(camera_matrix,distortion_coeffs,targetArmor,show,fd);
                            pnp(camera_matrix,distortion_coeffs,targetArmor,show);



                        }
                    }
                    imshow("showArmors", showArmors);
                    // imshow("showLight",showLight);
                    imshow("Camera Output", matImage);
                    imshow("show",show);
                    imshow("run", run);

                    waitKey(1);

                    CameraReleaseImageBuffer(hCamera, pbyBuffer);
                }
            }


            // 关键修复6：资源释放在循环结束后
            CameraUnInit(hCamera);
            free(g_pRgbBuffer);
            // close(fd);

            return 0;
}