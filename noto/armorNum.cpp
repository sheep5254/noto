#include "armor.h"

void ArmorNum(ArmorBox &armor,Mat warpPerspective_src,Mat showArmors) {

    Ptr<ml::SVM> svm = ml::SVM::load("/home/liufy/CLionProjects/untitled/General/123svm.xml");
    if (svm.empty()) {
        cerr << "Failed to load SVM model!" << endl;
    }
    cvtColor(warpPerspective_src, warpPerspective_src, 6);  // CV_BGR2GRAY=6
    threshold(warpPerspective_src, warpPerspective_src, 10, 255, THRESH_BINARY);

    Point2f dstPoints[4], srcPoints[4];
    Size armorImgSize = Size(40, 40);// 装甲板图像尺寸
    Mat warpPerspective_mat,warpPerspective_dst,p;
    p = Mat();// 创建一个空的Mat对象
    warpPerspective_mat = Mat(3, 3, CV_32FC1);// 创建3x3的浮点矩阵
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(armorImgSize.width, 0);
    dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
    dstPoints[3] = Point2f(0, armorImgSize.height);
    for (int i = 0; i < 4; i++) {
        srcPoints[i] = armor.armorVertices[i];
    }
    warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints); // 计算透视变换矩阵
    warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, armorImgSize,
                    INTER_NEAREST, BORDER_CONSTANT, Scalar(0));// 透视变换
    warpPerspective_dst.copyTo(armor.armorImg);
    p = armor.armorImg.reshape(1, 1);
    p.convertTo(p, CV_32FC1);

    armor.armorNum = (int) svm->predict(p);

}
