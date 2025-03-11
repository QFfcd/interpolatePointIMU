#include "ImageStabilizationUpdate.h"
#include <fstream>
#include <cmath>
#include <cstdio>
#include <QCoreApplication>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <QVector>

using namespace std;
using namespace cv;
ImageStabilizationUpdate::ImageStabilizationUpdate()
{
    /* Phan nay in ket qua ra log file de check voi ket qua matlab*/
    const char* filename = "/home/ngoclth/Documents/stabImg/stabUAV/targetEstimate.txt";
    const char* mode = "w";


    fp=fopen(filename,mode);

    assert(fp != nullptr);
    Target_x = 0;
    Target_y = 0;
    count = 0;
    transformMat = cv::Mat::eye(2, 3, CV_64F);
}

ImageStabilizationUpdate::~ImageStabilizationUpdate()
{
    fclose(fp);
    std::cout << "Ket thuc";
}
/*Ham nay de loai tru cac truong hop bi nhay bac tu 180<>-180
  - thres: muc threshold nhay bac
  - tolerance phan bu vao */
double ImageStabilizationUpdate::unwrap(double prevIMU, double currIMU, double thres, double tolerance)
{
    if (currIMU - prevIMU > thres)
    {
        currIMU -= 2 * tolerance;
    }
    if (currIMU - prevIMU < -thres)
    {
        currIMU += 2 * tolerance;
    }
    return currIMU;
}
/* Ham on dinh anh su dung thong tin IMU khong noi suy*/
std::vector<double>ImageStabilizationUpdate::readDataIMUNonInterpolate(std::ifstream& fileIMU_60Hz)
{
    double rollIMU_60Hz, pitchIMU_60Hz, yawIMU_60Hz, hFOV60Hz;
    fileIMU_60Hz >> rollIMU_60Hz >> pitchIMU_60Hz >> yawIMU_60Hz >> hFOV60Hz;
    std::vector<double> outputIMUValue;

    outputIMUValue.push_back(rollIMU_60Hz);
    outputIMUValue.push_back(pitchIMU_60Hz);
    outputIMUValue.push_back(yawIMU_60Hz);
    outputIMUValue.push_back(hFOV60Hz);

    return outputIMUValue;

}
/* Ham on dinh anh IMU co noi suy*/
std::vector<double>ImageStabilizationUpdate::readDataIMUInterpolate(std::ifstream& fileIMU_60Hz, std::ifstream& fileIMU_100Hz)
{
       std::vector<double> outputIMUValue;
       static double timestampBegin;

       double rollIMU_60Hz, pitchIMU_60Hz, yawIMU_60Hz, hFOV60Hz;
       double rollIMU_100Hz, pitchIMU_100Hz, yawIMU_100Hz;
       static double timeStam_100Hz;
       // cac gia tri bien roll pitch yaw
       static std::vector<double> arrayRoll_100Hz, arrayPitch_100Hz, arrayYaw_100Hz;
       // bien chua thong tin timestamp
       static std::vector<double> arrayTimeStam_100Hz;
       // bien nay danh cho luu tru thong tin 1000Hz
       std::vector<double> arrayRoll_1000Hz, arrayPitch_1000Hz, arrayYaw_1000Hz, arrayTimeStam_1000Hz;
       double IMUInterval_100Hz = 1.0 / 100;
       double IMUInterpolateInterval_1000Hz = 1.0 / 1000;
       double timeStamp_60Hz = count * 1.0 / 60;
       fileIMU_60Hz >> rollIMU_60Hz >> pitchIMU_60Hz >> yawIMU_60Hz >> hFOV60Hz;
       if (count == 0)
       {
           outputIMUValue.push_back(rollIMU_60Hz);
           outputIMUValue.push_back(pitchIMU_60Hz);
           outputIMUValue.push_back(yawIMU_60Hz);
           outputIMUValue.push_back(hFOV60Hz);

           cout << rollIMU_60Hz << "," << pitchIMU_60Hz << "," << yawIMU_60Hz << "," << hFOV60Hz << endl;

           while (arrayRoll_100Hz.size() == 0)
           {
               //             dem++;
               fileIMU_100Hz >> rollIMU_100Hz >> pitchIMU_100Hz >> yawIMU_100Hz >> timeStam_100Hz;
               //cout<<std::abs(rollIMU_100Hz-rollIMU_60Hz)<<" "<<std::abs(pitchIMU_100Hz-pitchIMU_60Hz)<<" "<<std::abs(yawIMU_100Hz-yawIMU_60Hz)<<endl;
               cout << rollIMU_100Hz << pitchIMU_100Hz << yawIMU_100Hz << endl;
               if (std::abs(rollIMU_100Hz - rollIMU_60Hz) < 0.001)
               {
                   cout << "Da vao day" << endl;
                   timestampBegin = timeStam_100Hz;
                   arrayRoll_100Hz.push_back(rollIMU_100Hz);
                   arrayPitch_100Hz.push_back(pitchIMU_100Hz);
                   arrayYaw_100Hz.push_back(yawIMU_100Hz);
                   arrayTimeStam_100Hz.push_back(0);
                   break;
               }
           }
       }
       else
       {
           /*Phan xu ly 100Hz->1000Hz */
           while ((timeStam_100Hz - timestampBegin) / 1000 <= timeStamp_60Hz+1.0/600000) {
              fileIMU_100Hz >> rollIMU_100Hz >> pitchIMU_100Hz >> yawIMU_100Hz >> timeStam_100Hz;
              arrayRoll_100Hz.push_back(unwrap(arrayRoll_100Hz.back(), rollIMU_100Hz, 100, 180));
              arrayPitch_100Hz.push_back(unwrap(arrayPitch_100Hz.back(), pitchIMU_100Hz, 100, 180));
              arrayYaw_100Hz.push_back(unwrap(arrayYaw_100Hz.back(), yawIMU_100Hz, 100, 180));
              arrayTimeStam_100Hz.push_back((timeStam_100Hz - timestampBegin) / 1000);
              cout << timeStam_100Hz << "Ung voi: " << timeStamp_60Hz*1000<<endl;
           }
           // do doi voi moi mo diem 60Hz se nam trong 3 diem noi suy tan so 100Hz
           int bufferSize = 5;
           while (arrayRoll_100Hz.size() > bufferSize) {
               arrayRoll_100Hz.erase(arrayRoll_100Hz.begin());
               arrayPitch_100Hz.erase(arrayPitch_100Hz.begin());
               arrayYaw_100Hz.erase(arrayYaw_100Hz.begin());
               arrayTimeStam_100Hz.erase(arrayTimeStam_100Hz.begin());
           }
           cout << "Kich thuoc vector sau khi xoa: "<<arrayRoll_100Hz.size() << endl;

           // interpolation len 1000Hz do chi co 5 phan tu=> noi suy ra 30 mau sau do gan mau gan voi gia tri timestamp 60Hz nhat
           for (size_t i = 0; i < arrayRoll_100Hz.size() - 1; ++i) {
               // Add the current original data point to the result
               arrayRoll_1000Hz.push_back(arrayRoll_100Hz[i]);
               arrayPitch_1000Hz.push_back(arrayPitch_100Hz[i]);
               arrayYaw_1000Hz.push_back(arrayYaw_100Hz[i]);
               arrayTimeStam_1000Hz.push_back(arrayTimeStam_100Hz[i]);
               // Calculate the number of new points needed between current and next original data point
               int numNewPoints = round((arrayTimeStam_100Hz[i+1]- arrayTimeStam_100Hz[i])/ IMUInterpolateInterval_1000Hz) - 1;

               // Linearly interpolate points between the current and next data points
               for (int j = 1; j <= numNewPoints; ++j) {
                   double t = j * 1.0/ (numNewPoints+1);  // Fraction between points
                   double roll_1000Hz = arrayRoll_100Hz[i] * (1 - t) + arrayRoll_100Hz[i + 1] * t;
                   double pitch_1000Hz = arrayPitch_100Hz[i] * (1 - t) + arrayPitch_100Hz[i + 1] * t;
                   double yaw_1000Hz = arrayYaw_100Hz[i] * (1 - t) + arrayYaw_100Hz[i + 1] * t;
                   double timeStamp_1000Hz = arrayTimeStam_100Hz[i] * (1 - t) + arrayTimeStam_100Hz[i + 1] * t;

                   arrayRoll_1000Hz.push_back(roll_1000Hz);
                   arrayPitch_1000Hz.push_back(pitch_1000Hz);
                   arrayYaw_1000Hz.push_back(yaw_1000Hz);
                   arrayTimeStam_1000Hz.push_back(timeStamp_1000Hz);
               }
           }
           // Add the last data point
           arrayRoll_1000Hz.push_back(arrayRoll_100Hz.back());
           arrayPitch_1000Hz.push_back(arrayPitch_100Hz.back());
           arrayYaw_1000Hz.push_back(arrayYaw_100Hz.back());
           arrayTimeStam_1000Hz.push_back(arrayTimeStam_100Hz.back());

           /* Noi suy tu 1000Hz ve 60Hz
           Match ve mat thoi gian */

           int indexNearestPoint = 0;
           double distance = std::abs(timeStamp_60Hz - arrayTimeStam_1000Hz[0]);
           for (int i = 0; i < arrayTimeStam_1000Hz.size(); i++)
           {

               if (std::abs(timeStamp_60Hz - arrayTimeStam_1000Hz[i]) < distance)
               {
                   distance = std::abs(timeStamp_60Hz - arrayTimeStam_1000Hz[i]);
                   indexNearestPoint = i;
               }
           }
           outputIMUValue.push_back(arrayRoll_1000Hz[indexNearestPoint]);
           outputIMUValue.push_back(arrayPitch_1000Hz[indexNearestPoint]);
           outputIMUValue.push_back(arrayYaw_1000Hz[indexNearestPoint]);
           outputIMUValue.push_back(hFOV60Hz);
       }
       count++;
       roll100HzInterpolate = outputIMUValue[0];
       pitch100HzInterpolate=outputIMUValue[1];
       yaw100HzInterpolate = outputIMUValue[2];

//       fprintf(fp, "%f %f %f %f %f %f \n", outputIMUValue[0], outputIMUValue[1], outputIMUValue[2],deltaRoll,deltaPitch,deltaYaw);
       return outputIMUValue;
}
void ImageStabilizationUpdate::processIMUdata(double roll, double pitch, double yaw, double hfov, int sizeArray)
{

    // calib ma tran dau
    pitch = -pitch;
    // interpolation IMU
    if (RollArray.empty())
    {
        // roll, pitch, yaw
        F2Finfo[0] = 0;
        F2Finfo[1] = 0;
        F2Finfo[2] = 0;
        this->hFov = hfov;
        this->ScaleImage = 1;
        RollArray.push_back(0);
        PitchArray.push_back(0);
        YawArray.push_back(0);
    }
    else
    {
        deltaYaw = yaw - YawPrevious;
        deltaPitch = pitch - PitchPrevious;
        deltaRoll = roll - RollPrevious;

        F2Finfo[2] = tan((yaw - YawPrevious) * M_PI / 180);
        F2Finfo[1] = tan((pitch - PitchPrevious) * M_PI / 180);
        F2Finfo[0] = roll - RollPrevious;

        RollArray.push_back(RollArray.back() + F2Finfo[0]);
        PitchArray.push_back(PitchArray.back() + F2Finfo[1]);
        YawArray.push_back(YawArray.back() + F2Finfo[2]);

        this->ScaleImage = tan(this->hFov * M_PI / 360) / tan(hfov * M_PI / 360);// Gia su neu Fov toi <Fov frame truoc=> zoom in=>scale>1;
        this->hFov = hfov;
    }

    if (this->RollArray.size() > sizeArray) {
        RollArray.erase(RollArray.begin());
        PitchArray.erase(PitchArray.begin());
        YawArray.erase(YawArray.begin());
    }
    float alphaSmoothX = 0.02;
    float alphaSmoothY = 0.02;
    float alphaSmoothRoll = 0.02;

    std::vector<float> xTrajectoryArraySmoothIMU(1, 0);
    std::vector<float> yTrajectoryArraySmoothIMU(1, 0);
    std::vector<float> rollTrajectoryArraySmoothIMU(1, 0);

    for (int i = 0; i < RollArray.size(); i++) {
        xTrajectoryArraySmoothIMU.push_back(alphaSmoothX * YawArray[i] + (1 - alphaSmoothX) * xTrajectoryArraySmoothIMU[i]);
        yTrajectoryArraySmoothIMU.push_back(alphaSmoothY * PitchArray[i] + (1 - alphaSmoothY) * yTrajectoryArraySmoothIMU[i]);
        rollTrajectoryArraySmoothIMU.push_back(alphaSmoothRoll * RollArray[i] + (1 - alphaSmoothRoll) * rollTrajectoryArraySmoothIMU[i]);
    }
    txIMU = xTrajectoryArraySmoothIMU.back() - YawArray.back();
    tyIMU = yTrajectoryArraySmoothIMU.back() - PitchArray.back();
    rollIMU = rollTrajectoryArraySmoothIMU.back() - RollArray.back();

    RollPrevious = roll;
    PitchPrevious = pitch;
    YawPrevious = yaw;
}
/* Doc feature extraction */
void ImageStabilizationUpdate::readFeatureExtract(cv::Mat framePrevious, cv::Mat frameCurrent)
{
    if (framePrevious.empty() || frameCurrent.empty()) {
        std::cerr << "Error: Could not read the frame!" << std::endl;
    }
    cv::Mat  prevFrameGray, currFrameGray;
    std::vector<cv::Point2f> prevCorners, currCorners;
    std::vector<uchar> status;
    std::vector<float> err;

    // Convert the current frame to grayscale
    cv::cvtColor(framePrevious, prevFrameGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frameCurrent, currFrameGray, cv::COLOR_BGR2GRAY);

    goodFeaturesToTrack(prevFrameGray, prevCorners, 100, 0.01, 10, cv::Mat(), 3, true, 0.04);

    // Variables for stabilization
    calcOpticalFlowPyrLK(prevFrameGray, currFrameGray, prevCorners, currCorners, status, err);

    // Filter out bad points
    std::vector<cv::Point2f> prevGoodPoints, currGoodPoints;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            prevGoodPoints.push_back(prevCorners[i]);
            currGoodPoints.push_back(currCorners[i]);
        }
    }
    // Estimate transformation matrix (affine transformation)
//    if (prevGoodPoints.size() >= 16) {
//        transformMat = cv::estimateAffine2D(prevGoodPoints, currGoodPoints);
//        std::cout << "Su dung FeatureExtraction" << std::endl;
//        // ve cac anh feature point de xem hieu qua cua feature exxtra
//    }
//    else {
        cv::Point2f centerImage(static_cast<float> (1920 / 2), static_cast<float>(1080 / 2));
        float f = framePrevious.cols / (2 * tan(hFov * M_PI / 360));
        cv::Mat affineMatrix = cv::getRotationMatrix2D(centerImage, -F2Finfo[0], ScaleImage);
        affineMatrix.at<double>(0, 2) += -F2Finfo[2] * f; // x
        affineMatrix.at<double>(1, 2) += -F2Finfo[1] * f;// y
        transformMat = affineMatrix;
//    }
}
void ImageStabilizationUpdate::imwarpPoint(double Target_x_FrameBefore, double Target_y_FrameBefore)
{
    // dua vector diem ve vector gan voi he toa do tam xoay anh
    // chinh offset khi tang muc zoom.
    cv::Mat p = (cv::Mat_<double>(3, 1) << (Target_x_FrameBefore), (Target_y_FrameBefore), 1);
    //std::cout << ScaleImage<<std::endl;
    cv::Mat p_prime = transformMat * cv::Mat(p);
    // for debugger
    std::vector<double>matrixRaw(transformMat.begin<double>(), transformMat.end<double>());
    std::vector<double>vecPoinRawp(p.begin<double>(), p.end<double>());
    std::vector<double>vecPoinRawp_prime(p_prime.begin<double>(), p_prime.end<double>());
    // Biến đổi điểm ảnh
    // loi do khong match chieu
    Target_x = p_prime.at<double>(0);
    Target_y = p_prime.at<double>(1);
    fprintf(fp, "%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f \n", Target_x_FrameBefore, Target_y_FrameBefore, Target_x, Target_y, roll100HzInterpolate, -pitch100HzInterpolate, yaw100HzInterpolate, deltaYaw, deltaPitch, deltaRoll);
//    fprintf(fp, "%f %f \n", Target_x, Target_y);
}
cv::Mat ImageStabilizationUpdate::stabilizing(cv::Mat frame)
{
    if (frame.cols <= 0 || frame.rows <= 0) {
        std::cout << "Invalid image dimensions!" << std::endl;
        return frame;
    }
    float f = frame.cols / (2 * tan(hFov * 3.1416 / 360));

    /* Phan nay xu ly du lieuIMU*/

    // Define translation matrix
    cv::Point2f center(static_cast<float>((frame.cols) / 2), static_cast<float>((frame.rows) / 2));

    cv::Mat affineMatrix = cv::getRotationMatrix2D(center, -rollIMU, ScaleImage);
    affineMatrix.at<double>(0, 2) += txIMU * f;
    affineMatrix.at<double>(1, 2) += tyIMU * f;
    cv::Mat frameStabled;
    cv::warpAffine(frame, frameStabled, affineMatrix, frame.size(), cv::WARP_INVERSE_MAP | cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return frameStabled;
}

cv::Mat ImageStabilizationUpdate::stabilizingUsingmatrixLog(cv::Mat frame, double const* DataMatrix)
{
    if (frame.cols <= 0 || frame.rows <= 0) {
        std::cout << "Invalid image dimensions!" << std::endl;
        return frame;
    }
    count++;
    cv::Mat affineMatrix = cv::Mat_<double>(2, 3);
    affineMatrix.at<double>(0, 0) = DataMatrix[0];
    affineMatrix.at<double>(0, 1) = DataMatrix[1];
    affineMatrix.at<double>(0, 2) = DataMatrix[2];
    affineMatrix.at<double>(1, 0) = DataMatrix[3];
    affineMatrix.at<double>(1, 1) = DataMatrix[4];
    affineMatrix.at<double>(1, 2) = DataMatrix[5];
    //std::cout<<count<<" "<< affineMatrix.at<double>(0, 2) << " " << affineMatrix.at<double>(1, 2) << std::endl;

    //fprintf(fp, "%f %f %f %f %f %f %f %f\n", txIMU * f, tyIMU * f, rollIMU, float(affineMatrix.at<double>(0, 2)), float(affineMatrix.at<double>(1, 2)), F2Finfo[0], F2Finfo[1], F2Finfo[2]);
    cv::Mat frameStabled;
    cv::warpAffine(frame, frameStabled, affineMatrix, frame.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    std::cout << frameStabled.cols << " " << frameStabled.rows << std::endl;
    return frameStabled;
}
void ImageStabilizationUpdate::deleteFile()
{
    fclose(fp);
    //std::cout << "Da vao day";
}
