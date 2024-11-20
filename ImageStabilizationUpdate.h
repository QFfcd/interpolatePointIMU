#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "opencv2/core.hpp"
#include <string>
class ImageStabilizationUpdate
{
private:
    //cv::Mat Image;
    std::vector <double> RollArray;
    std::vector <double> PitchArray;
    std::vector <double> YawArray;
    double F2Finfo[3];            // roll, pitch, yaw
    double RollPrevious;
    double PitchPrevious;
    double YawPrevious;
    double hFov;
    double rollIMU;
    double txIMU;
    double tyIMU;
    double lastUnwrap;
    double deltaYaw;
    double deltaPitch;
    double deltaRoll;
    // for debugging
    FILE* fp = NULL;
    float matrixLog[6];
    const int F_IMU = 100;
    const int F_CAM = 60;


public:
    int count;
    cv::Mat transformMat;

    double ScaleImage;
    double Target_x;
    double Target_y;
    double yaw100HzInterpolate;
    double roll100HzInterpolate;
    double pitch100HzInterpolate;
    double unwrap(double preIMU, double currIMU, double thres, double tolerance);
    double wrapToPi(double currIMU);
    ImageStabilizationUpdate();
    ~ImageStabilizationUpdate();
    /* Dung method nay khi khong muon dung interpolate*/
    void processIMUdata(double roll, double pitch, double yaw, double hfov, int sizeArrayBuffer);
    /* Dung method nay khi muon dung interpolate */
    std::vector<double> readDataIMUInterpolate(std::ifstream& fileIMU_60Hz, std::ifstream& fileIMU_100Hz);
    std::vector<double> readDataIMUNonInterpolate(std::ifstream& fileIMU_60Hz);
    //void readDataMatrixLog(float const* DataMatrix);
    cv::Mat stabilizing(cv::Mat Image);
    void imwarpPoint(double Target_x_FrameBefore, double Target_y_FrameBefore);
    cv::Mat stabilizingUsingmatrixLog(cv::Mat Image, double const* matrixData);
    void deleteFile();
    void readFeatureExtract(cv::Mat prevFrame, cv::Mat currFrame);

};
