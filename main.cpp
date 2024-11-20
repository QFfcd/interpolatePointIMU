#include <iostream>
#include "ImageStabilizationUpdate.h"
#include <fstream>
#include <vector>

using namespace std;
using namespace cv;



// for debug pur
void showMatrix(cv::Mat transformMat)
{
    cout << "Matrix:\n" << transformMat << endl;
};

int main()
{
    /* ===================== Phan khai bao bien =================================================*/
    ImageStabilizationUpdate ImgS1Update;
    string VideonameIn = "/home/ngoclth/Documents/stabImg/stabUAV/videoDay60Hz.avi";
    string VideonameOutIMU = "/home/ngoclth/Documents/stabImg/stabUAV/videoDay60Hz_out.avi";
    cv::VideoCapture cap(VideonameIn);

    /* Phan code trong ham main duoc dung nhu la code dau vao cua class
    Frame read in have size 1920x1088 */
    /* for CAM DAY*/
    const int W = 1920;
    const int H = 1080;
    cv::VideoWriter videoWriter(VideonameOutIMU, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 27, cv::Size(W, H));
    /*For cam IR*/
    //const int W = 640;
    //const int H = 512;
    //cv::VideoWriter videoWriter(VideonameOutIMU, cv::VideoWriter::fourcc('M','J','P','G'), 50, cv::Size(W, H));
    cv::Mat frameCurr;
    cv::Mat framePrev;
    cv::Mat frameStabled;

    string dataIMU_60Hz = "/home/ngoclth/Documents/stabImg/stabUAV/imu60Hz.txt";
    string dataIMU_100Hz = "/home/ngoclth/Documents/stabImg/stabUAV/imu100Hz.txt";
    string targetLocaton = "/home/ngoclth/Documents/stabImg/stabUAV/targetOri.txt";
    // defind mode on dinh anh
    const int NONSTAB = 0;
    const int FEATURE_EXTRACTION = 1;   // only feature extraction
    const int IMUNORMAL = 2;            // only IMU 60Hz
    const int FUSION = 3;  // IMU interpolation and Feature extraction
    int stabMode = FUSION;



    /*========================================== Phan trien khai ============================================================*/
    // Kiem tra xem co mo duoc file khong
    std::ifstream inputFileIMU60Hz(dataIMU_60Hz);
    if (!inputFileIMU60Hz.is_open()) {
        std::cerr << "Error: Unable to open the file IMU." << std::endl;
        return -1;
    }
    std::ifstream inputFileIMU100Hz(dataIMU_100Hz);
    if (!inputFileIMU100Hz.is_open()) {
        std::cerr << "Error: Unable to open the file IMU." << std::endl;
        return -1;
    }
    std::ifstream inputFileTargetLocation(targetLocaton);
    if (!inputFileTargetLocation.is_open()) {
        std::cerr << "Error: Unable to open the file Target location." << std::endl;
        return -1;
    }
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the video file." << std::endl;
        return -1;
    }
    if (!videoWriter.isOpened()) {
        std::cerr << "Error opening video writer." << std::endl;
        return -1;
    }
    // double RollIMU, PitchIMU, YawIMU;
    double xLocationPre, yLocationPre,frameID,widthTarget,heightTarget;
    vector<double> angularIMU;
    cout << "Ban da su dung on dinh anh Feature Extraction";

    // double HFOV;
    /* Xu ly anh */
    while (cap.isOpened())
    {
        cap.read(frameCurr);
        switch (stabMode)
        {
        case FEATURE_EXTRACTION:
            cout << "Ban da su dung on dinh anh Feature Extraction";
            break;
        case IMUNORMAL:
            cout << "Ban da su dung on dinh anh IMUNormal"<<endl;
            angularIMU = ImgS1Update.readDataIMUNonInterpolate(inputFileIMU60Hz);
            // Sau ham nay angularIMU chua thong tin lan luot tu 0->2  roll, pitch, yaw
            ImgS1Update.processIMUdata(angularIMU[0], angularIMU[1], angularIMU[2], angularIMU[3], 1800);
            if (ImgS1Update.count > 1)
            {
                ImgS1Update.readFeatureExtract(framePrev, frameCurr);
            }
            break;
        case FUSION:
            cout << "Ban da su dung on dinh anh FUSION" << endl;
            angularIMU = ImgS1Update.readDataIMUInterpolate(inputFileIMU60Hz, inputFileIMU100Hz);
            cout <<"Gia tri goc hFOV"<< angularIMU[0] << " " << angularIMU[1] << " " << angularIMU[2] << " " << angularIMU[3] << endl;
            // Sau ham nay angularIMU chua thong tin lan luot tu 0->2  roll, pitch, yaw
            ImgS1Update.processIMUdata(angularIMU[0], angularIMU[1], angularIMU[2], angularIMU[3], 1800);

            if (ImgS1Update.count > 1)
            {
                ImgS1Update.readFeatureExtract(framePrev, frameCurr);
            }
            break;
        default:
            cout << "Khong su dung on dinh anh" << endl;
        }
        inputFileTargetLocation >> frameID>> xLocationPre >> yLocationPre>> widthTarget>> heightTarget;
        ImgS1Update.imwarpPoint(xLocationPre, yLocationPre);
        framePrev = frameCurr;
        rectangle(frameCurr, Point(ImgS1Update.Target_x - 60, ImgS1Update.Target_y - 60), Point(ImgS1Update.Target_x + 60, ImgS1Update.Target_y + 60), Scalar(0, 0, 255), 2);// blue color, thickness 2
        //rectangle(frame, Point(xLocationPre-6, yLocationPre-6), Point(xLocationPre+6, yLocationPre+6), Scalar(0, 0, 255), 2);// blue color, thickness 2
        frameStabled = ImgS1Update.stabilizing(frameCurr);
        cv::imshow("Video Frame ", frameStabled);
        cv::waitKey(100);
        videoWriter.write(frameStabled);
        showMatrix(ImgS1Update.transformMat);
    }
    ImgS1Update.deleteFile();
    cap.release();
    videoWriter.release();
    waitKey(0);
    return 0;
}
