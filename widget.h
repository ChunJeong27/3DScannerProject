#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

using namespace std;
using namespace cv;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private:
    Ui::Widget *ui;

private:
    int id_image;
    TermCriteria criteria;
    VideoCapture camL, camR;
    QTimer *timer;
    bool retL = false, retR = false;
    cv::Mat frameL, frameR, grayL, grayR, cornersL, cornersR;

    const int CHECKERBOARD[2] = {8,6};
    cv::Mat cameraMatrixL, cameraMatrixR, distCoeffsL, distCoeffsR, RL, RR, TL,TR;

    bool isCaliStart = false;
    bool isCameraStart = true;

    string gstreamer_pipeline (int sensor_id, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method);
    QImage Mat2QImage(cv::Mat const& src);
    cv::Mat QImage2Mat(QImage const& src);
    QImage GrayMat2QImage(cv::Mat const& src);
signals:
    void detectCornor();

public slots:
    void cameraUpdate();
    void cameraCalibration();
    void startCamera(bool);
    void saveImage();
    void removeDistortion();
};
#endif // WIDGET_H
