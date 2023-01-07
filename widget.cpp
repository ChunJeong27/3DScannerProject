#include "widget.h"
#include "ui_widget.h"

#include <QImage>
#include <QTimer>
#include <QPixmap>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , id_image(0)
{
    ui->setupUi(this);

    std::cout << "Starting the Calibration. Press and maintain the space bar to exit the script\n" << std::endl;
    std::cout << "Push (s) to save the image you want and push (c) to see next frame without saving the image" << std::endl;

    // termination criteria !
    criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    // Create Pipelines & Call the two cameras
    int capture_width = 640;
    int capture_height = 480;
    int display_width = 640;
    int display_height = 480;
    int framerate = 30;
    int flip_method = 0;

    std::string pipeline0 = gstreamer_pipeline(0, capture_width,
                                              capture_height,
                                              display_width,
                                              display_height,
                                              framerate,
                                              flip_method);
    std::cout << "Using pipeline0: \n\t" << pipeline0 << "\n";

    camL = cv::VideoCapture(pipeline0, cv::CAP_GSTREAMER);
    if(!camL.isOpened()) {
        std::cout<<"Failed to open camera0."<<std::endl;
        return;
    }

    std::string pipeline1 = gstreamer_pipeline(1, capture_width,
                                              capture_height,
                                              display_width,
                                              display_height,
                                              framerate,
                                              flip_method);
    std::cout << "Using pipeline1: \n\t" << pipeline1 << "\n";

    camR = cv::VideoCapture(pipeline1, cv::CAP_GSTREAMER);
    if(!camR.isOpened()) {
        std::cout<<"Failed to open camera1."<<std::endl;
        return;
    }

    connect(ui->caliPushButton, SIGNAL(clicked()), this, SLOT(cameraCalibration()));
    connect(ui->savePushButton, SIGNAL(clicked()), this, SLOT(saveImage()));
    connect(ui->takeImagesPushButton, SIGNAL(clicked(bool)), this, SLOT(startCamera(bool)));
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(removeDistortion()));

    ui->progressBar->setValue(0);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(cameraUpdate()));
    timer->start(30);

}

Widget::~Widget()
{
    camL.release();
    camR.release();
    cv::destroyAllWindows();

    delete ui;
}

std::string Widget::gstreamer_pipeline (int sensor_id, int capture_width,
                                        int capture_height, int display_width,
                                        int display_height, int framerate,
                                        int flip_method) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id)
            + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width)
            + ", height=(int)" + std::to_string(capture_height)
            + ", framerate=(fraction)" + std::to_string(framerate)
            + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method)
            + " ! video/x-raw, width=(int)" + std::to_string(display_width)
            + ", height=(int)" + std::to_string(display_height)
            + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

/* OpenCV의 Mat 클래스를 QImage 클래스로 변환 */
QImage Widget::Mat2QImage(cv::Mat const& src)
{
    QImage img = QImage((uchar*)src.data, src.cols, src.rows, src.step,
                        QImage::Format_RGB888);
    QImage dest = img.rgbSwapped();
    return dest;
}

/* OpenCV의 Mat 클래스를 QImage 클래스로 변환 */
QImage Widget::GrayMat2QImage(cv::Mat const& src)
{
    QImage img = QImage((uchar*)src.data, src.cols, src.rows, src.step,
                        QImage::Format_Grayscale8);
    QImage dest = img.rgbSwapped();
    return dest;
}

/* Qt의 QImage 클래스를 OpenCV의 Mat 클래스로 변환 */
cv::Mat Widget::QImage2Mat(QImage const& src)
{
    QImage temp = src.rgbSwapped().copy();
    cv::Mat res(temp.height(), temp.width(), CV_8UC3, (uchar*)temp.bits(),
                temp.bytesPerLine());
    return res;
}

void Widget::cameraUpdate()
{
    if ( isCameraStart ) {
        if(!camL.read(frameL)) {
            std::cout << "Capture0 read error" << std::endl;
            return;
        }
        if(!camR.read(frameR)) {
            std::cout << "Capture1 read error" << std::endl;
            return;
        }

    if (isCaliStart) {
        cv::cvtColor(frameL, grayL, COLOR_BGR2GRAY);
        cv::cvtColor(frameR, grayR, COLOR_BGR2GRAY);

        // Find the chess board corners
        // Define the number of chess corners (here 9 by 6) we are looking for with the left Camera
            retL = cv::findChessboardCorners(grayL, Size(CHECKERBOARD[0], CHECKERBOARD[1]), cornersL);
        // Same with the right camera
            retR = cv::findChessboardCorners(grayR, Size(CHECKERBOARD[0], CHECKERBOARD[1]), cornersR);

        //    cv::imshow("imgR", frameR);
        //    cv::imshow("imgL", frameL);
        ui->camLeftLabel->setPixmap(QPixmap::fromImage(Mat2QImage(frameL))
                                    .scaled(ui->camLeftLabel->size()));
        ui->camRightLabel->setPixmap(QPixmap::fromImage(Mat2QImage(frameR))
                                     .scaled(ui->camRightLabel->size()));

        // If found, add object points, image points (after refining them)
        if (retR == true && retL == true) {
            cv::cornerSubPix(grayL, cornersL, Size(11,11), Size(-1,-1), criteria);  // Refining the Position
            cv::cornerSubPix(grayR, cornersR, Size(11,11), Size(-1,-1), criteria);

            // Draw and display the corners
            cv::drawChessboardCorners(grayL, Size(CHECKERBOARD[0], CHECKERBOARD[1]), cornersL, retL);
            cv::drawChessboardCorners(grayR, Size(CHECKERBOARD[0], CHECKERBOARD[1]), cornersR, retR);
//                    cv::imshow("VideoL",grayL);
//                    cv::imshow("VideoR",grayR);
            ui->test1->setPixmap(QPixmap::fromImage(GrayMat2QImage(grayL))
                                 .scaled(ui->test1->size()));
            ui->test2->setPixmap(QPixmap::fromImage(GrayMat2QImage(grayR))
                                 .scaled(ui->test2->size()));

            isCameraStart = false;

//            if ((cv::waitKey(0) & 0xFF) == 's') {   // Push "s" to save the images and "c" if you don't want to
//                string str_id_image = to_string(id_image);
//                cout << "Images " + str_id_image + " saved for right and left cameras" << endl;
//                cv::imwrite("chessboard-R"+str_id_image+".png",frameR); // Save the image in the file where this Programm is located
//                cv::imwrite("chessboard-L"+str_id_image+".png",frameL);
//                id_image++;
//            } else {
//                qDebug("Images not saved");
//            }

        }
    } else {
        ui->camLeftLabel->setPixmap(QPixmap::fromImage(Mat2QImage(frameL))
                                    .scaled(ui->camLeftLabel->size()));
        ui->camRightLabel->setPixmap(QPixmap::fromImage(Mat2QImage(frameR))
                                     .scaled(ui->camRightLabel->size()));
    }
    }
}

void Widget::cameraCalibration()
{
    // creating vector to store vectors of 3D points
    vector<vector<Point3f> > objpoints;

    // creating vector to store vectors of 2D points
    vector<vector<Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    vector<Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(Point3f(j,i,0));
    }

    vector<String> images;
    string path = "./*.png";

    glob(path, images);

    Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners
    vector<Point2f> corner_pts;
    bool success;

    int cnt = 0;
    ui->progressBar->setMaximum(images.size());

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
        frame = imread(images[i]);
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        success = findChessboardCorners(gray, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        /*
        * If desired number of corner are detected,
        * we refine the pixel coordinates and display
        * them on the images of checker board
        */
        if(success)
        {
            cnt++;
            ui->progressBar->setValue(cnt);

            TermCriteria criteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cornerSubPix(gray,corner_pts,Size(11,11), Size(-1,-1), criteria);

            // Displaying the detected corner points on the checker board
            drawChessboardCorners(frame, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

//        imshow("Image",frame);
        ui->test1->setPixmap(QPixmap::fromImage(Mat2QImage(frame))
                                    .scaled(ui->test1->size()));
//        waitKey(0);
    }

    destroyAllWindows();

    /*
    * Performing camera calibration by
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the
    * detected corners (imgpoints)
    */
    double rms = calibrateCamera(objpoints, imgpoints, Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);
qDebug("hi");
    cout << "cameraMatrix : " << cameraMatrix << endl;
    cout << "distCoeffs : " << distCoeffs << endl;
    cout << "Rotation vector : " << R << endl;
    cout << "Translation vector : " << T << endl;
    cout << "RMS : " << rms << endl;


    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(0, 0))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(0, 1))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(0, 2))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(1, 0))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(1, 1))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(1, 2))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(2, 0))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(2, 1))+", ");
    ui->matrixPlainTextEdit->appendPlainText(QString::number(cameraMatrix.at<double>(2, 2)));

    ui->coeffisPlainTextEdit->appendPlainText(QString::number(distCoeffs.at<double>(0)));
    ui->coeffisPlainTextEdit->appendPlainText(QString::number(distCoeffs.at<double>(1)));
    ui->coeffisPlainTextEdit->appendPlainText(QString::number(distCoeffs.at<double>(2)));
    ui->coeffisPlainTextEdit->appendPlainText(QString::number(distCoeffs.at<double>(3)));
    ui->coeffisPlainTextEdit->appendPlainText(QString::number(distCoeffs.at<double>(4)));

    ui->rmsLineEdit->setText(QString::number(rms));

}

void Widget::startCamera(bool isStart)
{
    isCaliStart = isStart;
}

void Widget::saveImage()
{
    std::string str_id_image = to_string(id_image);
    cout << "Images " + str_id_image + " saved for right and left cameras" << endl;
    ui->camLeftLabel->pixmap()->save(std::string("chessboard-L"+str_id_image+".png").c_str(), "PNG");
    ui->camLeftLabel->pixmap()->save(std::string("chessboard-R"+str_id_image+".png").c_str(), "PNG");
    id_image++;

    isCameraStart = true;
}

void Widget::removeDistortion()
{
    Mat frame1 = imread("./chessboard-L3.png");
    Mat frame2;

    Mat map1, map2;
    Size imageSize=Size(320,240);

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);

    remap(frame1, frame2, map1, map2, INTER_LINEAR);

//    imshow("Original Image", frame1);
//    imshow("Rectified Image", frame2);
    ui->test1->setPixmap(QPixmap::fromImage(Mat2QImage(frame1))
                                .scaled(ui->test1->size()));
    ui->test2->setPixmap(QPixmap::fromImage(Mat2QImage(frame2))
                                .scaled(ui->test2->size()));

}
