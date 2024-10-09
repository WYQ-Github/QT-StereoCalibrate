#pragma once
#include "ui_QtDemo.h"
#include <QMainWindow>
#include <QString>
#include <QTextStream>
#include <QDateTime>
#include <QPixmap>
#include <QPainter>
#include <QLabel>
#include <QStringList>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include "stereo_calibration.h"
#include "single_calibration.h"
class QtDemo : public QMainWindow {
    Q_OBJECT
    
public:
    QtDemo(QWidget* parent = nullptr);
    ~QtDemo();

private slots:
    // 单目标定 按钮
    void onSelectFolder();
    void onSelectSaveFile();
    void onSingleCalibrate();
    // 双目标定 按钮
    void onSelectLeftFolder();
    void onSelectRightFolder();
    void onSelectLeftCalibFile();
    void onSelectRightCalibFile();
    void onStereoCalibrate();
    void onSelectStereoSaveFolder();

private:
    Ui_QtDemo* ui;
    QString m_folderPath;
    QString m_savePath;
    QLabel* m_imageLabel;
    QTimer* m_timer;

    void processNextStereoImage();
    void performStereoCalibration();
    bool m_isProcessing;
    void updateLog(const QString& message);
    void setupImageDisplay();
    void updateCalibrationSettings();
    void displayProcessedImage(const cv::Mat& image);


    QString m_leftFolderPath;
    QString m_rightFolderPath;  
    QString m_leftCalibFile;
    QString m_rightCalibFile;
    QString m_stereoSavePath;
    std::vector<cv::Mat> leftImages;    
    std::vector<cv::Mat> rightImages;
    doubleCalibration stereoCalib; 

    calibration singleCalib;
    void processSingleCalibration();                    
};