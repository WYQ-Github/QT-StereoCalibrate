#include "QtDemo.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>

QtDemo::QtDemo(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_QtDemo)
    , m_isProcessing(false)
{
    ui->setupUi(this);
    m_timer = new QTimer(this);
    connect(ui->btnSelectFolder, &QPushButton::clicked, this, &QtDemo::onSelectFolder);
    connect(ui->btnSelectSaveFile, &QPushButton::clicked, this, &QtDemo::onSelectSaveFile);
    connect(ui->btnSingleCalibrate, &QPushButton::clicked, this, &QtDemo::onSingleCalibrate);
   
    connect(ui->spinCornerWidth, QOverload<int>::of(&QSpinBox::valueChanged), this, &QtDemo::updateCalibrationSettings);
    connect(ui->spinCornerHeight, QOverload<int>::of(&QSpinBox::valueChanged), this, &QtDemo::updateCalibrationSettings);
    connect(ui->spinSquareWidth, QOverload<int>::of(&QSpinBox::valueChanged), this, &QtDemo::updateCalibrationSettings);
    connect(ui->spinSquareHeight, QOverload<int>::of(&QSpinBox::valueChanged), this, &QtDemo::updateCalibrationSettings);
    updateCalibrationSettings();

    connect(ui->btnSelectLeftFolder, &QPushButton::clicked, this, &QtDemo::onSelectLeftFolder);
    connect(ui->btnSelectRightFolder, &QPushButton::clicked, this, &QtDemo::onSelectRightFolder);
    connect(ui->btnSelectLeftCalibFile, &QPushButton::clicked, this, &QtDemo::onSelectLeftCalibFile);
    connect(ui->btnSelectRightCalibFile, &QPushButton::clicked, this, &QtDemo::onSelectRightCalibFile);
    connect(ui->btnStereoCalibrate, &QPushButton::clicked, this, &QtDemo::onStereoCalibrate);
    connect(ui->btnSelectStereoSaveFolder, &QPushButton::clicked, this, &QtDemo::onSelectStereoSaveFolder);
    setupImageDisplay();
}

QtDemo::~QtDemo()
{
    delete ui; 
}

void QtDemo::updateLog(const QString& message)
{
    ui->textEdit->append(message);
}

void QtDemo::setupImageDisplay()
{
    QScrollArea* scrollArea = ui->scrollArea;

    m_imageLabel = new QLabel(scrollArea);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    m_imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    m_imageLabel->setScaledContents(true);

    scrollArea->setWidget(m_imageLabel);
    scrollArea->setWidgetResizable(true);
}

void QtDemo::updateCalibrationSettings()
{
    int cornerWidth = ui->spinCornerWidth->value();
    int cornerHeight = ui->spinCornerHeight->value();
    int squareWidth = ui->spinSquareWidth->value();
    int squareHeight = ui->spinSquareHeight->value();

    updateLog(QString("更新校准设置：角点尺寸 (%1x%2)，棋盘格子大小 (%3x%4)")
              .arg(cornerWidth).arg(cornerHeight)
              .arg(squareWidth).arg(squareHeight));
}

void QtDemo::displayProcessedImage(const cv::Mat& image)
{
    if (image.empty()) {
        updateLog("无法加载图片");
        return;
    }
    
    cv::Mat rgb_image;
    if (image.channels() == 1) {
        cv::cvtColor(image, rgb_image, cv::COLOR_GRAY2RGB);
    } else {
        rgb_image = image.clone();
    }

    QImage qimg(rgb_image.data, 
                rgb_image.cols, 
               rgb_image.rows, 
                static_cast<int>(rgb_image.step),
                QImage::Format_RGB888);
    
    QPixmap pixmap = QPixmap::fromImage(qimg);
    m_imageLabel->setPixmap(pixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_imageLabel->setAlignment(Qt::AlignCenter);
    
    QApplication::processEvents();
}


// 下面是单目标定代码模块
#pragma region 单目标定
void QtDemo::onSelectFolder()
{
    m_folderPath = QFileDialog::getExistingDirectory(this, "选择图片文件夹");
    if (!m_folderPath.isEmpty()) {
        updateLog("已选择文件夹: " + m_folderPath);
    }
}

void QtDemo::onSelectSaveFile()
{
    QString selcetPath = QFileDialog::getExistingDirectory(this, "选择保存文件夹");
    QString folderName = QDir(m_folderPath).dirName();
    m_savePath = selcetPath + "/" + folderName + ".yml";

    if (!selcetPath.isEmpty()) {
        updateLog("已选择保存文件夹: " + m_savePath);
    }
}


void QtDemo::onSingleCalibrate()
{
    if (m_folderPath.isEmpty() || m_savePath.isEmpty()) {
        QMessageBox::warning(this, "警告", "请先选择图片文件夹和保存文件");
        return;
    }

    updateLog("开始单目标定...");
    
    singleCalib = calibration();
    singleCalib.setInputPath(m_folderPath.toStdString());
    singleCalib.setOutputPath(m_savePath.toStdString());
    singleCalib.setCornerSize(ui->spinCornerWidth->value(), ui->spinCornerHeight->value());
    singleCalib.setSqureSize(ui->spinSquareWidth->value(), ui->spinSquareHeight->value());

    singleCalib.setLogCallback([this](const std::string& message) {
        updateLog(QString::fromStdString(message));
    });

    singleCalib.setImageCallback([this](const cv::Mat& image) {
        displayProcessedImage(image);
    });

    processSingleCalibration();

    singleCalib.reset(); 
    m_folderPath.clear(); 
    m_savePath.clear(); 
}

void QtDemo::processSingleCalibration()
{
    try {
        updateLog("正在提取角点...");
        singleCalib.findCorners(true);
        updateLog("开始计算误差...");
        singleCalib.standard();
        updateLog("单目标定完成");
    } catch (const std::exception& e) {
        updateLog("单目标定失败: " + QString::fromStdString(e.what()));
    } catch (...) {
        updateLog("单目标定发生未知错误");
    }
}
#pragma endregion

// 下面是双目标定代码模块
#pragma region 双目标定
void QtDemo::onSelectLeftFolder()
{
    m_leftFolderPath = QFileDialog::getExistingDirectory(this, "选择左相机图片文件夹");
    if (!m_leftFolderPath.isEmpty()) {
        updateLog("已选择左相机图片文件夹: " + m_leftFolderPath);
    }
}

void QtDemo::onSelectRightFolder()
{
    m_rightFolderPath = QFileDialog::getExistingDirectory(this, "选择右相机图片文件夹");
    if (!m_rightFolderPath.isEmpty()) {
        updateLog("已选择右相机图片文件夹: " + m_rightFolderPath);
    }
}

void QtDemo::onSelectLeftCalibFile()
{
    m_leftCalibFile = QFileDialog::getOpenFileName(this, "选择左相机标定文件", "", "YAML文件 (*.yml)");
    if (!m_leftCalibFile.isEmpty()) {
        updateLog("已选择左相机标定文件: " + m_leftCalibFile);
    }
}

void QtDemo::onSelectRightCalibFile()
{
    m_rightCalibFile = QFileDialog::getOpenFileName(this, "选择右相机标定文件", "", "YAML文件 (*.yml)");
    if (!m_rightCalibFile.isEmpty()) {
        updateLog("已选择右相机标定文件: " + m_rightCalibFile);
    }
}

void QtDemo::onSelectStereoSaveFolder()
{
    m_stereoSavePath = QFileDialog::getExistingDirectory(this, "选择双目标定保存文件夹");
    if (!m_stereoSavePath.isEmpty()) {
        updateLog("已选择双目标定保存文件夹: " + m_stereoSavePath);
    }
}
void QtDemo::onStereoCalibrate()
{
    if (m_leftFolderPath.isEmpty() || m_rightFolderPath.isEmpty() || 
        m_leftCalibFile.isEmpty() || m_rightCalibFile.isEmpty() || m_stereoSavePath.isEmpty()) {
        QMessageBox::warning(this, "警告", "请先选择所有必要的文件和文件夹");
        return;
    }
    
    // 设置回调函数
    stereoCalib.setLogCallback([this](const std::string& message) {
        updateLog(QString::fromStdString(message));
    });

    updateLog("开始双目标定...");

    stereoCalib.setImgInPath(m_leftFolderPath.toStdString(), m_rightFolderPath.toStdString());
    stereoCalib.setOutPath(m_stereoSavePath.toStdString());
    stereoCalib.setCameraParameterInput(m_leftCalibFile.toStdString(), m_rightCalibFile.toStdString());
    stereoCalib.setCornerSize(ui->spinCornerWidth->value(), ui->spinCornerHeight->value());
    stereoCalib.setSquareSize(ui->spinSquareWidth->value(), ui->spinSquareHeight->value());
    
    m_isProcessing = true;
    
    connect(m_timer, &QTimer::timeout, this, &QtDemo::processNextStereoImage);
    m_timer->start(100);

    stereoCalib.reset(); 
    m_leftFolderPath.clear(); 
    m_rightFolderPath.clear(); 
    m_leftCalibFile.clear(); 
    m_rightCalibFile.clear(); 
    m_stereoSavePath.clear(); 
}

void QtDemo::processNextStereoImage()
{
    if (!m_isProcessing) {
        m_timer->stop();
        performStereoCalibration();
        return;
    }

    
    m_isProcessing = false;
}

void QtDemo::performStereoCalibration()
{
    try {
        stereoCalib.getCorners();
        stereoCalib.calibration();
        cv::Mat finalImage = stereoCalib.showRect();
        displayProcessedImage(finalImage);
        updateLog("双目标定完成");
    } catch (const std::exception& e) {
        updateLog("双目标定失败: " + QString::fromStdString(e.what()));
    } catch (...) {
        updateLog("双目标定发生未知错误");
    }
}
#pragma endregion