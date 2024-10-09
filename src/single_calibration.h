#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <functional>


class calibration {
private:
    std::string fileInPath;
    std::vector<cv::Mat> images;
    std::string fileOutPath;
    int image_count;
    cv::Size image_size;  
    cv::Size board_corner_size;                              //标定板每行、列的角点数 
    std::vector<cv::Point2f> image_points_buf;               // 缓存每幅图像上检测到的角点
    std::vector<std::vector<cv::Point2f>> image_points_seq;  // 保存检测到的所有角点 
    int corner_count;                                        // 角点个数  
    cv::Size square_size;                                    // 实际测量得到的标定板上每个棋盘格的大小
    std::vector<std::vector<cv::Point3f>> obejct_points;     // 保存标定板上角点的三维坐标
    cv::Mat camraMatrix;                                     // 摄像机内参数矩阵 
    cv::Mat disCoeffs;                                       // 摄像机的5个畸变系数；k1,k2,p1,p2,k3   
    std::vector<cv::Mat> vecsMat_T;                          //每幅图像的旋转向量
    std::vector<cv::Mat> vecMat_R;                           //每幅图像的平移向量

    std::function<void(const cv::Mat&)> m_imageCallback;

    std::function<void(const std::string&)> m_logCallback;
public:

    void setImageCallback(std::function<void(const cv::Mat&)> callback) {
        m_imageCallback = callback;
    }

    calibration()
    {
        // 初始化
        image_count = 0;
        corner_count = -1;
        camraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
        disCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
    }


   
    void setInputPath(std::string filePath)
    {
        // 设置输入
        std::vector<cv::String> fn;
        cv::glob(filePath, fn, false);

        int num = fn.size();
        for (int i = 0; i < num; i++)
        {
            images.push_back(cv::imread(fn[i]));
        }
    }

    // 设置角点尺寸
    void setCornerSize(int x, int y)
    {
        board_corner_size.width = x;
        board_corner_size.height = y;
    }

    // 设置棋盘格子大小
    void setSqureSize(int x, int y)
    {
        square_size = cv::Size(x, y);
    }

    // 设置相机参数输出路径
    void setOutputPath(std::string filepath)
    {
        fileOutPath = filepath;
    }

    // 提取角点信息，并进一步提取亚像素角点信息
    void findCorners(bool draw = false)
    {
        log("开始提取角点.......");

        for (int i = 0; i < images.size(); i++)
        {
            // 读取图片
            cv::Mat imageInput = images[i];

            image_count++;

            // 获取图片尺寸
            if (image_count == 1)
            {
                image_size.width = imageInput.cols;
                image_size.height = imageInput.rows;
            }

            // 转灰度图像
            cv::Mat gray_image;
            cvtColor(imageInput, gray_image, cv::COLOR_BGR2GRAY);

            // 提取角点
            if (0 == cv::findChessboardCorners(gray_image, board_corner_size, image_points_buf))
            {
                log(std::string("***") + " cannot find chessboardcorners");
            }
            else
            {
                cv::cornerSubPix(gray_image, image_points_buf,cv::Size(5, 5), cv::Size(-1, -1), 
                                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                // 绘制内角点
                if (draw)
                {
                    cv::drawChessboardCorners(gray_image, board_corner_size, image_points_buf, false);
                    
                    if (m_imageCallback) {
                        m_imageCallback(gray_image);
                    }
                    //cv::imshow("img", gray_image);
                    //cv::waitKey(500);
                }
                // 保存角点
                image_points_seq.push_back(image_points_buf);
            }
        }


        //查看每张图片所检测到的内角点数量
        int num = image_points_seq.size();
        for (int i = 0; i < num; i++)
        {
            log("第 " + std::to_string(i) + "张图片的内角点数量：" + std::to_string(image_points_seq[i].size()));
        }
        log("**********角点提取完成************");
    }



    // 相机标定
    void standard()
    {
        // 三维坐标
        int i, j, t;
        for (t = 0; t < image_count; t++)
        {
            std::vector<cv::Point3f> tempPointSet;
            for (i = 0; i < board_corner_size.height; i++)
            {
                for (j = 0; j < board_corner_size.width; j++)
                {
                    cv::Point3f realPoint;

                    realPoint.x = j * square_size.width;
                    realPoint.y = i * square_size.height;
                    realPoint.z = 0;
                    tempPointSet.push_back(realPoint);
                }
            }
            obejct_points.push_back(tempPointSet);
        }

        // 三维坐标与图像的二维点进行标定
        /* 三维点、角点--->相机内参数、相机外参数*/
        log("**********开始标定****************");
        cv::calibrateCamera(obejct_points, image_points_seq, image_size, camraMatrix, disCoeffs, vecMat_R, vecsMat_T);

        log("**********标定完成****************");

        // 对标定结果进行评价
        log("**********开始评价标定****************");
        double total_err = 0.0; // 所有图像的平均误差的总和
        double err = 0.0;      // 每幅图像的平均误差
        std::vector<cv::Point2f> image_points_new; // 保存重新计算得到的投影点

        /*三维坐标、相机内参数---->二维图像坐标*/
        for (i = 0; i < image_count; i++)
        {
            std::vector<cv::Point3f> temp_obejct = obejct_points[i];
            cv::projectPoints(temp_obejct, vecMat_R[i], vecsMat_T[i], camraMatrix, disCoeffs, image_points_new);

            /*计算刚刚的投影得到的坐标与角点坐标的误差值*/
            std::vector<cv::Point2f> temp_corners = image_points_seq[i];
            cv::Mat temp_corners_mat = cv::Mat(1, temp_corners.size(), CV_32FC2);
            cv::Mat image_points_new_mat = cv::Mat(1, image_points_new.size(), CV_32FC2);

            for (int j = 0; j < temp_corners.size(); j++)
            {
                image_points_new_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_new[j].x, image_points_new[j].y);
                temp_corners_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(temp_corners[j].x, temp_corners[j].y);
            }

            err = cv::norm(image_points_new_mat, temp_corners_mat, cv::NORM_L2);
            total_err += err /= (board_corner_size.width * board_corner_size.height);
            log("第" + std::to_string(i + 1) + "副图像的平均误差：" + std::to_string(err) + "像素");

        }

        log("总体误差：" + std::to_string(total_err / image_count) + "像素");
        log("********评价完成*******");

        // 保存相机参数
        cv::FileStorage fs2(fileOutPath, cv::FileStorage::WRITE);
        fs2 << "cameraMatrix" << camraMatrix;
        fs2 << "disCoeffs" << disCoeffs;
        fs2.release();
        log("*******已保存相机参数*****");

    }

    // 利用标记结果矫正
    void correction(cv::Mat& image, cv::Mat& out)
    {
        cv::Mat mapx = cv::Mat(image.size(), CV_32FC1);
        cv:: Mat mapy = cv::Mat(image.size(), CV_32FC1);
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        cv::initUndistortRectifyMap(camraMatrix, disCoeffs, R, camraMatrix, image.size(), CV_32FC1, mapx, mapy);
        cv::remap(image, out, mapx, mapy, cv::INTER_LINEAR);
        //cv::imshow("correction", out);
        //cv::imshow("img", image);
        //cv::waitKey(0);
    }

    void setLogCallback(std::function<void(const std::string&)> callback) {
        m_logCallback = callback;
    }

    void log(const std::string& message) {
        if (m_logCallback) {
            m_logCallback(message);
        }
    }

    void reset() {
        images.clear();
        image_points_seq.clear();
        obejct_points.clear();
        image_count = 0;
        corner_count = -1;
        camraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
        disCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
        vecsMat_T.clear();
        vecMat_R.clear();
    }
};