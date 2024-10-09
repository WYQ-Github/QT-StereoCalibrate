#pragma once
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// 双目标定
// 参考：https://blog.csdn.net/u010801994/article/details/84563208

class doubleCalibration {
private:

    std::function<void(const std::string&)> m_logCallback;

    // 左右摄像机图片路径
    std::string img_path_L, img_path_R;

    // 保存输出路径
    std::string outPath;

    // 左右相机内参
    /*摄像机内参反映的是摄像机坐标系到坐标系之间的投影关系
    /*包括fx,fy,cx,y,以及畸变系数[k1,k2,p1,p2,k3]
   /*事先标定好的左相机的内参矩阵
    fx 0 cx
    0 fy cy
    0  0  1
    */
    cv::Mat cameraMatrix_L;
    cv::Mat cameraMatrix_R;
    cv::Mat discoeff_L; /*畸变系数[k1,k2,p1,p2,k3]*/
    cv::Mat discoeff_R;

    // 标定版内角点尺寸
    cv::Size board_Corner_size;

    // 标定版棋盘格子尺寸
    cv::Size square_size;

    // 图片尺寸
    cv::Size img_size;

    // 标定板的实际物理坐标
    std::vector<std::vector<cv::Point3f>> obejct_points;

    // 被读取的图片数量
    int img_cout;

    // 左边、右边摄像机所有照片角点的坐标集合
    std::vector<std::vector<cv::Point2f>> corners_L;
    std::vector<std::vector<cv::Point2f>> corners_R;


    // 图像
    cv::Mat imgL;
    cv::Mat imgR;
    cv::Mat grayL;
    cv::Mat grayR;

    cv::Mat rectifyImageL2, rectifyImageR2;
    cv::Mat rectifyImageL, rectifyImageR;

    /*s摄像机外参反映的是摄像机坐标系和世界坐标系之间的旋转R和平移T关系*/
    /* 外参一旦标定好，下两个相机的结构就要保持固定，否杂外参就会发生变化，需要重新进行外参标定*/
    cv::Mat R, T, E, F;/*R旋转矢量 T平移矢量 E本征矩阵  F基础矩阵*/


    // 对极线校正（立体校正）用
    cv::Mat Rl, Rr, Pl, Pr, Q;/*矫正旋转矩阵R，投影矩阵P，重投影矩阵Q*/
    cv::Mat mapLx, mapLy, mapRx, mapRy; // 映射表
    cv::Rect validROIL, validROIR;/*图像矫正之后，会对图像进行剪裁，其中,validROI建材之后的区域*/

private:

    void outputCameraParam(void)
    {
        /*保存数据*/
        /*输出数据*/
        cv::Mat rectifyImageL2, rectifyImageR2;
        std::string outPath1 = outPath +"\\"+ "intrisics.yml";
        cv::FileStorage fs(outPath1, cv::FileStorage::WRITE);
        if (fs.isOpened())
        {
            fs << "cameraMatrixL" << cameraMatrix_L
                << "cameradistCoeffsL" << discoeff_L
                << "cameraMatrixR" << cameraMatrix_R
                << "cameradistCoeffsR" << discoeff_R;
            fs.release();
        }
        else
        {
            log("Erro:can not save the intrinsics!!!!");
        }

        std::string outPath2 = outPath + "\\"+ "extrinsics.yml";
        cv::FileStorage fs2(outPath2, cv::FileStorage::WRITE);

        if (fs2.isOpened())
        {
            fs2 << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr
                << "Pl" << Pl << "Pr" << Pr << "Q" << Q;

                //<< "mapLx" << mapLx /*这些应该就够了*/
                //<< "mapLy" << mapLy
                //<< "mapRx" << mapRx
                //<< "mapRy" << mapRy;
            fs2.release();
        }
        else
        {
            std::cout << "Erro:can not save the extrinsics parameters!!!!" << std::endl;
        }
    }

public:
    doubleCalibration() :img_cout(0) {}
    // 设置左右摄像机图片路径
    void setImgInPath(std::string L, std::string R)
    {
        img_path_L = L;
        img_path_R = R;
    }

    // 设置输出路径
    void setOutPath(std::string path)
    {
        outPath = path;
    }

    // 设置左右相机参数文件路径
    void setCameraParameterInput(std::string L, std::string R)
    {
        cv::FileStorage file_L(L, cv::FileStorage::READ);
        cv::FileStorage file_R(R, cv::FileStorage::READ);

        file_L["cameraMatrix"] >> cameraMatrix_L;
        file_L["disCoeffs"] >> discoeff_L;
        file_R["cameraMatrix"] >> cameraMatrix_R;
        file_R["disCoeffs"] >> discoeff_R;
    }

    // 设置标定板内角点尺寸
    void setCornerSize(int x, int y)
    {
        board_Corner_size.width = x;
        board_Corner_size.height = y;
    }

    // 设置标定版棋盘格子尺寸
    void setSquareSize(int x, int y)
    {
        square_size.width = x;
        square_size.height = y;
    }

    // 设置标定板的实际物理坐标
    void set3DCoordinates()
    {
        std::vector<cv::Point3f> tempPoints; // 与角点走向相匹配
        for (int rowIndex = 0; rowIndex < board_Corner_size.height; rowIndex++)
        {
            for (int colIndex = 0; colIndex < board_Corner_size.width; colIndex++)
            {
                cv::Point3f realPoint;
                realPoint.x = colIndex * square_size.width;
                realPoint.y = rowIndex * square_size.height;
                realPoint.z = 0;
                tempPoints.push_back(realPoint);
            }
        }

        for (int imgIndex = 0; imgIndex < img_cout; imgIndex++)
        {
            obejct_points.push_back(tempPoints);
        }
    }

    // 提取角点
    void getCorners()
    {

        std::vector<cv::Point2f> temp_points_L;
        std::vector<cv::Point2f> temp_points_R;

        std::string fileNameL;
        std::string fileNameR;

        log("**********正在提取角点......");

        std::vector<std::string> fnL, fnR;
        std::vector<cv::Mat> imagesL, imagesR;
        cv::glob(img_path_L, fnL);
        cv::glob(img_path_R, fnR);

        if (fnL.size() == fnR.size())
        {
            for (int i = 0; i < fnL.size(); i++)
            {
                imagesL.push_back(cv::imread(fnL[i]));
                imagesR.push_back(cv::imread(fnR[i]));
            }
        }
        for (int i = 0; i < imagesL.size(); i++)
        {
            img_cout++;
            imgL = imagesL[i];
            imgR = imagesR[i];

            cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
            cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);


            if (cv::findChessboardCorners(grayL, board_Corner_size, temp_points_L)
                && cv::findChessboardCorners(grayR, board_Corner_size, temp_points_R))
            {

                cv::cornerSubPix(grayL, temp_points_L,
                    cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 0.01));
                cv::drawChessboardCorners(imgL, board_Corner_size,
                    temp_points_L, true);

                cv::cornerSubPix(grayR, temp_points_R,
                    cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 0.01));
                cv::drawChessboardCorners(imgR, board_Corner_size,
                    temp_points_R, true);

                //imshow("chessborad", imgL);
                //imshow("chessborad", imgL);
                corners_L.push_back(temp_points_L);
                corners_R.push_back(temp_points_R);

                log("******" + fnL[i] + "成功提取到内角点");
                log("******" + fnR[i] + "成功提取到内角点");
                if ((char)cv::waitKey(50) == 'q')
                {
                    break;
                }
            }

            if (img_cout == 1)
            {
                img_size.width = imgL.cols;
                img_size.height = imgL.rows;
            }

        }

        log("总共有 " + std::to_string(img_cout) + " x2 张图片提取到内角点");
    }

    // 标定摄像头
    void calibration()
    {

        log("img_cout:" + std::to_string(img_cout));
        set3DCoordinates();

        // 双目标定
        cv::Mat perViewErrors;
        double rms = cv::stereoCalibrate(obejct_points, corners_L, corners_R,
            cameraMatrix_L, discoeff_L,
            cameraMatrix_R, discoeff_R,
            img_size,
            R,  /*右相机坐标系相对于左相机坐标系的旋转矩阵*/
            T, /* 右相机坐标系相对于左相机坐标系的平移向量*/
            E, /* 本征矩阵*/
            F, /* 基本矩阵*/
            perViewErrors,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                100, 1e-6));
        // CALIB_USE_INTRINSIC_GUESS 根据指定的标志优化部分或全部内在参数。初始值由用户提供。
        //CALIB_FIX_INTRINSIC：是否固定内参；只有R, T, E, and F 矩阵被估计。
        log("Stereo Calibration done with RMS error = " + std::to_string(rms));
        //cv::waitKey();

        // 要通过两幅图像估计物体的深度信息，就必须在两幅图像中准确地匹配到同一物体
        // 这样才能根据该物点在两幅图像中的位置关系，计算物体深度
        // 为了降低匹配的计算量，两个摄像头的成像平面应处于统一平面
        // 立体校正为每个摄像头计算立体校正所需要的映射矩阵，达到上述目的
        cv::stereoRectify(cameraMatrix_L, discoeff_L, cameraMatrix_R, discoeff_R,
            img_size, R, T, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, -1,
            img_size, &validROIL, &validROIR);

        // 对极线校正（立体校正），（得到校正映射）
        cv::initUndistortRectifyMap(cameraMatrix_L, discoeff_L,
            Rl, Pl, img_size, CV_32FC1,
            mapLx, mapLy);
        cv::initUndistortRectifyMap(cameraMatrix_R, discoeff_R,
            Rr, Pr, img_size, CV_32FC1,
            mapRx, mapRy);


        cv::cvtColor(grayL, rectifyImageL, cv::COLOR_GRAY2BGR);
        cv::cvtColor(grayR, rectifyImageR, cv::COLOR_GRAY2BGR);


        cv::remap(rectifyImageL, rectifyImageL2, mapLx, mapLy, cv::INTER_LINEAR);
        cv::remap(rectifyImageR, rectifyImageR2, mapLx, mapLy, cv::INTER_LINEAR);

        outputCameraParam();
    }

    // 显示校正结果
    cv::Mat showRect()
    {
        cv::Mat canvas;
        double sf;
        int w, h;
        sf = 600. / std::max(img_size.width, img_size.height); /*宽1200,高600*/
        w = cvRound(img_size.width * sf);
        h = cvRound(img_size.height * sf);
        canvas.create(h, w * 2, CV_8UC3);

        // 左图像画到画布上
        cv::Mat canvasPart = canvas(cv::Rect(0, 0, w, h)); // 浅复制
        cv::resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
        cv::Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
            cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
        rectangle(canvasPart, vroiL, cv::Scalar(0, 0, 255), 3, 8);
        std::cout << "Painted ImageL" << std::endl;

        // 右图像画到画布上
        canvasPart = canvas(cv::Rect(w, 0, w, h));
        resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA); /*基于区域像素关系的一种重采样或者插值方式.该方法是图像抽取的首选方法,
                                                                                它可以产生更少的波纹,但是当图像放大时,它的效果与INTER_NEAREST效果相似.*/
        cv::Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf), cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
        cv::rectangle(canvasPart, vroiR, cv::Scalar(0, 0, 255), 3, 8);
        std::cout << "Painted ImageR" << std::endl;

        // 画上对应的线条
        for (int i = 0; i < canvas.rows; i += 16)
        {
            cv::line(canvas, cv::Point(0, i), cv::Point(canvas.cols, i), cv::Scalar(0, 255, 0), 1, 8);
        }
        return canvas;
        // cv::imshow("rectified", canvas);
        // cv::waitKey(0);
    }



    // 添加一个设置回调函数的方法
    void setLogCallback(std::function<void(const std::string&)> callback) {
        m_logCallback = callback;
    }

    // 替换所有的 std::cout 为这个新的日志函数
    void log(const std::string& message) {
        if (m_logCallback) {
            m_logCallback(message);
        }
    }

    void reset() {
        img_cout = 0;
        corners_L.clear();
        corners_R.clear();
        obejct_points.clear();
    }
};
