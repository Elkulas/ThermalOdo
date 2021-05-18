/**
 * 
 * 图像特征的提取操作函数的集合
 * 
 */

#include "feature.h"

using namespace std;

/**
 * @brief 计算最大最小值
 * @param[in]   image   输入图像
 * @param[out]  maxmin  最大最小值
 */
Eigen::Vector2d GetMaxAndMin(cv::Mat& image)
{  
     
    // 8 bit 
    if(image.type() == 0)
    {
        Eigen::Vector2d maxmin;
        double vmax = 0;
        double vmin = 255;
        for(int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            double value = (double)(image.at<uchar>(i,j));
            if(value > vmax) vmax = value;
            if(value < vmin) vmin = value;
        }

        maxmin << vmax, vmin;
        return maxmin;
    }
    // 16 bit
    else if(image.type() == 2)
    {
        Eigen::Vector2d maxmin;
        double vmax = 0;
        double vmin = 65536;
        for(int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            double value = (double)(image.at<ushort>(i,j));
            if(value > vmax) vmax = value;
            if(value < vmin) vmin = value;
        }

        maxmin << vmax, vmin;
        return maxmin;
    }
}


/**
 * @brief 将16位图片压缩为8位图片
 * @param[in]   image_16    16位图像
 * @param[out]  image_8     8位图像
 */
cv::Mat Swap16To8(cv::Mat& image_16)
{
    cv::Mat image_8(image_16.size(), CV_8UC1);

    double vmax = 0;
    double vmin = 65536;
    // find max and min
    for(int i = 0; i < image_16.rows; i++)
    for(int j = 0; j < image_16.cols; j++)
    {
        double value = (double)(image_16.at<ushort>(i,j));
        if(value > vmax) vmax = value;
        if(value < vmin) vmin = value;
    }
    // cout << " HIHIHIHI "<< vmax << " " << vmin <<endl;
    double span = vmax - vmin;

    for(int i = 0; i < image_16.rows; i++)
    for(int j = 0; j < image_16.cols; j++)
    {
        double value = (double)(image_16.at<ushort>(i,j));
        image_8.at<uchar>(i,j) = ((value - vmin )  / span) * 255.;
    }

    return image_8;
}

/**
 * @brief 检查并输出3x3像素格中九个像素
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标 cols
 * @param[in]   y       点的纵坐标 rows
 */
void CheckBlock3x3(cv::Mat& image, int x, int y)
{   
    if(x == 0|| y ==0 || x == image.rows || y == image.cols)
    {
        cout << "The point is on the edge!" <<endl;
        return;
    }
    // CV_8UC1
    if(image.type() == 0)
    {
    cout << "Good! 8 bit" <<endl;

        for(int i = x-1; i <= x+1 ; i++)
        {
            for(int j = y-1; j <= y+1 ; j++)
            {
                cout << (int)(image.at<uchar>(j,i)) << " ";
            }
            cout << endl;
        }
        return;
    }
    else if(image.type() == 2)
    {
    cout << "Good! 16 bit" <<endl;

        for(int i = x-1; i <= x+1 ; i++)
        {
            for(int j = y-1; j <= y+1 ; j++)
            {
                cout << (int)(image.at<ushort>(j,i)) << " ";
            }
            cout << endl;
        }
        return;
    }
}

/**
 * @brief 使用Sobel算子计算的方式计算梯度X,8位or16位
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  Gx      x方向梯度   
 */
double PointGradXSobel(cv::Mat& image, int x, int y)
{
    double Gx;
    if(image.type() == 0)
    {
        Gx = abs((image.at<uchar>(y-1,x+1) + 2*image.at<uchar>(y,x+1) + image.at<uchar>(y+1,x+1))
            -(image.at<uchar>(y-1,x-1) + 2*image.at<uchar>(y,x-1) + image.at<uchar>(y+1,x-1)));
    }
    else if(image.type() == 2)
    {
        Gx = abs((image.at<ushort>(y-1,x+1) + 2*image.at<ushort>(y,x+1) + image.at<ushort>(y+1,x+1))
            -(image.at<ushort>(y-1,x-1) + 2*image.at<ushort>(y,x-1) + image.at<ushort>(y+1,x-1)));
    }

    return Gx;
}

/**
 * @brief 使用Sobel算子计算的方式计算梯度Y,8位or16位
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  Gy      y方向梯度   
 */
double PointGradYSobel(cv::Mat& image, int x, int y)
{
    double Gy;
    if(image.type() == 0)
    {
        Gy = abs((image.at<uchar>(y-1,x-1) + 2*image.at<uchar>(y-1,x) + image.at<uchar>(y-1,x+1))
            -(image.at<uchar>(y+1,x-1) + 2*image.at<uchar>(y+1,x) + image.at<uchar>(y+1,x+1)));
    }
    else if(image.type() == 2)
    {
        Gy = abs((image.at<ushort>(y-1,x-1) + 2*image.at<ushort>(y-1,x) + image.at<ushort>(y-1,x+1))
            -(image.at<ushort>(y+1,x-1) + 2*image.at<ushort>(y+1,x) + image.at<ushort>(y+1,x+1)));
    }

    return Gy;
}

/**
 * @brief 使用Sobel算子计算的方式计算梯度,8位or16位
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  grad    x,y方向   
 */
Eigen::Vector2d PointGradSobel(cv::Mat& image, int x, int y)
{
    double Gx, Gy;
    if(image.type() == 0)
    {
        Gx = abs((image.at<uchar>(y-1,x+1) + 2*image.at<uchar>(y,x+1) + image.at<uchar>(y+1,x+1))
            -(image.at<uchar>(y-1,x-1) + 2*image.at<uchar>(y,x-1) + image.at<uchar>(y+1,x-1)));
        Gy = abs((image.at<uchar>(y-1,x-1) + 2*image.at<uchar>(y-1,x) + image.at<uchar>(y-1,x+1))
            -(image.at<uchar>(y+1,x-1) + 2*image.at<uchar>(y+1,x) + image.at<uchar>(y+1,x+1)));
    }
    else if(image.type() == 2)
    {
        Gx = abs((image.at<ushort>(y-1,x+1) + 2*image.at<ushort>(y,x+1) + image.at<ushort>(y+1,x+1))
            -(image.at<ushort>(y-1,x-1) + 2*image.at<ushort>(y,x-1) + image.at<ushort>(y+1,x-1)));
        Gy = abs((image.at<ushort>(y-1,x-1) + 2*image.at<ushort>(y-1,x) + image.at<ushort>(y-1,x+1))
            -(image.at<ushort>(y+1,x-1) + 2*image.at<ushort>(y+1,x) + image.at<ushort>(y+1,x+1)));
    }
    Eigen::Vector2d grad;
    grad << Gx, Gy;
    return grad;
}

/**
 * @brief 对整个图进行sobel处理,输出GradX图,cv16uc1
 * TODO: 添加八位的输入
 * @param[in]   image   输入图像
 * @param[out]  GradX   输出x方向梯度图像
 */
cv::Mat GetGradXImage(cv::Mat& image)
{
    cv::Mat GradX(image.size(), CV_16UC1);

    for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
    {
        // 边沿的像素点计算不了,移除
        // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
        // 将错误的点进行去除
        if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
        {
            GradX.at<ushort>(i,j) = 0;
        }
        else
        {
            GradX.at<ushort>(i,j) = (double)(PointGradXSobel(image,j,i));
        }
    }
    return GradX;
}


/**
 * @brief 对整个图进行sobel处理,输出GradX图
 * @param[in]   image   输入图像
 * @param[out]  GradX   输出x方向梯度图像
 */
cv::Mat GetGradYImage(cv::Mat& image)
{
    cv::Mat GradY(image.size(), CV_16UC1);

    for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
    {
        // 边沿的像素点计算不了,移除
        // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
        // 将错误的点进行去除
        if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
        {
            GradY.at<ushort>(i,j) = 0;
        }
        else
        {
            GradY.at<ushort>(i,j) = (double)(PointGradYSobel(image,j,i));
        }
    }
    return GradY;
}

/**
 * @brief 对整个图进行sobel处理,输出GradX * GradX图,cv16uc1
 * @param[in]   image    输入图像
 * @param[out]  GradXX   输出X方向梯度图像
 */
cv::Mat GetGradXXImage(cv::Mat& image)
{
    cv::Mat GradXX(image.size(), CV_16UC1);

    for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
    {
        // 边沿的像素点计算不了,移除
        // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
        // 将错误的点进行去除
        if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
        {
            GradXX.at<ushort>(i,j) = 0;
        }
        else
        {
            GradXX.at<ushort>(i,j) = (double)(PointGradXSobel(image,j,i) * PointGradXSobel(image,j,i));
        }
    }
    return GradXX;
}

/**
 * @brief 对整个图进行sobel处理,输出GradY * GradY图
 * @param[in]   image       输入图像
 * @param[out]  GradYY      输出Y方向梯度图像
 */
cv::Mat GetGradYYImage(cv::Mat& image)
{
    cv::Mat GradYY(image.size(), CV_16UC1);

    for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
    {
        // 边沿的像素点计算不了,移除
        // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
        // 将错误的点进行去除
        if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
        {
            GradYY.at<ushort>(i,j) = 0;
        }
        else
        {
            GradYY.at<ushort>(i,j) = (double)(PointGradYSobel(image,j,i) * PointGradYSobel(image,j,i));
        }
    }
    return GradYY;
}

/**
 * @brief 对整个图进行sobel处理,输出GradX * GradY图
 * @param[in]   image       输入图像
 * @param[out]  GradXY      输出X方向梯度 * Y方向梯度图像
 */
cv::Mat GetGradXYImage(cv::Mat& image)
{
    cv::Mat GradXY(image.size(), CV_16UC1);

    for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
    {
        // 边沿的像素点计算不了,移除
        // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
        // 将错误的点进行去除
        if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
        {
            GradXY.at<ushort>(i,j) = 0;
        }
        else
        {
            GradXY.at<ushort>(i,j) = (double)(PointGradXSobel(image,j,i) * PointGradYSobel(image,j,i));
        }
    }
    return GradXY;
}

/**
 * @brief 对窗口内的XY,XX,YY图像做boxfilter
 * @param[in]   GradXX      Gx*Gx图
 * @param[in]   GradXY      Gx*Gy图
 * @param[in]   GradYY      Gy*Gy图
 * @param[in]   blocksize   滤波的窗口大小
 * @param[in]   normal      是否进行均一化
 */
void BoxFilter3(cv::Mat& GradXX, cv::Mat& GradXY, cv::Mat& GradYY, int blocksize, bool normal)
{
    cout << "In the Box filter stage, using OpenCV functions." << endl;
    cv::boxFilter(GradXX, GradXX, GradXX.depth(),cv::Size(blocksize, blocksize),cv::Point(-1,-1), normal);
    cv::boxFilter(GradXY, GradXY, GradXY.depth(),cv::Size(blocksize, blocksize),cv::Point(-1,-1), normal);
    cv::boxFilter(GradYY, GradYY, GradYY.depth(),cv::Size(blocksize, blocksize),cv::Point(-1,-1), normal);
    return;
}

/**
 * @brief 对整个图进行sobel处理,输出Grad图
 * @param[in]   image   输入图像
 * @param[out]  Grad    输出梯度图像
 */
cv::Mat GetGradImage(cv::Mat& image)
{
    if(image.type() == 2)
    {
        cv::Mat Grad(image.size(), CV_16UC1);

        for(int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            // 边沿的像素点计算不了,移除
            // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
            // 将错误的点进行去除
            if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
            {
                Grad.at<ushort>(i,j) = 0;
            }
            else
            {
                // Grad = sqrt(Gx^2 + Gy^2)
                Grad.at<ushort>(i,j) = (double)sqrt(pow((double)(PointGradXSobel(image,j,i)),2)+pow((double)(PointGradYSobel(image,j,i)),2));
            }
        }
        return Grad;
    }
    else if(image.type() == 0)
    {
        cv::Mat Grad(image.size(), CV_8UC1);

        for(int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            // 边沿的像素点计算不了,移除
            // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
            // 将错误的点进行去除
            if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
            {
                Grad.at<uchar>(i,j) = 0;
            }
            else
            {
                // Grad = sqrt(Gx^2 + Gy^2)
                Grad.at<uchar>(i,j) = (float)sqrt(pow((float)(PointGradXSobel(image,j,i)),2)+pow((float)(PointGradYSobel(image,j,i)),2));
            }
        }
        return Grad;
    }
}

/**
 * @brief 对整个图进行sobel处理,输出Grad图
 * @param[in]   image   输入图像
 * @param[out]  Grad    输出梯度图像
 */
cv::Mat GetGradImageByAdd(cv::Mat& image)
{
    if(image.type() == 2)
    {
        cv::Mat Grad(image.size(), CV_16UC1);

        for(int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            // 边沿的像素点计算不了,移除
            // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
            // 将错误的点进行去除
            if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
            {
                Grad.at<ushort>(i,j) = 0;
            }
            else
            {
                // Grad = sqrt(Gx^2 + Gy^2)
                Grad.at<ushort>(i,j) = (double)(PointGradXSobel(image,j,i))+(double)(PointGradYSobel(image,j,i));
            }
        }
        return Grad;
    }
    else if(image.type() == 0)
    {
        cv::Mat Grad(image.size(), CV_8UC1);

        for(int i = 0; i < image.rows; i++)
        for(int j = 0; j < image.cols; j++)
        {
            // 边沿的像素点计算不了,移除
            // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
            // 将错误的点进行去除
            if(i -1 < 0 || j-1 <0 || j+1 > image.cols || i+1 > image.rows -1 || (i == 478 && j == 639))
            {
                Grad.at<uchar>(i,j) = 0;
            }
            else
            {
                // Grad = sqrt(Gx^2 + Gy^2)
                Grad.at<uchar>(i,j) = (float)(PointGradXSobel(image,j,i))+(float)(PointGradYSobel(image,j,i));
            }
        }
        return Grad;
    }
    
}

/**
 * @brief 计算该点梯度特征值之比
 *        ratio = a1/a2 < 1
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  ratio   点梯度特征值比值
 */
double GetEigenRatio(cv::Mat& image, int x, int y)
{
    double Gx, Gy;
    Gx = PointGradXSobel(image, x, y);
    Gy = PointGradYSobel(image, x, y);

    Eigen::Matrix2d cov;
    cov << Gx*Gx , Gx*Gy , Gx*Gy , Gy*Gy;
    cout << " THe Cov matrix is " << endl << cov <<endl;
    Eigen::EigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Matrix2d D = solver.pseudoEigenvalueMatrix();
    cout << "The Eigen matrix is " << endl << D << endl;
    double eigen1 = D(0,0);
    double eigen2 = D(1,1);
    // 如果一个特征值为零,那么直接比值就是0

    if(!eigen1 || !eigen2) return 0;

    if(eigen1 >= eigen2)
        return (double)(eigen2 / eigen1);
    else
        return (double)(eigen1 / eigen2);
}

/**
 * @brief 计算该点的响应值
 *        R = det(M) - a*trace(M)^2
 * @param[in]   grad    该点的x,y梯度
 * @param[in]   alpha   权重值
 * @param[out]  R       角点响应值
 */
double CornerScore(Eigen::Vector2d& grad, double alpha)
{
    // calculate M;
    double Ixx, Ixy, Iyy;
    Ixx = grad[0] * grad[0]; // Ixx = Ix * Ix
    Ixy = grad[0] * grad[1]; // Ixy = Ix * Iy
    Iyy = grad[1] * grad[1]; // Iyy = Iy * Iy

    // calculate R;
    double R;
    R = (double)(Ixx*Iyy - Ixy*Ixy - alpha * (Ixx+Iyy) * (Ixx+Iyy));
    return R;
}


/**
 * @brief 判断图像类型是否为16位
 * @param[in]   image   16位or8位图像
 * @param[out]  res     输出的布尔变量    
 */
bool Check16BitImage(cv::Mat& image)
{
    if(image.type() == 16)
    {
        cout << "The image type is CV_8UC3." << endl;
        return false;
    }
    else if(image.type() == 2)
    {
        cout << "The image type is CV_16UC1." << endl;
        return true;
    }
    else
    {
        return false;
    }
    
}

/**
 * @brief 判断图像类型是否为8位3通道
 * @param[in]   image   16位or8位图像
 * @param[out]  res     输出的布尔变量    
 */
bool Check8BitImage(cv::Mat& image)
{
    if(image.type() == 16)
    {
        cout << "The image type is CV_8UC3." << endl;
        return true;
    }
    else if(image.type() == 2)
    {
        cout << "The image type is CV_16UC1." << endl;
        return false;
    }
    else
    {
        return false;
    }
    
}

/**
 * @brief 图像提取FAST
 * @param[in]   image       16位or8位图像
 * @param[in]   threshold   阈值
 * @param[out]  keypoints   特征点输出
 */
vector<cv::KeyPoint> ComputeFAST(cv::Mat& image, const int threshold)
{
    vector<cv::KeyPoint> keypoints, keypoints_flip;
    if(Check8BitImage(image))
    {
        cv::FAST(image, keypoints, threshold);
    }
    else
    {
        cv::Mat image_flip;
        cv::flip(image, image_flip, 1);
        cv::FAST(image, keypoints, threshold);
        cv::FAST(image_flip, keypoints_flip, threshold);

        // 获得左半边的角点
        for (auto &iter:keypoints)
        {
            iter.pt.x = iter.pt.x/2;
        }
        //获得右半边的角点
        for(auto &iter:keypoints_flip)
        {
            iter.pt.x = image.cols - iter.pt.x/2;
            keypoints.push_back(iter);
        }
    }

    return keypoints;
}

/**
 * @brief 在图像上绘制角点并显示
 * @param[in]   image       图像
 * @param[in]   keypoints   角点
 */
void DrawKeypoints(cv::Mat& image, vector<cv::KeyPoint>& keypoints)
{
    cv::Mat show;
    cv::drawKeypoints(image, keypoints, show);
    cv::imshow("Keypoints", show);
    cv::waitKey(0);
}

/**
 * @brief 8位灰度图像提取直线特征,使用FastLineDetector
 * @param[in]   image                   原图像
 * @param[in]   length_threshold        直线线段长度阈值
 * @param[in]   distance_threshold      超出假定线的点的阈值
 * @param[in]   canny_th1               First threshold for hysteresis procedure in canny
 * @param[in]   canny_th2               Second threshold for hysteresis procedure in canny
 * @param[in]   canny_aperture_size     Aperturesize for the sobel operator in Canny
 * @param[in]   do_merge                是否对线段进行merge
 * @param[in]   lines                   输出的结果线段
 */
vector<cv::Vec4f> ComputeLine(cv::Mat& image,
                              int length_threshold,
                              float distance_threshold,
                              int canny_th1,
                              int canny_th2,
                              int canny_aperture_size,
                              bool do_merge)
{
    // 检查是否是灰度图
    if(image.type() != 0)
        cerr << "Please input a gray image!!" <<endl;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
            distance_threshold, canny_th1, canny_th2, canny_aperture_size,
            do_merge);
    vector<cv::Vec4f> lines;
    fld->detect(image, lines);

    return lines;
}

/**
 * @brief 8位灰度图像提取直线特征,使用FastLineDetector
 * @param[in]   image                   原图像
 * @param[in]   length_threshold        直线线段长度阈值
 * @param[in]   distance_threshold      超出假定线的点的阈值
 * @param[in]   canny_th1               First threshold for hysteresis procedure in canny
 * @param[in]   canny_th2               Second threshold for hysteresis procedure in canny
 * @param[in]   canny_aperture_size     Aperturesize for the sobel operator in Canny
 * @param[in]   do_merge                是否对线段进行merge
 * @param[in]   lines                   输出的结果线段
 */
void ComputeAndDrawLine(cv::Mat& image,
                        int length_threshold,
                        float distance_threshold,
                        int canny_th1,
                        int canny_th2,
                        int canny_aperture_size,
                        bool do_merge)
{
    if(image.type() != 0)
        cerr << "Please input a gray image!!" <<endl;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
            distance_threshold, canny_th1, canny_th2, canny_aperture_size,
            do_merge);
    vector<cv::Vec4f> lines;
    fld->detect(image, lines);
    cv::Mat out2(image);
    fld->drawSegments(out2, lines);
    cv::imshow("Lines", out2);
    cv::waitKey(0);
}

// void myHarries( const cv::Mat& src, cv::Mat& eigenv, int block_size, int aperture_size, double k)
// {
//     eigenv.create(src.size(), CV_32F);
//     cv::Mat Dx, Dy;
//     //soble operation get Ix, Iy
//     cv::Sobel(src, Dx, CV_32F, 1, 0, aperture_size);
//     cv::Sobel(src, Dy, CV_32F, 0, 1, aperture_size);
 
//     //get covariance matrix
//     cv::Size size = src.size();
//     cv::Mat cov(size, CV_32FC3);     //创建一个三通道cov矩阵分别存储[Ix*Ix, Ix*Iy, Iy*Iy];
 
//     for( int i=0; i < size.height;i++)
//     {
//         float* cov_data = cov.ptr<float>(i);
//         const float* dxdata = Dx.ptr<float>(i);
//         const float* dydata = Dy.ptr<float>(i);
 
//         for( int j=0; j < size.width; j++ )
//         {
//             float dx = dxdata[j];
//             float dy = dydata[j];
 
//             cov_data[j*3] = dx * dx;    //即  Ix*Ix
//             cov_data[j*3 + 1] = dx*dy;  //即  Ix*Iy
//             cov_data[j*3 + 2] = dy*dy;  //即  Iy*Iy
//         }
//     }
 
 
//     //方框滤波W(x,y)卷积, 也可用高斯核加权...
//     //W(Y,Y)与矩阵cov卷积运算得到 H 矩阵,后面通过H矩阵的特征值决定是否是角点
//     cv::boxFilter(cov, cov, cov.depth(), cv::Size(block_size,block_size),cv::Point(-1,-1),false);
 
//     //cale Harris
//     size = cov.size();
//     if( cov.isContinuous() && eigenv.isContinuous())
//     {
//         size.width *= size.height;
//         size.height = 1;
//         //cout << "yes"<<endl;
//     }
 
//     //此处计算响应R= det(H) - k*trace(H)*trace(H);
//     for (int i = 0; i < size.height; i++)
//     {
//         const float* covPtr = cov.ptr<float>(i);
//         float* dstPtr = eigenv.ptr<float>(i);
 
//         for( int j = 0; j < size.width; j++)
//         {
//             float a = covPtr[j*3];
//             float b = covPtr[j*3 + 1];
//             float c = covPtr[j*3 + 2];
 
//             //根据公式 R = det(H) - k*trace(H)*trace(H);
//             dstPtr[j] = (float)(a*c - b*b - k * (a + c)*(a + c));
//         }
//     }
 
//     double max, min;
//     minMaxLoc(eigenv, &min, &max);
//     //cout<< max << endl;
//     double threshold = 0.1*max;
//     cv::threshold(eigenv, eigenv,threshold, 1,cv::ThresholdTypes::THRESH_BINARY);   //eigenv的类型是CV_32F,
//     cv::imshow("eigenv", eigenv);
// }