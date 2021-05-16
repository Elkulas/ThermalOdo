// 一些简单的算法

#include "algorithm.h"

/**
 * @brief 单层光流
 * @param [in] img1 第一张图像
 * @param [in] img2 第二张图像
 * @param [in] kp1 第一张图像中特征点
 * @param [in|out] kp2 第二张图像中特征点,如果为空,使用第一张图像中作为初始估计
 * @param [out] success 如果一个特征点成功追踪,那么便设置为真
 * @param [in] 是否使用反向光流
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 20;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        // 如果没有初始值,那么初始kp2也就是使用kp1,所以这边计算得到的dxdy就都是0
        // 此处的dxdy都是针对当前这个点的
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                // 定位的点加上patch的大小在图片范围外面了
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // 开始对8x8窗口内像素进行遍历工作
                    // 获得template上的坐标
                    float u1 = float(kp.pt.x + x);
                    float v1 = float(kp.pt.y + y);
                    // 获得第二幅图像上更新的坐标
                    float u2 = float(u1 + dx);
                    float v2 = float(v1 + dy);

                    double error = 0;
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian 这边就是计算像素梯度的雅克比
                        // dIx/du 计算第二幅图像中在update之后的点x轴上的图像梯度
                        J.x() = double(GetPixelValue(img2, u2+1, v2) - GetPixelValue(img2, u2-1, v2))/2;
                        // 计算第二幅图像中在update之后的点y轴上的图像梯度
                        J.y() = double(GetPixelValue(img2, u2, v2+1) - GetPixelValue(img2, u2, v2-1))/2;
                        // 计算error
                        error = double(GetPixelValue(img2, u2, v2) - GetPixelValue(img1, u1, v1));

                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        // 使用在template图像上点的梯度
                        J.x() = double(GetPixelValue(img1, u1 + 1, v1) - GetPixelValue(img1, u1 - 1, v1))/2;
                        J.y() = double(GetPixelValue(img1, u1, v1 + 1) - GetPixelValue(img1, u1, v1 - 1))/2;
                        error = double(GetPixelValue(img2, u2, v2) - GetPixelValue(img1, u1, v1));
                    }

                    // compute H, b and set cost;
                    // 这里注意之前求解得到的J是2x1的，但实际上雅克比矩阵应该是1x2的
                    //cout << "之前:::::: "<<endl << J <<endl;
                    Eigen::Matrix<double, 1, 2> J12;
                    J12 = J.transpose();
                    //cout << "之后：：：：：：" << endl << J12 << endl;
                    // H = J^T*J
                    H += J12.transpose() * J12;
                    // b = - J^T * error
                    b += -J12.transpose() * error;
                    cost += pow(error, 2);
                }

            // compute update
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                //cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                //cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        } // 结束一个特征点的GN迭代

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {

    // parameters
    //int pyramids = 4;
    //double pyramid_scale = 0.5;
    //double scales[] = {1.0, 0.5, 0.25, 0.125};

    int pyramids = 4;
    double pyramid_factor = 2;
    double pyramid_scale = (1.0) / pyramid_factor;

    vector<double> scales;
    double initial = 1.0;
    scales.emplace_back(initial);
    for(int i = 1; i < pyramids ;i ++)
    {
        initial *= pyramid_scale;
        scales.emplace_back(initial);
    }
    for(int i = 0; i < scales.size(); i++)
    {
        cout << scales[i] <<endl;
    }

    // create pyramids
    cout << "= = = = GENERATING PYRAMIDS = = = = " << endl;
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++) {
        // 生成不同分辨率的图像，并存放到pyr容器中
        Mat img1_temp, img2_temp;
        // 使用resize函数对图像进行分辨率重构
        // 同时对第一张和第二张图片进行分辨率重构
        // vector序号从低到高说明金字塔层数从低到高
        cv::resize(img1, img1_temp, cv::Size(img1.cols * scales[i], img1.rows * scales[i]));
        cv::resize(img2, img2_temp, cv::Size(img2.cols * scales[i], img2.rows * scales[i]));
        pyr1.push_back(img1_temp);
        pyr2.push_back(img2_temp);
        // 输出进行检查
        cout << "Pyramid " << i << "has " << img1_temp.cols << " x " << img2_temp.rows << endl;

    }
    // 本质上说就是从较低分辨率图像开始检测光流，然后传递到下一层
    // 从金字塔高层到底层
    vector<KeyPoint> kp2_now;
    vector<KeyPoint> kp2_last;
    vector<bool> vsucc;
    for (int i = pyramids -1; i >= 0; i--)
    {
        cout << "At pyramid " << i << endl;
        // 生成一个对template图像特征点进行缩放过后的容器
        vector<KeyPoint> kp1_now;
        // 对所有kp1进行尺度上的转化
        
        for(int j = 0; j < kp1.size(); j++)
        {
            KeyPoint kp1_temp;
            kp1_temp = kp1[j]; // 这里之前写了一个bug，是kp1[j],不是kp1[i]
            // 乘以该层的缩放系数
            kp1_temp.pt = kp1_temp.pt * scales[i];
            kp1_now.push_back(kp1_temp);
            // 对所有上一层的kp2进行尺度上的转化, 进行放大
            if (i < pyramids -1)
            {
                KeyPoint kp2_temp;
                kp2_temp = kp2_last[j];
                kp2_temp.pt /= pyramid_scale;
                kp2_now.push_back(kp2_temp);
            }
        }
        vsucc.clear();
        // 获得该层第二幅图像中的对应特征点
        // 所以之前但产能中设计kp2存在初值就是为了给这里服务的
        OpticalFlowSingleLevel(pyr1[i], pyr2[i], kp1_now, kp2_now, vsucc, inverse);
        cout<<"pyramid: "<<i<<" kp2_last size: "<<kp2_last.size()<<"kp2_nowsize "<<kp2_now.size()<<endl;
        if(i == 3)
        for(int k = 0; k < kp2_now.size(); k++) 
        cout << kp2_now[k].pt << endl;
        // 将上一层的kp2存入last，将now清空
        kp2_last.clear();
        kp2_last.swap(kp2_now);
    } // 结束金字塔层数的迭代

    kp2 = kp2_last;
    success = vsucc;

}

/**
 * @brief 特征点筛选,使用最主流的结果进行筛选与选取, 筛选最主流的长度以及最主流的角度
 * @param pt1 第一张图片上的特征点
 * @param pt2 第二张图片追踪的特征点
 * @param status 追踪的结果
 * TODO: 把这个函数的形参写成模板类,不然status卡太死了
 * TODO: 添加长度筛选
 */
void SelectMajority(vector<cv::Point2f>& pt1, vector<cv::Point2f>& pt2, vector<uchar>& status)
{
    // Step1: 获取角度直方图
    // 角度的容器
    const int HISTO_LENGTH = 30;
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    // 获得一个factor,放到不同的box中
    const float factor = 1.0f/HISTO_LENGTH;

    // 计算特征点追踪的角度所在的直方图
    for(int i = 0; i < status.size(); i++)
    {
        // 如果成功追踪
        if(status[i])
        {
            // 计算两个特征点的角度
            float rot = atan2((float)(pt2[i].x - pt1[i].x),(float)(pt1[i].y - pt2[i].y)) / 3.1415926 * 180.0f;
            
            if(rot < 0) rot+=360.f;
            // 将角度差放到bin中
            int bin = round(rot * factor);
            //cout << rot << endl;
            if(bin == HISTO_LENGTH)
                bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(i);
        }
    }

    // Test output all situation
    // for(int i = 0; i < HISTO_LENGTH; i++)
    // {
    //     cout << i << '\t' << rotHist[i].size() << endl;
    // }

    // Step 2: 筛除直方图中非主流部分
    // 角度一致性检测,保证匹配点对方向上的一致,而不是出现角度差别较大的情况

    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    // 筛选出在旋转角度差落在直方图区间内数量最多的前三个bin 的索引
    ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

    cout << ind1 << '\t' << ind2 << '\t' << ind3 << endl;

    // 获得三个bin之后对原来的status进行修改
    for(int i = 0; i < HISTO_LENGTH; i++)
    {
        // 筛选使用的是两个bin进筛选
        if(i == ind1 || i == ind2)
        continue;

        // 删除掉那些不在前三bin中的匹配对，因为他们不符合主流的方向
        for(size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
        {
            int idx1 = rotHist[i][j];
            if(status[idx1] > 0)
            {
                status[idx1] = 0;
            }
        }
    }
}

void SelectMajorityBool(vector<cv::Point2f>& pt1, vector<cv::Point2f>& pt2, vector<bool>& status)
{
    // Step1: 获取角度直方图
    // 角度的容器
    const int HISTO_LENGTH = 30;
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    // 获得一个factor,放到不同的box中
    const float factor = 1.0f/HISTO_LENGTH;

    // 计算特征点追踪的角度所在的直方图
    for(int i = 0; i < status.size(); i++)
    {

        // 如果成功追踪
        if(status[i])
        {

            // 计算两个特征点的角度
            float rot = atan2((float)(pt2[i].x - pt1[i].x),(float)(pt1[i].y - pt2[i].y)) / 3.1415926 * 180.0f;
            
            if(rot < 0) rot+=360.f;
            // 将角度差放到bin中
            int bin = round(rot * factor);
            //cout << rot << endl;
            if(bin == HISTO_LENGTH)
                bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(i);
        }

    }

    // Test output all situation
    // for(int i = 0; i < HISTO_LENGTH; i++)
    // {
    //     cout << i << '\t' << rotHist[i].size() << endl;
    // }

    // Step 2: 筛除直方图中非主流部分
    // 角度一致性检测,保证匹配点对方向上的一致,而不是出现角度差别较大的情况

    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    // 筛选出在旋转角度差落在直方图区间内数量最多的前三个bin 的索引
    ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

    cout << ind1 << '\t' << ind2 << '\t' << ind3 << endl;

    // 获得三个bin之后对原来的status进行修改
    for(int i = 0; i < HISTO_LENGTH; i++)
    {
        // 筛选使用的是两个bin进筛选
        if(i == ind1 || i == ind2)
        continue;

        // 删除掉那些不在前三bin中的匹配对，因为他们不符合主流的方向
        for(size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
        {
            int idx1 = rotHist[i][j];
            if(status[idx1])
            {
                // cout<< "hihihihi" <<endl;
                status[idx1] = false;
            }
        }
    }
}

/**
 * @brief 筛选直方图最多的三个bin
 * @param histo     输入的直方图
 * @param L         bin的个数
 * @param ind1      最多的bin的index
 * @param ind2      次多的bin的index
 * @param ind3      次次多的bin的index
 * 
 */
void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}



/**
 * @brief 基础LBP算法 https://blog.csdn.net/simonforfuture/article/details/78852173
 * 
 * 
 * 
 */

void getOriginLBPFeature(cv::Mat& _src, cv::Mat& _dst)
{
    Mat src = _src;
    _dst.create(src.rows-2,src.cols-2,CV_8UC1);
    Mat dst = _dst;
    dst.setTo(0);
    for(int i = 1; i < src.rows - 1; i++)
    {
        for(int j = 1; j < src.cols - 1; j++)
        {
            uchar center = src.at<uchar>(i,j);
            unsigned char lbpCode = 0;
            lbpCode |= (src.at<uchar>(i-1,j-1) > center) << 7;
            lbpCode |= (src.at<uchar>(i-1,j  ) > center) << 6;
            lbpCode |= (src.at<uchar>(i-1,j+1) > center) << 5;
            lbpCode |= (src.at<uchar>(i  ,j+1) > center) << 4;
            lbpCode |= (src.at<uchar>(i+1,j+1) > center) << 3;
            lbpCode |= (src.at<uchar>(i+1,j  ) > center) << 2;
            lbpCode |= (src.at<uchar>(i+1,j-1) > center) << 1;
            lbpCode |= (src.at<uchar>(i  ,j-1) > center) << 0;
            dst.at<uchar>(i-1,j-1) = lbpCode;
        }
    }
}

// 16位 original 的LBP计算
void getOriginLBPFeature16(cv::Mat& _src, cv::Mat& _dst)
{
    Mat src = _src;
    _dst.create(src.rows-2,src.cols-2,CV_8UC1);
    Mat dst = _dst;
    dst.setTo(0);
    for(int i = 1; i < src.rows - 1; i++)
    {
        for(int j = 1; j < src.cols - 1; j++)
        {
            ushort center = src.at<ushort>(i,j);
            unsigned char lbpCode = 0;
            lbpCode |= (src.at<ushort>(i-1,j-1) > center) << 7;
            lbpCode |= (src.at<ushort>(i-1,j  ) > center) << 6;
            lbpCode |= (src.at<ushort>(i-1,j+1) > center) << 5;
            lbpCode |= (src.at<ushort>(i  ,j+1) > center) << 4;
            lbpCode |= (src.at<ushort>(i+1,j+1) > center) << 3;
            lbpCode |= (src.at<ushort>(i+1,j  ) > center) << 2;
            lbpCode |= (src.at<ushort>(i+1,j-1) > center) << 1;
            lbpCode |= (src.at<ushort>(i  ,j-1) > center) << 0;
            dst.at<uchar>(i-1,j-1) = lbpCode;
        }
    }
}

// 两个size相同的图片做差得到的残差值
void getResidual(cv::Mat& img1, cv::Mat& img2, cv::Mat& dst)
{
    // 判断两个是不是一致的大小
    if(img1.size() != img2.size())
    {
        cout << "Size not Match" << endl;
        return;
    }

    dst.create(img1.rows, img2.cols, CV_8UC1);

    double sum = 0;

    for(int i = 0; i < img1.rows; i++)
    {
        for(int j = 0; j < img2.cols; j++)
        {
            uchar res = abs(img1.at<uchar>(i,j) - img2.at<uchar>(i,j));
            sum+=res;
            dst.at<uchar>(i, j) = res;
        }
    }

    cout << "Residual: " << (double)sum/(double)(img1.rows*img1.cols) << endl;
}

//圆形LBP特征计算，效率优化版本，声明时默认neighbors=8
void getCircularLBPFeatureOptimization(cv::Mat& _src, cv::Mat& _dst, int radius, int neighbors)
{
    Mat src = _src;
    //LBP特征图像的行数和列数的计算要准确
    _dst.create(src.rows-2*radius,src.cols-2*radius,CV_8UC1);
    Mat dst = _dst;
    dst.setTo(0);
    for(int k=0;k<neighbors;k++)
    {
        //计算采样点对于中心点坐标的偏移量rx，ry
        float rx = static_cast<float>(radius * cos(2.0 * CV_PI * k / neighbors));
        float ry = -static_cast<float>(radius * sin(2.0 * CV_PI * k / neighbors));
        //为双线性插值做准备
        //对采样点偏移量分别进行上下取整
        int x1 = static_cast<int>(floor(rx));
        int x2 = static_cast<int>(ceil(rx));
        int y1 = static_cast<int>(floor(ry));
        int y2 = static_cast<int>(ceil(ry));
        //将坐标偏移量映射到0-1之间
        float tx = rx - x1;
        float ty = ry - y1;
        //根据0-1之间的x，y的权重计算公式计算权重，权重与坐标具体位置无关，与坐标间的差值有关
        float w1 = (1-tx) * (1-ty);
        float w2 =    tx  * (1-ty);
        float w3 = (1-tx) *    ty;
        float w4 =    tx  *    ty;
        //循环处理每个像素
        for(int i=radius;i<src.rows-radius;i++)
        {
            for(int j=radius;j<src.cols-radius;j++)
            {
                //获得中心像素点的灰度值
                uchar center = src.at<uchar>(i,j);
                //根据双线性插值公式计算第k个采样点的灰度值
                float neighbor = src.at<uchar>(i+x1,j+y1) * w1 + src.at<uchar>(i+x1,j+y2) *w2 \
                    + src.at<uchar>(i+x2,j+y1) * w3 +src.at<uchar>(i+x2,j+y2) *w4;
                //LBP特征图像的每个邻居的LBP值累加，累加通过与操作完成，对应的LBP值通过移位取得
                dst.at<uchar>(i-radius,j-radius) |= (neighbor>center) <<(neighbors-k-1);
            }
        }
    }
}

/**
 * @brief CV8UC1图像直方图获取
 * @param pt1 第一张图片上的特征点
 * @param pt2 第二张图片追踪的特征点
 * @param 
 */
void getHist(cv::Mat& _src, int box_num)
{
    // 直方图容器
    vector<int> hist(box_num);

    const float factor = 1.0f / box_num;
    if(_src.type() != 0)
    {
        cerr << "Error Type" << endl;
    }

    for(int i = 0; i < _src.rows; i++)
    for(int j = 0; j < _src.cols; j++)
    {
        // 获取像素值
        double pvalue = _src.at<uchar>(i,j);
        

    }




}
