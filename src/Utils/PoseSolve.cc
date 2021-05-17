
// 代码参考ORBmatcher中的initialize函数,直接进行求解

#include "PoseSolve.h"




void GetPose(vector<cv::Point2f>& vPts1, vector<cv::Point2f>& vPts2, vector<uchar>& vStatus,
             Mat& R21, cv::Mat& t21, cv::Mat K, int MaxIter)
{
    // Step1: 重置匹配关系
    // 追踪成功的点
    int succ_count = 0;
    cout << "debug pts size" << '\t' << vPts1.size() << endl;
    // 筛选得到成功追踪的点
    for(int i = 0; i < vStatus.size(); i++)
    {
        // 如果被成功追踪
        if(vStatus[i])
        {
            if(i == succ_count)
            {
                succ_count++;
            }
            else
            {
                vPts1[succ_count] = vPts1[i];
                vPts2[succ_count] = vPts2[i];
                succ_count++;
            }
        }
    }
    cout << "debug succ_count " << succ_count << endl;

    // 构建新的容器来存储特征点索引,并预分配空间
    vector<size_t> vAllIndices;
    vAllIndices.reserve(succ_count);

    // 在RANSAC的某次迭代中,还可以被抽取用来作为数据样本的特征点对的索引
    // 可用索引
    vector<size_t> vAvailableIndices;

    // 初始化所有的点对的索引
    for(int i = 0; i < succ_count; i++)
    {
        vAllIndices.push_back(i);
    }

    // Step2: 在所有匹配特征点对中随机选择8对匹配特征点为一组, 用来估计初始H和F
    // 一共选择MaxIter组
    vector<vector<size_t> > vSets(MaxIter,vector<size_t>(8,0));
    

    // 随机数
    RNG rng(0);

    // 开始迭代
    for(int it = 0; it < MaxIter; it++)
    {
        // 迭代开始时所有点都是可以用的
        vAvailableIndices = vAllIndices;
        for(size_t j = 0; j < 8; j++)
        {
            // 随机产生一对点的id，范围从0 - (N-1)
            int randi = rng.uniform(0,vAvailableIndices.size()-1);
            // idx表示哪一个索引对应的特征点对被选中
            int idx = vAvailableIndices[randi];

            // 将本次迭代中这个选中的第j个特征点对的索引添加到mvSets中
            // 注意这里存的是索引
            // mvSets 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点,实际上是八对
            vSets[it][j] = idx;

            // 由于这对点在本次迭代中已经被使用了,所以我们为了避免再次抽到这个点,就在"点的可选列表"中,
            // 将这个点原来所在的位置用vector最后一个元素的信息覆盖,并且删除尾部的元素
            // 这样就相当于将这个点的信息从"点的可用列表"中直接删除了
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }// 依次提取8个特征点对
    }// 迭代mMaxIterations次，获得mMaxIterations个最小的数据集，也就是八个点

    cout << "Check POint 2" << endl;

    // Step 3 计算fundamental 矩阵 和homography 矩阵
    // 这两个变量用于标记在H和F的计算中哪些特征点对被认为是Inlier
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;

    // HF的打分
    float SH, SF;

    // 这两个是经过RANSAC算法后计算出来的单应矩阵和基础矩阵
    cv::Mat H, F;

    // 计算F矩阵
    FindFundamental(vPts1, vPts2, vbMatchesInliersF, vSets, SF, F, succ_count, MaxIter);
    std::cout << "CheckPoint4 " << SF << endl << F << std::endl;


    //TODO: Merge DecomposeE函数,输出
}

/**
 * @brief 计算单应矩阵，假设场景为平面情况下通过前两帧求取Homography矩阵，并得到该模型的评分
 * 原理参考Multiple view geometry in computer vision  P109 算法4.4
 * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
 * Step 2 选择8个归一化之后的点对进行迭代
 * Step 3 八点法计算单应矩阵矩阵
 * Step 4 利用重投影误差为当次RANSAC的结果评分
 * Step 5 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
 * 
 * @param[in & out] vbMatchesInliers          标记是否是外点
 * @param[in & out] score                     计算单应矩阵的得分
 * @param[in & out] H21                       单应矩阵结果
 */
void FindFundamental(vector<cv::Point2f>& vPts1, vector<cv::Point2f>& vPts2, 
                    vector<bool> &vbMatchesInliers,  vector<vector<size_t> >& vSets, 
                    float &score, cv::Mat &F21, int succ_count, int MaxIter)
{
    //匹配的特征点对总数succ_count

    // Step 1 将当前帧和参考帧中的特征点坐标进行归一化，主要是平移和尺度变换
    //归一化后的参考帧1和当前帧2中的特征点坐标
    vector<cv::Point2f> vPn1, vPn2;
    // 记录各自的归一化矩阵
    cv::Mat T1, T2;
    Normalize(vPts1,vPn1, T1);
    Normalize(vPts2,vPn2, T2);

    // 注意这里取的是归一化矩阵T2的转置,因为基础矩阵的定义和单应矩阵不同，两者去归一化的计算也不相同
    cv::Mat T2t = T2.t();

    //最优结果
    score = 0.0;
    vbMatchesInliers = vector<bool>(succ_count, false);

    // 某次迭代中，参考帧的特征点坐标
    vector<cv::Point2f> vPn1i(8);
    // 某次迭代中，当前帧的特征点坐标
    vector<cv::Point2f> vPn2i(8);
    // 某次迭代中，计算的基础矩阵
    cv::Mat F21i;

    // 每次RANSAC记录的Inliers与得分
    vector<bool> vbCurrentInliers(succ_count,false);
    float currentScore;

    // 下面进行每次的RANSAC迭代
    for(int it = 0; it < MaxIter; it++)
    {
        // Step 2 选择8个归一化之后的点对进行迭代
        for(int j = 0; j < 8; j++)
        {
            // 找到特征点id
            int idx = vSets[it][j];
            // 在小的群组中固定好对应的点
            vPn1i[j] = vPn1[idx];
            vPn2i[j] = vPn2[idx];
        }// 8个随机点遍历结束

        // Step 3 八点法计算基础矩阵
        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        // 基础矩阵约束：p2^t*F21*p1 = 0，其中p1,p2 为齐次化特征点坐标    
        // 特征点归一化：vPn1 = T1 * mvKeys1, vPn2 = T2 * mvKeys2  
        // 根据基础矩阵约束得到:(T2 * mvKeys2)^t* Hn * T1 * mvKeys1 = 0   
        // 进一步得到:mvKeys2^t * T2^t * Hn * T1 * mvKeys1 = 0
        F21i = T2t*Fn*T1;

        // Step 4 利用重投影误差为当次RANSAC的结果评分
        currentScore = CheckFundamental(F21i, vbCurrentInliers, 1.0, succ_count, vPts1, vPts2);

        // Step 5 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
        if(currentScore>score)
        {
            //如果当前的结果得分更高，那么就更新最优计算结果
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            cout <<  it << '\t'<< currentScore << endl;
            score = currentScore;
        }

    }// 迭代结束

}

/**
 * @brief 归一化特征点到同一尺度，作为后续normalize DLT的输入
 *  [x' y' 1]' = T * [x y 1]' 
 *  归一化后x', y'的均值为0，sum(abs(x_i'-0))=1，sum(abs((y_i'-0))=1
 *
 *  为什么要归一化？
 *  在相似变换之后(点在不同的坐标系下),他们的单应性矩阵是不相同的
 *  如果图像存在噪声,使得点的坐标发生了变化,那么它的单应性矩阵也会发生变化
 *  我们采取的方法是将点的坐标放到同一坐标系下,并将缩放尺度也进行统一 
 *  对同一幅图像的坐标进行相同的变换,不同图像进行不同变换
 *  缩放尺度是为了让噪声对于图像的影响在一个数量级上
 * 
 *  Step 1 计算特征点X,Y坐标的均值 
 *  Step 2 计算特征点X,Y坐标离均值的平均偏离程度
 *  Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1 
 *  Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
 * 
 * @param[in] vKeys                               待归一化的特征点
 * @param[in & out] vNormalizedPoints             特征点归一化后的坐标
 * @param[in & out] T                             归一化特征点的变换矩阵
 */
void Normalize(const vector<cv::Point2f> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)                           //将特征点归一化的矩阵
{
    // 归一化的是这些点在x方向和在y方向上的一阶绝对矩（随机变量的期望）。

    // Step 1 计算特征点X,Y坐标的均值 meanX, meanY
    float meanX = 0;
    float meanY = 0;

	//获取特征点的数量
    const int N = vKeys.size();

	//设置用来存储归一后特征点的向量大小，和归一化前保持一致
    vNormalizedPoints.resize(N);

	//开始遍历所有的特征点
    for(int i=0; i<N; i++)
    {
		//分别累加特征点的X、Y坐标
        meanX += vKeys[i].x;
        meanY += vKeys[i].y;
    }

    //计算X、Y坐标的均值
    meanX = meanX/N;
    meanY = meanY/N;

    // Step 2 计算特征点X,Y坐标离均值的平均偏离程度 meanDevX, meanDevY，注意不是标准差
    float meanDevX = 0;
    float meanDevY = 0;

    // 将原始特征点减去均值坐标，使x坐标和y坐标均值分别为0
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].x - meanX;
        vNormalizedPoints[i].y = vKeys[i].y - meanY;

		//累计这些特征点偏离横纵坐标均值的程度
        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    // 求出平均到每个点上，其坐标偏离横纵坐标均值的程度；将其倒数作为一个尺度缩放因子
    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;
    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    // Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1 
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值（期望）
    for(int i=0; i<N; i++)
    {
		//对，就是简单地对特征点的坐标进行进一步的缩放
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

/**
 * @brief 根据特征点匹配求fundamental matrix（normalized 8点法）
 * 注意F矩阵有秩为2的约束，所以需要两次SVD分解
 * 
 * @param[in] vP1           参考帧中归一化后的特征点
 * @param[in] vP2           当前帧中归一化后的特征点
 * @return cv::Mat          最后计算得到的基础矩阵F
 */
cv::Mat ComputeF21(
    const vector<cv::Point2f> &vP1, //归一化后的点, in reference frame
    const vector<cv::Point2f> &vP2) //归一化后的点, in current frame
{
    // 原理详见附件推导
    // x'Fx = 0 整理可得：Af = 0
    // A = | x'x x'y x' y'x y'y y' x y 1 |, f = | f1 f2 f3 f4 f5 f6 f7 f8 f9 |
    // 通过SVD求解Af = 0，A'A最小特征值对应的特征向量即为解

	//获取参与计算的特征点对数
    const int N = vP1.size();

	//初始化A矩阵
    cv::Mat A(N,9,CV_32F); // N*9维

    // 构造矩阵A，将每个特征点添加到矩阵A中的元素
    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    //存储奇异值分解结果的变量
    cv::Mat u,w,vt;

	
    // 定义输出变量，u是左边的正交矩阵U， w为奇异矩阵，vt中的t表示是右正交矩阵V的转置
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	// 转换成基础矩阵的形式
    cv::Mat Fpre = vt.row(8).reshape(0, 3); // v的最后一列

    //基础矩阵的秩为2,而我们不敢保证计算得到的这个结果的秩为2,所以需要通过第二次奇异值分解,来强制使其秩为2
    // 对初步得来的基础矩阵进行第2次奇异值分解
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

	// 秩2约束，强制将第3个奇异值设置为0
    w.at<float>(2)=0; 
    
    // 重新组合好满足秩约束的基础矩阵，作为最终计算结果返回 
    return  u*cv::Mat::diag(w)*vt;
}

/**
 * @brief 对给定的Fundamental matrix打分
 * 
 * @param[in] F21                       当前帧和参考帧之间的基础矩阵
 * @param[in] vbMatchesInliers          匹配的特征点对属于inliers的标记
 * @param[in] sigma                     方差，默认为1
 * @return float                        返回得分
 */
float CheckFundamental(
    const cv::Mat &F21,             //当前帧和参考帧之间的基础矩阵
    vector<bool> &vbMatchesInliers, //匹配的特征点对属于inliers的标记
    float sigma, int succ_count,    //方差
    vector<cv::Point2f>& vPts1, 
    vector<cv::Point2f>& vPts2)
{

    // 说明：在已值n维观测数据误差服从N(0，sigma）的高斯分布时
    // 其误差加权最小二乘结果为  sum_error = SUM(e(i)^T * Q^(-1) * e(i))
    // 其中：e(i) = [e_x,e_y,...]^T, Q维观测数据协方差矩阵，即sigma * sigma组成的协方差矩阵
    // 误差加权最小二次结果越小，说明观测数据精度越高
    // 那么，score = SUM((th - e(i)^T * Q^(-1) * e(i)))的分数就越高
    // 算法目标：检查基础矩阵
    // 检查方式：利用对极几何原理 p2^T * F * p1 = 0
    // 假设：三维空间中的点 P 在 img1 和 img2 两图像上的投影分别为 p1 和 p2（两个为同名点）
    //   则：p2 一定存在于极线 l2 上，即 p2*l2 = 0. 而l2 = F*p1 = (a, b, c)^T
    //      所以，这里的误差项 e 为 p2 到 极线 l2 的距离，如果在直线上，则 e = 0
    //      根据点到直线的距离公式：d = (ax + by + c) / sqrt(a * a + b * b)
    //      所以，e =  (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)

    // 算法流程
    // input: 基础矩阵 F 左右视图匹配点集 mvKeys1
    //    do:
    //        for p1(i), p2(i) in mvKeys:
    //           l2 = F * p1(i)
    //           l1 = p2(i) * F
    //           error_i1 = dist_point_to_line(x2,l2)
    //           error_i2 = dist_point_to_line(x1,l1)
    //           
    //           w1 = 1 / sigma / sigma
    //           w2 = 1 / sigma / sigma
    // 
    //           if error1 < th
    //              score +=   thScore - error_i1 * w1
    //           if error2 < th
    //              score +=   thScore - error_i2 * w2
    // 
    //           if error_1i > th or error_2i > th
    //              p1(i), p2(i) are inner points
    //              vbMatchesInliers(i) = true
    //           else 
    //              p1(i), p2(i) are outliers
    //              vbMatchesInliers(i) = false
    //           end
    //        end
    //   output: score, inliers

	// 获取匹配的特征点对的总对数succ_count

	// Step 1 提取基础矩阵中的元素数据
    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

	// 预分配空间
    vbMatchesInliers.resize(succ_count);

	// 设置评分初始值（因为后面需要进行这个数值的累计）
    float score = 0;

    // 基于卡方检验计算出的阈值
	// 自由度为1的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 3.841;

    // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float thScore = 5.991;

	// 信息矩阵，或 协方差矩阵的逆矩阵
    const float invSigmaSquare = 1.0/(sigma*sigma);


    // Step 2 计算img1 和 img2 在估计 F 时的score值
    for(int i=0; i<succ_count; i++)
    {
		//默认为这对特征点是Inliers
        bool bIn = true;

	    // Step 2.1 提取参考帧和当前帧之间的特征匹配点对
        const cv::Point2d &kp1 = vPts1[i];
        const cv::Point2d &kp2 = vPts2[i];

		// 提取出特征点的坐标
        const float u1 = kp1.x;
        const float v1 = kp1.y;
        const float u2 = kp2.x;
        const float v2 = kp2.y;

        // Reprojection error in second image
        // Step 2.2 计算 img1 上的点在 img2 上投影得到的极线 l2 = F21 * p1 = (a2,b2,c2)
		const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;
    
        // Step 2.3 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num2 = a2*u2+b2*v2+c2;
        const float squareDist1 = num2*num2/(a2*a2+b2*b2);
        // 带权重误差
        const float chiSquare1 = squareDist1*invSigmaSquare;
		
        // Step 2.4 误差大于阈值就说明这个点是Outlier 
        // ? 为什么判断阈值用的 th（1自由度），计算得分用的thScore（2自由度）
        // ? 可能是为了和CheckHomography 得分统一？
        if(chiSquare1>th)
            bIn = false;
        else
            // 误差越大，得分越低
            score += thScore - chiSquare1;

        // 计算img2上的点在 img1 上投影得到的极线 l1= p2 * F21 = (a1,b1,c1)
        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        // 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num1 = a1*u1+b1*v1+c1;
        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        // 带权重误差
        const float chiSquare2 = squareDist2*invSigmaSquare;

        // 误差大于阈值就说明这个点是Outlier 
        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;
        
        // Step 2.5 保存结果
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    //  返回评分
    return score;
}



