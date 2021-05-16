// 测试sobel梯度在8bit上的表现
    // cv::Mat resultx(rgbgrey.size(),CV_8UC1);
    // cv::Mat resulty(rgbgrey.size(),CV_8UC1);
    // cv::Mat resultt(rgbgrey.size(),CV_8UC1);
    // cout << resultx.size <<endl;
    // for(int i = 0; i < rgbgrey.rows; i++)
    // for(int j = 0; j < rgbgrey.cols; j++)
    // {
    //     if(i -1 < 0 || j-1 <0 || j+1 > rgbgrey.cols || i+1 > rgbgrey.rows)
    //     {
    //         resultx.at<uchar>(i,j) = rgbgrey.at<uchar>(i,j);
    //         resulty.at<uchar>(i,j) = rgbgrey.at<uchar>(i,j);
    //         resultt.at<uchar>(i,j) = rgbgrey.at<uchar>(i,j);
    //     }
    //     else
    //     {
    //         Eigen::Vector2d grad = PointGradSobel8(rgbgrey,j,i);
    //         resultx.at<uchar>(i,j) = (float)(grad[0]);
    //         resulty.at<uchar>(i,j) = (float)(grad[1]);
    //         resultt.at<uchar>(i,j) = (float)sqrt(pow((float)(grad[1]),2)+pow((float)(grad[0]),2));
    //     }
    // }
    // cv::Mat resultxout(rgbgrey.size(),CV_8UC1);
    // cv::Mat resultyout(rgbgrey.size(),CV_8UC1);
    // cv::Mat resulttout(rgbgrey.size(),CV_8UC1);
    // cv::normalize(resultx, resultxout, 0, 255, cv::NORM_MINMAX);
    // cv::normalize(resulty, resultyout, 0, 255, cv::NORM_MINMAX);
    // cv::normalize(resultt, resulttout, 0, 255, cv::NORM_MINMAX);
    // cv::imshow("8bit y out", resultyout);
    // cv::imshow("8bit x out", resultxout);
    // cv::imshow("8bit out", resulttout);
    // cv::waitKey();

    // 测试sobel梯度在16bit上的表现
    // cv::Mat resultx16(thermal16.size(),CV_16UC1);
    // cv::Mat resulty16(thermal16.size(),CV_16UC1);
    // cv::Mat resultt16(thermal16.size(),CV_16UC1);
    
    // cout << resultx16.size <<endl;
    // double maxdx,maxdy;
    // maxdx = 0;
    // maxdy = 0;
    // double mindx = 65500;
    // double mindy = 65500;
    // for(int i = 0; i < thermal16.rows; i++)
    // for(int j = 0; j < thermal16.cols; j++)
    // {
    //     // 边沿的像素点计算不了,移除
    //     if(i -1 < 0 || j-1 <0 || j+1 > thermal16.cols || i+1 > thermal16.rows)
    //     {
    //         resultx16.at<ushort>(i,j) = thermal16.at<ushort>(i,j);
    //         resulty16.at<ushort>(i,j) = thermal16.at<ushort>(i,j);
    //         resultt16.at<ushort>(i,j) = thermal16.at<ushort>(i,j);
    //     }
    //     else
    //     {
    //         Eigen::Vector2d grad = PointGradSobel16(thermal16,j,i);
    //         resultx16.at<ushort>(i,j) = (double)(grad[0]);
    //         if(grad[0] <= 2412 && grad[0] >=500) cout << "CATCH YOU ! " << j << '\t' <<i <<endl;
    //         if(grad[0] <= 2412 && grad[0] >=500) continue;
    //         if(grad[0]>maxdx) maxdx = grad[0];
    //         if(grad[0]<mindx) mindx = grad[0];
    //         resulty16.at<ushort>(i,j) = (double)(grad[1]);
    //         if(grad[1] <= 5089 && grad[1] >=500) cout << "CATCH YOU Y ! " << j << '\t' <<i <<endl;
    //         if(grad[1] <= 5089 && grad[1] >=500) continue;
    //         if(grad[1]>maxdy) maxdy = grad[1];
    //         if(grad[1]<mindy) mindy = grad[1];
    //         resultt16.at<ushort>(i,j) = (double)sqrt(pow((double)(grad[1]),2)+pow((double)(grad[0]),2));
    //     }
    // }
    // cout << "GOOOOOOOOOOOD" << maxdy << '\t' <<mindy<<endl;
    
    // cv::Mat resultx16out(thermal16.size(),CV_8UC1);
    // cv::Mat resulty16out(thermal16.size(),CV_8UC1);
    // cv::Mat resultt16out(thermal16.size(),CV_8UC1);
    // for(int i = 0; i < thermal16.rows; i++)
    // for(int j = 0; j < thermal16.cols; j++)
    // {
    //     resultx16out.at<uchar>(i,j) = ((double)resultx16.at<ushort>(i,j) / (double)(maxdx - mindx)) * 255;
    //     resulty16out.at<uchar>(i,j) = ((double)resulty16.at<ushort>(i,j) / (double)(maxdy - mindy)) * 255;
    // }
    // for(int i = 0; i < thermal16.rows; i++)
    // for(int j = 0; j < thermal16.cols; j++)
    // {
    //     uchar value = sqrt(pow(resultx16out.at<uchar>(i,j),2)+pow(resulty16out.at<uchar>(i,j),2));
    //     if(value > 255) value = 255;
    //     else{
    //         resultt16out.at<uchar>(i,j) = value;
    //     }
    // }
    // cv::imshow("8bit y out", resulty16out);
    // cv::imshow("8bit x out", resultx16out);
    // cv::imshow("8bit out", resultt16out);
    // cv::waitKey();

     //测试sobel梯度在16bit上的表现
    // cv::Mat resultx16(thermal16.size(),CV_16UC1);
    // cv::Mat resulty16(thermal16.size(),CV_16UC1);
    // cv::Mat resultt16(thermal16.size(),CV_16UC1);
    // cv::Mat test = GetGradImage(thermal16);
    // cv::Mat test3 = GetGradImage8(rgbgrey);
    // Eigen::Vector2d resui = GetMaxAndMin(test);
    // cout << "The max grad in 16 is " << resui[0] << " min " << resui[1] <<endl;
    // cv::Mat test2 = Swap16To8(test);
    // Eigen::Vector2d resui2 = GetMaxAndMin(test3);
    // cout << "The max grad in 8 is " << resui2[0] << " min " << resui2[1] <<endl;

    // cv::imshow("testttttttt", test2);
    // cv::imshow("testttttttt88888888", test3);

    // cv::waitKey();
    // cout << resultx16.size <<endl;
    // double maxdx,maxdy;
    // maxdx = 0;
    // maxdy = 0;
    // double mindx = 65500;
    // double mindy = 65500;
    // for(int i = 0; i < thermal16.rows; i++)
    // for(int j = 0; j < thermal16.cols; j++)
    // {
    //     // 边沿的像素点计算不了,移除
    //     // 因为实际自己的数据集存在缺陷,所以在判断条件的过程中间需要考虑到这些
    //     // 将错误的点进行去除
    //     if(i -1 < 0 || j-1 <0 || j+1 > thermal16.cols || i+1 > thermal16.rows -1 || (i == 478 && j == 639))
    //     {
    //         resultx16.at<ushort>(i,j) = 0;
    //         resulty16.at<ushort>(i,j) = 0;
    //         resultt16.at<ushort>(i,j) = 0;
    //     }
    //     else
    //     {
    //         Eigen::Vector2d grad = PointGradSobel(thermal16,j,i);
    //         resultx16.at<ushort>(i,j) = (double)(grad[0]);
    //         if(grad[0]>maxdx) maxdx = grad[0];
    //         if(grad[0]<mindx) mindx = grad[0];
    //         resulty16.at<ushort>(i,j) = (double)(grad[1]);
    //         if(grad[1]>maxdy) maxdy = grad[1];
    //         if(grad[1]<mindy) mindy = grad[1];
    //         resultt16.at<ushort>(i,j) = (double)sqrt(pow((double)(grad[1]),2)+pow((double)(grad[0]),2));
    //     }
    // }
    // cout << "GOOOOOOOOOOOD x " << maxdx << '\t' <<mindx<<endl;
    // cout << "GOOOOOOOOOOOD y " << maxdy << '\t' <<mindy<<endl;
    
    // cv::Mat resultx16out(thermal16.size(),CV_8UC1);
    // cv::Mat resulty16out(thermal16.size(),CV_8UC1);
    // cv::Mat resultt16out(thermal16.size(),CV_8UC1);
    // for(int i = 0; i < thermal16.rows; i++)
    // for(int j = 0; j < thermal16.cols; j++)
    // {
    //     resultx16out.at<uchar>(i,j) = ((double)resultx16.at<ushort>(i,j) / (double)(maxdx - mindx)) * 255;
    //     resulty16out.at<uchar>(i,j) = ((double)resulty16.at<ushort>(i,j) / (double)(maxdy - mindy)) * 255;
    // }
    // for(int i = 0; i < thermal16.rows; i++)
    // for(int j = 0; j < thermal16.cols; j++)
    // {
    //     uchar value = sqrt(pow(resultx16out.at<uchar>(i,j),2)+pow(resulty16out.at<uchar>(i,j),2));
    //     if(value > 255) value = 255;
    //     else{
    //         resultt16out.at<uchar>(i,j) = value;
    //     }
    // }
    // cv::imshow("8bit y out", resulty16out);
    // cv::imshow("8bit x out", resultx16out);
    // cv::imshow("8bit out", resultt16out);
    // // cv::imwrite("../results/sobel16bitthermal3.png", resultt16out);
    // cv::waitKey();