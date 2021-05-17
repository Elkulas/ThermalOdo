
#include "ReadData.h"

void GetImageData(vector<pair<double, cv::Mat>>& imgDataset, string& sData_path, string& sConfig_path)
{
    string sImage_file = sConfig_path + "times.txt";

    cout << "1 GetImageData start sImage_file: " << sImage_file << endl;

    // 读取文件名的文件
    ifstream fsImage;

    fsImage.open(sImage_file.c_str());

    if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

    // 读取每一行的内容
    std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

    while(getline(fsImage, sImage_file) && !sImage_line.empty())
    {
        // 解析得到image的文件名
        istringstream ssImgData(sImage_line);
        ssImgData >> dStampNSec >> sImgFileName;
		cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;

        string imagePath = sData_path + sImgFileName;

		cv::Mat img = cv::imread(imagePath.c_str(), 0);

        if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}

        // 将img保存,将时间戳保存
        imgDataset.push_back(make_pair(dStampNSec, img));
    }

    fsImage.close();

}

void GetImageDataWithNum(vector<pair<double, cv::Mat>>& imgDataset, 
                         string& sData_path, string& sConfig_path, int num)
{
    string sImage_file = sConfig_path + "times.txt";

    cout << "1 GetImageData start sImage_file: " << sImage_file << endl;

    // 读取文件名的文件
    ifstream fsImage;

    fsImage.open(sImage_file.c_str());

    if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

    // 读取每一行的内容
    std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

    int count = 0;

    while(getline(fsImage, sImage_line) && !sImage_line.empty())
    {
        if(count == num) break;

        // 解析得到image的文件名
        istringstream ssImgData(sImage_line);
        ssImgData >> dStampNSec >> sImgFileName;
		cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;

        string imagePath = sData_path + sImgFileName;

		cv::Mat img = cv::imread(imagePath.c_str(), 0);

        if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}

        // 将img保存,将时间戳保存
        imgDataset.push_back(make_pair(dStampNSec, img));

        count++;
    }

    fsImage.close();

}