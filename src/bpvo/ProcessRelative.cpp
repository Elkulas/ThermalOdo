/* 
 * Contributor: elkulasjiang@zju.edu.cn
 */

#include "ProcessRelative.h"

#include <opencv2/imgproc/imgproc.hpp>

// constructor
BitPlaneDescriptor::BitPlaneDescriptor(float s0, float s1)
  : rows_(0), cols_(0), sigma_ct_(s0), sigma_bp_(s1) {}

// destructor
BitPlaneDescriptor::~BitPlaneDescriptor() {}

// compute
void BitPlaneDescriptor::compute(cv::Mat& I) {
  rows_ = I.rows;
  cols_ = I.cols;
  // Step 1: Get LBP Image
  cv::GaussianBlur(I, I, cv::Size(3,3), sigma_ct_, sigma_ct_);
  getOriginLBPFeature(I, lbp_);

  cv::imshow("lbp", lbp_);

  cv::waitKey();

  // DFT test
  cv::Mat src_proc;
  I.convertTo(src_proc, CV_64FC1);
  cv::Mat spectrum(I.size(), CV_64FC2);

  cv::dft(src_proc, spectrum, cv::DFT_COMPLEX_OUTPUT, 0);
  // shift
  dftShift(spectrum);
 
  showSpectrum(spectrum, "origin dft", true);

  // for(int i = 174; i < 179; i++){
  //   for(int j = 238; j < 242; j++){
  //   notchFilter(spectrum, i, j, 0);
  //   }
  // }

  cv::Mat temp[2];
  cv::split(spectrum, temp);
  for(int i = 174; i < 179; i++){
    for(int j = 238; j < 242; j++){
    notchFilter(temp[0], i, j, 0);
    notchFilter(temp[1], i, j, 0);
    }
  }
  cv::Mat filtered;
  cv::merge(temp, 2, filtered);
  showSpectrum(filtered, "filtered dft", false);
  
  cout << spectrum.type()<<endl;

  showSpectrum(spectrum, "filtered dft", false);


  // BHPF
  {
    cv::Mat bhpf_filter;
    makeBHPF(spectrum.cols, spectrum.rows, 30, 2, bhpf_filter);
    cv::Mat complex[2];
    cv::Mat temp[2];
    cv::split(spectrum, complex);
    cv::multiply(complex[0], bhpf_filter, temp[0], 1.0, 6);
    cv::multiply(complex[1], bhpf_filter, temp[1], 1.0, 6);
    cv::Mat idft, iout;
    cv::merge(temp, 2, idft);
    showSpectrum(idft, "bhpf dft", false);
    cv::dft(idft, iout, cv::DFT_INVERSE);
    showSpectrum(iout, "bhpf", true);
    dftShift(idft);
    cv::dft(idft, iout, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
    cv::imshow("bhpf2", iout);
    cv::waitKey();
  }

  // BIPF
  {
    cv::Mat bipf_filter;
    makeBIPF(spectrum.cols, spectrum.rows, 60, 20, bipf_filter);
    cv::Mat complex[2];
    cv::Mat temp[2];
    cv::split(spectrum, complex);
    cv::multiply(complex[0], bipf_filter, temp[0], 1.0, 6);
    cv::multiply(complex[1], bipf_filter, temp[1], 1.0, 6);
    cv::Mat idft, iout;
    cv::merge(temp, 2, idft);
    showSpectrum(idft, "bipf dft", false);
    cv::dft(idft, iout, cv::DFT_INVERSE);
    showSpectrum(iout, "bipf", true);
    dftShift(idft);
    cv::dft(idft, iout, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
    cv::imshow("bipf2", iout);
    cv::waitKey();
  }

  // ftDFT test
  cv::Mat dtft_mat;
  cv::dft(spectrum, dtft_mat, cv::DFT_INVERSE);
  
  //dtfft
  showSpectrum(dtft_mat, "origin dft2", false);

  cv::waitKey();
  // Step 2: Get 8 channel images

  cv::Mat out0;
  ExtractChannel(lbp_, out0, 0, sigma_bp_);
  std::cout << "Num: 0" << std::endl;
  cv::imshow("lbp0", out0);
  cv::waitKey();

}

void BitPlaneDescriptor::ExtractChannel(const cv::Mat& src, cv::Mat& dst, int bit, float sigma) {
  dst.create(src.size(), cv::DataType<float>::type);
  float scale = float(1.0);
  float bias = float(0.0);

  auto src_ptr = src.ptr<const uint8_t>();
  auto dst_ptr = dst.ptr<float>();

  auto n = rows_*cols_;
  std::cout << "N " << n << std::endl;

  // traverse
  for(int i = 0; i < n; ++i) {
    dst_ptr[i] = scale * ((src_ptr[i] & (1 << bit)) >> bit) - bias;
  }

  // if(sigma > 0.0f) {
  //   cv::GaussianBlur(dst, dst, cv::Size(5,5), sigma, sigma);
  // }
}

void BitPlaneDescriptor::dftShift(cv::Mat& img){
  int cx = img.cols / 2;
  int cy = img.rows / 2;
  cv::Mat q0 = img(cv::Rect(0, 0, cx, cy));
  cv::Mat q1 = img(cv::Rect(cx, 0, cx, cy));
  cv::Mat q2 = img(cv::Rect(0, cy, cx, cy));
  cv::Mat q3 = img(cv::Rect(cx, cy, cx, cy));
  cv::Mat tmp;
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);
  q1.copyTo(tmp);
  q2.copyTo(q1);
  tmp.copyTo(q2);
}

void BitPlaneDescriptor::showSpectrum(cv::Mat& spectrum, std::string title, bool inverse){
  cv::Mat out_mag;
  // split channel
  cv::Mat mag_channel[2];
  cv::split(spectrum, mag_channel);
  cv::magnitude(mag_channel[0], mag_channel[1], out_mag);
  // inverse
  if(!inverse){
    out_mag += cv::Scalar::all(1);
    cv::log(out_mag, out_mag);
  }
  cv::normalize(out_mag, out_mag, 0, 255, cv::NORM_MINMAX);
  out_mag.convertTo(out_mag, CV_8UC1);
  cv::imshow(title, out_mag);
  std::string outpath = "/home/jjj/NGCLAB/ThermalOdo/bin/reslut/";
  std::string output = "";
  output = outpath + "/" + title + ".png";
  cv::imwrite(output, out_mag);
  cv::waitKey();
  // cv::Mat lbp;
  // getOriginLBPFeature(out_mag, lbp);
  // cv::imshow(title, lbp);
  // cv::waitKey();
}

void BitPlaneDescriptor::makeBHPF(int width, int height, double D0, double n, cv::Mat& dst){
  dst = cv::Mat::ones(Size(width, height), CV_32F);
	double D0_2 = pow(D0, 2);
	double half_h = height * (1.0) / 2;
	double half_w = width * (1.0) / 2;
	//创建理想高通滤波器
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			double distance = pow((half_h - (double)i), 2) + pow((half_w - (double)j), 2);
			double temp = pow(D0 / sqrt(distance), 2 * n);
			dst.at<float>(i, j) = 1.0 / (1.0 + temp);
		}
	}
}

void BitPlaneDescriptor::makeBIPF(int width, int height, double D0, double n, cv::Mat& dst){
  dst = cv::Mat::ones(Size(width, height), CV_32F);
	double D0_2 = pow(D0, 2);
	double half_h = height * (1.0) / 2;
	double half_w = width * (1.0) / 2;
	//布特沃斯低通滤波器
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			double distance = pow((half_h - (double)i), 2) + pow((half_w - (double)j), 2);
			double temp = pow(sqrt(distance)/D0, 2 * n);
			dst.at<float>(i, j) = 1.0 / (1.0 + temp);
		}
	}
}

void BitPlaneDescriptor::notchFilter(cv::Mat& spectrum, int x, int y, int value){
  cv::Vec2d* pf = spectrum.ptr<cv::Vec2d>(y);
  cout << "Cout! " << pf[x][0] << endl;
  pf[x][0] = value;
}



