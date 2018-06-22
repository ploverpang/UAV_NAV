#include "uav_nav/depth_generation.h"

using namespace cv;

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber) {
  float average = 0;
  if (numberOfArrayElements == 0){
    return average;
  }
  else if (stopNumber == numberOfArrayElements){ //optimization in case all values are taken into account
    for (std::vector<int>::iterator it=arr.begin(); it!=arr.end(); ++it){
      average += *it;
    }
  }
  else{
    std::priority_queue<int, std::vector<int>, std::greater<int> >pq;
    for (int i = 0; i < numberOfArrayElements; i++){
      pq.push(arr[i]);
    }
    for (int i = 0; i < stopNumber; i++){
      average += pq.top();
      //arr[i]=pq.top();
      pq.pop();
    }
  }
  return average/numberOfArrayElements;
}

void show_histogram(std::string const& name, cv::Mat1b const& image){
  // Set histogram bins count
  int bins = 256;
  int histSize[] = {bins};
  // Set ranges for histogram bins
  float lranges[] = {0, 256};
  const float* ranges[] = {lranges};
  // create matrix for histogram
  cv::Mat hist;
  int channels[] = {0};

  // create matrix for histogram visualization
  int const hist_height = 256;
  cv::Mat3b hist_image = cv::Mat3b::zeros(hist_height, bins);

  cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);

  double max_val=0;
  minMaxLoc(hist, 0, &max_val);

  // visualize each bin
  for(int b = 0; b < bins; b++) {
    float const binVal = hist.at<float>(b);
    int   const height = cvRound(binVal*hist_height/max_val);
    cv::line
    ( hist_image
      , cv::Point(b, hist_height-height), cv::Point(b, hist_height)
      , cv::Scalar::all(255)
      );
  }
  cv::imshow(name, hist_image);
}

void maskOutliers(const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& prevFrame, const bool& clearCMD, const int nFrames, const float diffThreshold){
  static std::list<cv::Mat> maskBuffer;  // Container for stored masks;
  // If previous ID doesn't match current ID clear buffer and return
  if (clearCMD){
    maskBuffer.clear();
    dst_img = src_img;
    return;
  }
  if (nFrames == 0){
    dst_img = src_img;
    return;
  }

  cv::Mat mask = Mat(src_img.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat zeros = cv::Mat::zeros(src_img.size(), src_img.type());
  cv::Mat maskSum = Mat(src_img.size(), CV_32FC1, Scalar(0));

  for (int x_pix=0; x_pix<src_img.cols; x_pix++){
    for (int y_pix=0; y_pix<src_img.rows; y_pix++){
      if (abs(src_img.at<float>(y_pix, x_pix) - prevFrame.at<float>(y_pix, x_pix)) > diffThreshold){
        mask.at<unsigned char>(y_pix, x_pix) = 255;
      }
    }
  }

  maskBuffer.push_front(mask);
  if (maskBuffer.size() == nFrames+1){
    maskBuffer.pop_back();
  }

  for(std::list<cv::Mat>::iterator it = maskBuffer.begin(); it != maskBuffer.end(); it++){
    cv::Mat buf = *it;
    cv::accumulate(buf, maskSum);
  }
  maskSum.convertTo(maskSum, CV_8UC1);
  cv::threshold(maskSum, maskSum, 1, 255, 1);
  src_img.copyTo(zeros,maskSum);
  zeros.copyTo(dst_img);

  return;
}


void legacyRoundMorph(cv::Mat& src_img, int byNumber, int xy){
  src_img /= byNumber;
  src_img *= byNumber;

  // Morphology
  cv::Mat element = getStructuringElement(MORPH_RECT, Size(xy, xy));
  morphologyEx(src_img, src_img, MORPH_OPEN, element);
  morphologyEx(src_img, src_img, MORPH_CLOSE, element);
  //imshow("after morphology", src_img);
  return;
}

void roundMorph(cv::Mat& src_img, cv::Mat& dst_img, int xy, int threshold){
  if (xy<3 || xy%2 != 1) return;
  int offset = (xy-1)/2;
  cv::Mat mask(src_img.size(), CV_8UC1, cv::Scalar(255));
  for (int i = 0; i < src_img.rows; ++i){
    for (int j = 0; j < src_img.cols; ++j){
      bool marked = 0;
      for (int k = i - offset; k <= i + offset; k++){
        if (k >= 0 && k < src_img.rows){
          for (int l = j - offset; l <= j + offset; l++){
            if (l >= 0 && l < src_img.cols){
              if (abs(src_img.at<unsigned char>(i,j) - src_img.at<unsigned char>(k,l)) > threshold){
                if (src_img.at<unsigned char>(i,j) != 0 && src_img.at<unsigned char>(k,l) != 0){
                marked = 1;}
              }
            }
          }
        }
      }
      if (marked == 1){
        mask.at<unsigned char>(i,j) = 0; // may throw runtime error. not tested
      }
    }
  }

  cv::Mat zeros = cv::Mat::zeros(src_img.size(), src_img.type());
  src_img.copyTo(zeros, mask);
  cv::Mat element = getStructuringElement(MORPH_RECT, Size(xy, xy));
  morphologyEx(zeros, dst_img, MORPH_CLOSE, element);
  //imshow("Symmetric rounded morph mask", mask);
  return;
}

void dispToMeter(cv::Mat src_img, cv::Mat& dst_img){
  src_img.convertTo(src_img, CV_32FC1);
  src_img *= 16.0*3.6; // fractional bits of StereoSGBM disparity maps
  src_img = (247.35*150)/src_img; // disparity map values to 'm'
  src_img.copyTo(dst_img);
  return;
}

void fovReduction(float alt, cv::Mat src_img, cv::Mat& dst_img){
  const double FOV_y = CAMERAFOV_Y*M_PI/180;
  double clearance = CLEARANCE;
  if(alt > 1){
    clearance = alt >= 2.5 ? 2.0 : alt-0.5;
  }
  else{
    clearance = 0.5;
  }
  double newAngle = atan2(clearance, CAMERARANGE*1.5); //1.5 might be useful
  if (newAngle>FOV_y/2) newAngle = FOV_y/2;
  int cutoffPixel = (src_img.rows-(round(double(src_img.rows)*(newAngle*2)/FOV_y)))/2;
  cv::Rect crop(0, cutoffPixel, src_img.cols, src_img.rows-2*cutoffPixel);
  cv::Mat cropped = src_img(crop);
  dst_img = cropped.clone();
  return;
}


void maskGround(float alt, cv::Mat src_img, cv::Mat& dst_img){
  double clearance = CLEARANCE;
  if(alt > 1){
    clearance = alt >= 2.5 ? (float)CLEARANCE : alt-0.5;
  }
  else{
    clearance = 0.5;
  }
  double newAngle = atan2(clearance, CAMERARANGE*1.5); //1.5 might be useful
  const double FOV_y = atan2(CLEARANCE, CAMERARANGE*1.5); //1.5 might be useful
  if (newAngle>FOV_y) newAngle = FOV_y;
  int cutoffPixel = (src_img.rows-(round(double(src_img.rows)*newAngle/FOV_y)))/2;
  cv::Rect crop(0, 0, src_img.cols, src_img.rows-cutoffPixel);
  cv::Mat mask = cv::Mat::zeros(src_img.size(), src_img.type());
  mask(crop) = 255;
  cv::Mat zeros = cv::Mat::zeros(src_img.size(), src_img.type());
  src_img.copyTo(zeros, mask);
  zeros.copyTo(dst_img);
  return;
}