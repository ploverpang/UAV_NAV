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

  cv::Mat maskOutliers(cv::Mat src_img, cv::Mat prevFrame, int nFrames, int diffThreshold){
    cv::Mat mask = Mat(src_img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat dst_img = cv::Mat::zeros(src_img.size(), CV_8UC1);
    cv::Mat maskSum = Mat(src_img.size(), CV_32FC1, Scalar(0));
    static std::list<cv::Mat> maskBuffer;  // Container for stored masks;

    for (int x_pix=0; x_pix<src_img.cols; x_pix++){
      for (int y_pix=0; y_pix<src_img.rows; y_pix++){
        if (abs(src_img.at<unsigned char>(y_pix, x_pix) - prevFrame.at<unsigned char>(y_pix, x_pix)) > diffThreshold){
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
    cv::threshold(maskSum, maskSum, 1, 255, 1);
    maskSum.convertTo(maskSum, CV_8UC1);
    //imshow("Thresholded mask", maskSum);
    waitKey(1);

    src_img.copyTo(dst_img,maskSum);

    return dst_img;
  }


  cv::Mat legacyRoundMorph(Mat src_img, int byNumber, int xy){
    src_img /= byNumber;
    src_img *= byNumber;

    // Morphology
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(xy, xy));
    morphologyEx(src_img, src_img, MORPH_OPEN, element);
    morphologyEx(src_img, src_img, MORPH_CLOSE, element);
    //imshow("after morphology", src_img);
    return src_img;
  }

  cv::Mat roundMorph(cv::Mat src_img, int offset, int threshold){
    cv::Mat mask(src_img.size(), CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < src_img.rows; ++i){
      for (int j = 0; j < src_img.cols; ++j){
        bool marked = 0;
        for (int k = i - offset; k <= i + offset; k++){
          if (k >= 0 && k < src_img.rows){
            for (int l = j -offset; l <= i + offset; l++){
              if (l >= 0 && l < src_img.rows && (k != i || l != j)){
                if (abs(src_img.at<unsigned char>(j,i) - src_img.at<unsigned char>(l,k)) > threshold){
                  marked = 1;
                }
              }
            }
          }
        }
        if (marked == 1){
          mask.at<unsigned char>(j,i) = 0;
        }
      }
    }
    cv::Mat output(src_img.size(), src_img.type(), cv::Scalar(0));
    src_img.copyTo(output, mask);
    return output;
  }

  cv::Mat dispToMeter(Mat distMap){
    distMap.convertTo(distMap, CV_16UC1);
    distMap *= 16.0*3.6; // fractional bits of StereoSGBM disparity maps
    distMap = (247.35*150)/distMap; // disparity map values to 'm'
    //distMap.convertTo(distMap, CV_8UC1);
    return distMap;
  }

  cv::Mat fovReduction(cv::Mat src_img){
    static double newAngle = atan2(CLEARANCE, CAMERARANGE);
    const double FOV_y = 45*M_PI/180;
    static int cutoffPixel = (src_img.rows-(round(double(src_img.rows)*(FOV_y-newAngle*2)/FOV_y)))/2;

    static cv::Rect crop(0, cutoffPixel, src_img.cols, src_img.rows-2*cutoffPixel);
    cv::Mat cropped = src_img(crop);

    return cropped;
  }
