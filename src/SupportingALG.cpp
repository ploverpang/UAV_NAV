/*
*Supporting algorithms for main depthProcessing node
*
*/
#include <DepthGeneration.hpp>

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

cv::Mat maskOutliers(cv::Mat src_img, cv::Mat prevFrame, std::list<cv::Mat> &maskBuffer, int nFrames, int diffThreshold){
    cv::Mat mask = Mat(src_img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat dst_img = cv::Mat::zeros(src_img.size(), CV_8UC1);
    cv::Mat maskSum = Mat(src_img.size(), CV_32FC1, Scalar(0));

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
    imshow("Thresholded mask", maskSum);
    waitKey(1);

    src_img.copyTo(dst_img,maskSum);

    return dst_img;
}
