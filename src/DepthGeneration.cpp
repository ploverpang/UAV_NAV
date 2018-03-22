/*
 * depthProcessing.cpp
 *
 *  Created on: Dec, 1 2017
 *      Author: ghostfromthebottle
 *
 * uses: supportingALG.cpp for findsmallestX, show_histogram
 */

#include <DepthGeneration.hpp>

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Publisher laser_scan_pub;

cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat imgDisparity16S(HEIGHT, WIDTH, CV_16UC1);
cv::Mat frameBuffer;
std::list<cv::Mat> maskList;  // Container for stored masks;


// For Stereo BM
const int numDisp = 64;
const int blockSize = 9;
cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(numDisp,blockSize);
cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(sbm);
cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(sbm);

// For StereoSGBM
const int wsize =5;
cv::Ptr<cv::StereoSGBM> sgbm  = cv::StereoSGBM::create(0,numDisp,wsize);
cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_sgbm = cv::ximgproc::createDisparityWLSFilter(sgbm);
cv::Ptr<cv::StereoMatcher> right_matcher_sgbm = cv::ximgproc::createRightMatcher(sgbm);

//Other variables
int imgFlag = 0;
double scale16_8 = 0.00390625;
std::string left_id, right_id;


/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  left_id = left_img->header.frame_id;
  cv_ptr->image.convertTo(left_img1, CV_8UC1);
  imgFlag++;
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  right_id = right_img->header.frame_id;
  cv_ptr->image.convertTo(right_img1, CV_8UC1);
  imgFlag++;
}

cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img){

  if( L_img.empty() || R_img.empty() ){
  }
  else
  {
    cv::Mat left_disp, right_disp, filtered_disp, left_sgbm, right_sgbm, filtered_sgbm;

    // Stereo camera to depth image
    sbm->compute( L_img, R_img, left_disp );
    right_matcher->compute( R_img, L_img, right_disp);
    wls_filter->setLambda(8000);
  wls_filter->setSigmaColor(1.5); //0.8
  wls_filter->filter(left_disp,left_img1,filtered_disp,right_disp);
  filtered_disp = (247.35 * 150)/filtered_disp; //disparity map values to 'm' for 8bit grayscale img (approximation)

  // StereoSGBM
  sgbm->setP1(8*wsize*wsize);
  sgbm->setP2(32*wsize*wsize);
  sgbm->setPreFilterCap(21);
  sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
  sgbm->compute( L_img, R_img, left_sgbm);
  right_matcher_sgbm->compute(R_img, L_img, right_sgbm);
  wls_filter_sgbm->setLambda(8000);
  wls_filter_sgbm->setSigmaColor(1.5); //0.8
  wls_filter_sgbm->filter(left_sgbm,left_img1,filtered_sgbm,right_sgbm);
  filtered_sgbm = (247.35*150)/filtered_sgbm;
  //left_sgbm /= 12.8; //Devide by 128 to get meter values

    /*// Morphology
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(9, 9));
    morphologyEx(left_sgbm, left_sgbm, MORPH_OPEN, element);
    morphologyEx(left_sgbm, left_sgbm, MORPH_CLOSE, element);
    //imshow("after morphology", out_img);*/


    //Masking unused borders
  int x_delta = (blockSize-1)/2;
  int y_delta = (blockSize-1)/2;
  int croppedWIDTH = WIDTH-numDisp-2*(x_delta);
  int croppedHEIGHT = HEIGHT-2*(y_delta);

  cv::Rect mask(numDisp + x_delta, y_delta, croppedWIDTH, croppedHEIGHT);
  filtered_disp = filtered_disp(mask);
  filtered_disp.convertTo(filtered_disp, CV_8UC1);
  left_sgbm = left_sgbm(mask);
  left_sgbm.convertTo(left_sgbm, CV_8UC1);
  imshow("SGBM", left_sgbm);
  filtered_sgbm = filtered_sgbm(mask);
  filtered_sgbm.convertTo(filtered_sgbm, CV_8UC1, scale16_8);
    //imshow("filtered sgbm", filtered_sgbm);


  if (frameBuffer.empty() == false){
    cv::Mat debug_mask = maskOutliers(left_sgbm, frameBuffer, maskList, 10, 10);
    imshow("mask Outliers", debug_mask);
  }
  left_sgbm.copyTo(frameBuffer);

  cv::Mat out_img = filtered_disp;
    //cv::Mat processedDepth = DepthProcessing(croppedHEIGHT, croppedWIDTH, out_img);

  return out_img;
}
}

cv::Mat DepthProcessing(int croppedHEIGHT, int croppedWIDTH, cv::Mat src_img){

    // X and Y component of the field of view of the depth image
  float FOV_x = 45;
  float FOV_y = 45;
    float slice_x = 5; // width of each slice in degrees
    float slice_y = 45;
    float percentMin = 100 /float(100); // The bottom X percent of the grid values.


    int numSlices_x = floor(float(FOV_x/slice_x));
    int numSlices_y = floor(float(FOV_y/slice_y));
    int slicePixelWIDTH = floor(float(croppedWIDTH/numSlices_x));
    int slicePixelHEIGHT = floor(float(croppedHEIGHT/numSlices_y));
    int pushByPixelAmmount_x = floor((croppedWIDTH-(slicePixelWIDTH*numSlices_x))/2);
    int pushByPixelAmmount_y = floor((croppedHEIGHT-(slicePixelHEIGHT*numSlices_y))/2);

    // Multi dimensional array containing the average values from each slice/square of the depth image.
    float depthGidValues[numSlices_x][numSlices_y][1] = {};
    // Process non NULL values
    for (int x_grid=0; x_grid<numSlices_x; x_grid++){
      for (int y_grid=0; y_grid<numSlices_y; y_grid++){
        std::vector<int> sorted1D;
        for (int x_pix=0; x_pix<slicePixelWIDTH; x_pix++){
          for (int y_pix=0; y_pix<slicePixelHEIGHT; y_pix++){
            if (src_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix) != 0){
              sorted1D.push_back(src_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix));
            }
          }
        }

        int numberOfArrayElements = sorted1D.size();
        int stopNumber = numberOfArrayElements*percentMin;
        float average;

            //ROS_INFO("Start sorting");
        average = findsmallestX(sorted1D, numberOfArrayElements, stopNumber);
            //ROS_INFO("End sorting");
            //printf("\n\n");

            //ROS_INFO("The average of the bottom %f percent of the grid %ix,%iy is %f\n", percentMin*100, x_grid, y_grid, average);
        depthGidValues[x_grid][y_grid][0] = average;
            //ROS_INFO("Check for good copy x: %i, y: %i, average: %f\n", x_grid, y_grid, average);

      }
    }

    sensor_msgs::LaserScan scans;
    scans.header.frame_id = left_id;
    scans.header.stamp = ros::Time::now();
    scans.angle_min = (-FOV_x/2) * (M_PI/180);//((slice_x-FOV_x)/2) * (M_PI/180);
    scans.angle_max = (FOV_x)/2 * (M_PI/180);//(FOV_x-slice_x)/2 * (M_PI/180);
    scans.angle_increment = slice_x * (M_PI/180);
    scans.time_increment = 1/20/((FOV_x/slice_x)+1);
    scans.range_min = 0; //should be in 'm' but right now it's in 'pixel depth'
    scans.range_max= 1000;
    scans.ranges.resize(numSlices_x);
    for (int i=0; i < numSlices_x; i++){
      scans.ranges[i] = depthGidValues[i][0][0];
    }
    laser_scan_pub.publish(scans);


    //pic showing the output of this method
    cv::Mat debug(slicePixelHEIGHT*numSlices_y, slicePixelWIDTH*numSlices_x, CV_8UC1);
    for (int i=0; i<numSlices_x;i++){
      for (int j =0; j<numSlices_y; j++){
        //printf("x:%i, y:%i, %f, \n", i, j, depthGidValues[i][j][0]);
        for (int ii = 0; ii < slicePixelWIDTH; ii++){
          for (int jj = 0; jj < slicePixelHEIGHT; jj++){
            debug.at<unsigned char>(j*slicePixelHEIGHT+jj, i*slicePixelWIDTH+ii) = depthGidValues[i][j][0];
          }
        }
      }
    }
    cv:imshow("Debug pic", debug);
    return debug;
  }

  int main(int argc, char** argv) {
    ros::init(argc, argv, "DepthGeneration");
    ros::NodeHandle nh;

    left_image_sub  = nh.subscribe("/rob666/guidance/left_image",  1, left_image_callback);
    right_image_sub = nh.subscribe("/rob666/guidance/right_image", 1, right_image_callback);

    laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/rob666/guidance/occupancy_grid_values", 1);

    while(ros::ok()) {
    if(imgFlag >= 2 && imgFlag%2==0 && left_id.compare(right_id) == 0) {  // Initial IMG rendering delays the main loop
      imgDisparity16S = CreateDepthImage(left_img1, right_img1);
      cv::imshow("MORPHOLOGY disp", imgDisparity16S);
      //show_histogram("disp hist", imgDisparity16S);

      cv::waitKey(1);
      imgFlag = 0;
    }

    ros::spinOnce();
  }

  return 0;
}
