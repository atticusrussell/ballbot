//
// Created by yahboom on 2021/7/30.
//

#include "KCF_Tracker.h"

Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;

const int &ACTION_ESC = 27;
const int &ACTION_SPACE = 32;

void onMouse(int event, int x, int y, int, void *) {
    if (select_flag) {
        selectRect.x = MIN(origin.x, x);
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);
        selectRect.height = abs(y - origin.y);
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == 1) {
//    if (event == CV_EVENT_LBUTTONDOWN) {
        bBeginKCF = false;
        select_flag = true;
        origin = Point(x, y);
        selectRect = Rect(x, y, 0, 0);
    } else if (event == 4) {
//    } else if (event == CV_EVENT_LBUTTONUP) {
        select_flag = false;
        bRenewROI = true;
    }
}

ImageConverter::ImageConverter(ros::NodeHandle &n) {
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
    float linear_KP=0.9;
    float linear_KI=0.0;
    float linear_KD=0.1;
    float angular_KP=0.5;
    float angular_KI=0.0;
    float angular_KD=0.2;


    image_sub_ = n.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
    depth_sub_ = n.subscribe("/camera/depth/image_raw", 1, &ImageConverter::depthCb, this);
    Move_sub_ = n.subscribe("/grab", 1, &ImageConverter::MoveCb, this);
    image_pub_ = n.advertise<sensor_msgs::Image>("/KCF_image", 1);
    pub_pos = n.advertise<dofbot_pro_info::Position>("/pos_xyz", 1);
    pub.publish(geometry_msgs::Twist());
    server.setCallback(f);
    namedWindow(RGB_WINDOW);
}

ImageConverter::~ImageConverter() {
    pub.publish(geometry_msgs::Twist());
    n.shutdown();
    pub.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;

    destroyWindow(RGB_WINDOW);
//        destroyWindow(DEPTH_WINDOW);
}



void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    get_depth = 0.0;

    pub.publish(geometry_msgs::Twist());
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(rgbimage);
    setMouseCallback(RGB_WINDOW, onMouse, 0);
    if (bRenewROI) {
         if (selectRect.width <= 0 || selectRect.height <= 0)
         {
             bRenewROI = false;
             return;
         }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }
    if (bBeginKCF) {
        result = tracker.update(rgbimage);
        rectangle(rgbimage, result, Scalar(0, 255, 0), 1, 8);

        circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
    } else rectangle(rgbimage, selectRect, Scalar(0, 255, 0), 2, 8, 0);
    sensor_msgs::ImagePtr kcf_imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbimage).toImageMsg();
    image_pub_.publish(kcf_imagemsg);
    std::string text = std::to_string(get_depth);
    std::string units= "m";
    text  = text + units;
    cv::Point org(30, 30); 
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    cv::Scalar color(0, 0, 255);  // 文字颜色：红色
    putText(rgbimage, text, org, fontFace, fontScale, color, thickness);
    imshow(RGB_WINDOW, rgbimage);
    int action = waitKey(1) & 0xFF;
    if (action == 'q' || action == ACTION_ESC) this->Cancel();
    else if (action == 'r'||action == 'R')  this->Reset();
    else if (action == ACTION_SPACE) enable_get_depth = true;
}

void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
    if (enable_get_depth) {
        int center_x = (int)(result.x + result.width / 2);
        int center_y = (int)(result.y + result.height / 2);
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5)/1000;
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5)/1000;
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5)/1000;
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5)/1000;
        dist_val[4] = depthimage.at<float>(center_y, center_x)/1000;
        float distance = 0;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
           
            if (dist_val[i] !=0 ) distance += dist_val[i];
            else num_depth_points--;
        }
        distance /= num_depth_points;
        dofbot_pro_info::Position pos;
		pos.x = center_x;
		pos.y = center_y;
		//pos.distance = distance;
        if (std::isnan(distance))
        {
            ROS_INFO("distance error!");
            distance = 999;
        }
        pos.z = distance;
        get_depth = distance;
        ROS_INFO("center_x =  %d, center_y =  %d,distance = %.3f", center_x,center_y,distance);
		pub_pos.publish(pos);
    }
//        imshow(DEPTH_WINDOW, depthimage);
    waitKey(1);
}

void ImageConverter::Cancel() {
    this->Reset();
    ros::Duration(0.5).sleep();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    n.shutdown();
    pub.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    destroyWindow(RGB_WINDOW);
//        destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::JoyCb(const std_msgs::BoolConstPtr &msg) {
    enable_get_depth = msg->data;
}

void ImageConverter::MoveCb(const std_msgs::BoolConstPtr &msg) {
    if (msg->data==true)
    {
        this->Reset();
        ROS_INFO("Rest Mode!");
    }
    
}




