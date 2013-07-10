//#include "opencv2/calib3d/calib3d.hpp"
//#include <opencv/highgui.h>
//#include <opencv/cv.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
//#include <sensor_msgs/PointCloud.h>

#include <cata_sphereo/ocam_functions.h>
#include <cata_sphereo/lanes.h>
//#include "cata_sphereo/common.h" // TODO: weird...not compiling with this header file

#define CAMERA_HOLD_TIME 33 // milliseconds
static const char OMNI_WINDOW[] = "Omni Image";
static const char MASKED_WINDOW[] = "Masked Image";
static const char SAMPLE_WINDOW[] = "Sample ROI";
static const char THRESH_WINDOW[] = "Threshold White";
static const char PERSPECTIVE_WINDOW[] = "Undistorted Perspective";
static const char RANSAC_WINDOW[] = "RANSAC";

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//#define DEVELOP
//#define OMNI_DEVELOP

void
help()
{
  printf(
      "\n\n"
        "OMNI\n"
        "This uses a monocular video camera to demonstrate motion templates (MHI) and Optical Flow LK with Pyramids.\n"
        "  \n"
        " 1 Point the camera away from you\n"
        " 2 Start the program\n"
        " 3 Wait a few seconds until the video window goes black\n"
        " 4 Move in front of the camera and watch the templates being drawn along with\n"
        "   their segmented motion vectors.\n"
        "   \n"
        "\n");
}

void
thresholdTrackbarHandler(int nBarValue, void *)
{
  ROS_INFO("Threshold: %d\n", nBarValue);
}

void
zoomTrackbarHandler(int nBarValue, void* user_data)
{
  bool* that = static_cast<bool*> (user_data);
  *that = true;
  ROS_INFO("Zoom: %d\n", nBarValue);
}

// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
class Sphereo
{
  public:
    Sphereo(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~Sphereo()
    {
#ifdef DEVELOP
      cv::destroyWindow(OMNI_WINDOW);
#endif
    }
    //void
    //samplePointsOnLine(cv::Point p1, cv::Point p2, double res,
    //    PointCloudT &pts);
    void
    samplePointsOnLineWithFullcrum(cv::Point p1, cv::Point p2, cv::Point fullcrum,
        double res, PointCloudT &cloud);
  private:
    void
    captureVideo();
    void
    setVideoCaptureProperties(cv::VideoCapture &cap);
    void
    getParams();
    bool
    processCVFrames();
    void
    subImageCallback(const sensor_msgs::ImageConstPtr &img_ptr);

    void
    applySelfMask(cv::Mat &img_input, cv::Mat &img_masked);
    void
    collectSamples(cv::Mat& img_src, cv::Mat& img_masked_out);
    void
    getHistogram(const cv::Mat& img_src, cv::Mat& img_masked_out,
        const cv::Mat& img_mask);
    void
    getWhiteThreshold(cv::Mat& img_color, cv::Mat& img_bin);

    void
    undistort(cv::Mat& img_in, cv::Mat& img_out);

    void
    doLaneDetection(cv::Mat & img_bin);
    void
    drawLane(cv::Mat &img, lane L, CvScalar color, PointCloudT &cloud);
    void
    drawLanes(cv::Mat &img, struct lanes Lanes);

    cv::Mat frameCap_;
    cv::VideoCapture capture_; ///< Create an object that decodes the input video stream.

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    int video_port_number_;
    std::string video_file_name_;
    std::string calib_results_minor_mirror_file_name_;
    int camera_width_;
    int camera_height_;
    bool data_collect_;
    bool pub_img_topic_;
    bool sub_to_img_topic_;
    bool has_calib_file_minor;
    bool perspective_;
    bool do_lane_detection_;
    ros::Publisher img_pub_;
    ros::Publisher cloud_pub_;
    ros::Subscriber img_sub_;

    int lane_size_px_;

    struct ocam_model o_cata_;
    int ox_; ///< image center x
    int oy_; ///< image center y
    int inner_radius_; ///< mask's inner radius
    int outer_radius_; ///< mask's outer radius

    int thresholdValue_; ///< For white threshold segmentation
    int threshLevels_;
    int zoom_factor_pers_; ///< Zoom factor for perspective undistortion
    bool zoom_update_;

    double point_line_res_; ///< The scale factor (empirically obtained) for coordinates on the ground on an undistorted perspective image
};
// %EndTag(CLASSDEF)%

Sphereo::Sphereo(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh), nh_private_(nh_private)
{
#ifdef DEVELOP
  cv::namedWindow(OMNI_WINDOW, CV_WINDOW_NORMAL);
  cv::namedWindow(MASKED_WINDOW, CV_WINDOW_NORMAL);
  cv::namedWindow(THRESH_WINDOW, CV_WINDOW_NORMAL);
//  cv::namedWindow(SAMPLE_WINDOW, CV_WINDOW_NORMAL);
#endif

  getParams();

  if (has_calib_file_minor)
    {

      zoom_update_ = true;
#ifdef DEVELOP
      void * user_params = &zoom_update_;
      cv::namedWindow(PERSPECTIVE_WINDOW, CV_WINDOW_NORMAL);
      cv::createTrackbar("Zoom", PERSPECTIVE_WINDOW, &zoom_factor_pers_, 20,
          zoomTrackbarHandler, user_params);
#endif
    }

#ifdef DEVELOP
  cv::createTrackbar("Threshold", THRESH_WINDOW, &thresholdValue_,
      threshLevels_, thresholdTrackbarHandler);
#endif
  if (do_lane_detection_)
      {
        cv::namedWindow(RANSAC_WINDOW, CV_WINDOW_NORMAL);
        cloud_pub_ = nh_private.advertise<PointCloudT> ("cloud", 1);
      }

  if (!sub_to_img_topic_)
    // %Tag(PUB)%
    img_pub_ = nh_private_.advertise<sensor_msgs::Image> ("omni_img", 1);
  // %EndTag(PUB)%
  if (sub_to_img_topic_)
    {
      // %Tag(SUB)%
      img_sub_ = nh_private_.subscribe<sensor_msgs::Image> ("omni_img", 10,
          &Sphereo::subImageCallback, this);
      // %EndTag(SUB)%
    }
  else
    captureVideo();

}

// %Tag(CALLBACK)%
void
Sphereo::subImageCallback(const sensor_msgs::ImageConstPtr &img_ptr)
{
  frameCap_ = cv_bridge::toCvShare(img_ptr)->image;
  processCVFrames();
}
// %EndTag(CALLBACK)%


// %Tag(PARAMS)%
void
Sphereo::getParams()
{

  if (!nh_private_.getParam("do_lane_detection", do_lane_detection_))
    {
      do_lane_detection_ = false;
      ROS_WARN("Not doing lane detection");
    }
  else
    {
      if (do_lane_detection_)
        {
          if (!nh_private_.getParam("lane_size_px", lane_size_px_))
            {
              lane_size_px_ = 100;
            }
          ROS_INFO("Doing lane detection");
        }
      else
        ROS_WARN("Not doing lane detection");
    }

  if (nh_private_.getParam("calib_file_minor",
      calib_results_minor_mirror_file_name_))
    {
      has_calib_file_minor = true;
      ROS_INFO("Calibration file for minor mirror found, undistort is possible");
      // Read the parameters of the omnidirectional camera from the TXT file
      get_ocam_model(&o_cata_, calib_results_minor_mirror_file_name_);
      nh_private_.getParam("perspective", perspective_);
      point_line_res_ = 0.005; // TODO: For now, assume 1px = 0.01m
    }
  else
    {
      has_calib_file_minor = false;
      ROS_WARN("Calibration file for minor mirror NOT found, undistort is not possible");
    }

  if (!nh_private_.getParam("sub_to_img_topic", sub_to_img_topic_))
    {
      sub_to_img_topic_ = false;
      ROS_INFO("ROS is NOT subscribing to image topic, and is using OpenCV only");
    }
  else
    {
      if (sub_to_img_topic_)
        ROS_INFO("ROS is subscribing to image topic");
      else
        ROS_INFO("ROS is NOT subscribing to image topic, and is using OpenCV only");
    }
  if (!nh_private_.getParam("data_collect", data_collect_))
    {
      data_collect_ = bool(false);
      ROS_INFO("Not Collecting data");
    }
  else
    {
      if (data_collect_)
        ROS_INFO("Collecting data");
      else
        ROS_INFO("Not Collecting data");
    }

  if (!nh_private_.getParam("pub_img_topic", pub_img_topic_))
    {
      pub_img_topic_ = bool(false);
      ROS_INFO("ROS is NOT publishing images");
    }
  else
    {
      if (pub_img_topic_)
        ROS_INFO("ROS is publishing images");
      else
        ROS_INFO("ROS is NOT publishing images");
    }

  if (!sub_to_img_topic_)
    {
      if (!nh_private_.getParam("camera_width", camera_width_))
        {
          camera_width_ = 640;
          ROS_INFO("Using default camera WIDTH = %d", camera_width_);
        }
      if (!nh_private_.getParam("camera_height", camera_height_))
        {
          camera_height_ = 480;
          ROS_INFO("Using default camera HEIGHT = %d", camera_height_);
        }

      if (!nh_private_.getParam("video_port_number", video_port_number_))
        {
          if (nh_private_.getParam("video_file_name", video_file_name_))
            {
              ROS_INFO("Using video video file name: \"%s\"", video_file_name_.c_str());
              capture_.open(video_file_name_);
            }
          else
            {
              video_port_number_ = 0;
              ROS_INFO("Using video port number: %d", video_port_number_);
              capture_.open(video_port_number_);
              setVideoCaptureProperties(capture_);
            }
        }
      else
        {
          ROS_INFO("Using video port number: %d", video_port_number_);
          capture_.open(video_port_number_);
          setVideoCaptureProperties(capture_);
        }
    }

  // TODO: Parameters from calibration
  // Manual offset
  int offset_hor = 0;
  int offset_ver = 0;
  if (has_calib_file_minor)
    {
      // TODO: check this!
      ox_ = o_cata_.yc + offset_hor;
      oy_ = o_cata_.xc + offset_ver;
    }
  else
    {
      ox_ = camera_width_ / 2 + offset_hor;
      oy_ = camera_height_ / 2 + offset_ver;
    }
  inner_radius_ = 34;
  outer_radius_ = 66; // Including horizon on image

  // TODO: Fine tune this default value before run
  thresholdValue_ = 235; // Default threshold value for white color thresholding on the value of HSV
  threshLevels_ = 255;
  zoom_factor_pers_ = 4;
}
// %EndTag(PARAMS)%

void
Sphereo::undistort(cv::Mat& img_in, cv::Mat& img_out)
{
#ifdef OMNI_DEVELOP
  // Print ocam_model parameters
  int i;
  printf("pol =\n");
  for (i = 0; i < o_cata_.length_pol; i++)
    {
      printf("\t%e\n", o_cata_.pol[i]);
    };
  printf("\n");
  printf("invpol =\n");
  for (i = 0; i < o_cata_.length_invpol; i++)
    {
      printf("\t%e\n", o_cata_.invpol[i]);
    };
  printf("\n");
  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n", o_cata_.xc,
      o_cata_.yc, o_cata_.width, o_cata_.height);

  // Carlos: coordinates are as follows
  //
  //          -x^
  //            |
  //            |
  //            |
  //  -y<-------+-------> +y    -z ...towards the floor if camera was pointing towards the ceiling
  //            |
  //            |
  //            |
  //            |
  //         +x v  Robot's front

  // --------------------------------------------------------------------
  // WORLD2CAM projects 3D point into the image
  // NOTE!!! The coordinates are expressed according the C convention,
  // that is, from the origin (0,0) instead than from 1 (MATLAB).
  // --------------------------------------------------------------------
  // NOTE: real distance to ground from mirror base is 115 cm = 1.15 meters
  // Undistorted dimensions are non-linear, for instance:
  // TODO: Interpolate
  // 0.5 m = 50px
  // 1.0 m = 150px
  // 1.5 m = 250px
  // For now, assume 1px = 0.01m

  double point3D[3] =
    { 100, 200, -300}; // a sample 3D point
  //  double point3D[3] = { 0.7071 , 0.7071, 0 };       // {sin 45, cos 45, 0} should give the 2-D point at 45 degrees of the 3-D world spherical-mirror
  double point2D[2]; // the image point in pixel coordinates
  //  double point2D[2] = {328, 389};                              // the CENTER image point in pixel coordinates
  //  double point2D[2] = {428, 389};                              // the image point in pixel coordinates
  //  double point2D[2] = {412, 412};                              // the image point in pixel coordinates
  world2cam(point2D, point3D, &o_cata_); // The behaviour of this function is the same as in MATLAB
  // Display re-projected coordinates
  printf("\nworld2cam: pixel coordinates reprojected onto the image\n");
  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);

  // CAM2WORLD back-projects pixel points on to the unit sphere
  // The behaviour of this function is the same as in MATLAB
  cam2world(point3D, point2D, &o_cata_);

  // Display back-projected normalized coordinates (on the unit sphere)
  printf(
      "\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);
  //
  //    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //    // Getting points coords:
  //    cv::namedWindow("LEFT", CV_WINDOW_AUTOSIZE);
  //    cv::Mat tempSrc(img_in.size(), CV_8UC3);
  //    if(img_in.type() = CV_8UC1)
  //      {
  //      cv::Mat img_input_array[] = {img_in, img_in, img_in};
  //      cv::merge(img_input_array, 3, tempSrc);
  //      }
  //    else
  //      img_in.copyTo(tempSrc);
  //
  //    cvSetMouseCallback("LEFT", on_mouseLeft, 0, &tempSrc);
  //
  //    // Hold the thread
  //    while ((cv::waitKey(300) & 255) != 27) // as long as ESC key is not pressed
  //      {
  //            cv::imshow("LEFT", tempSrc);
  //      }
  //    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif
  static CvMat* mapx_persp;
  static CvMat* mapy_persp;
  static CvMat* mapx_pan;
  static CvMat* mapy_pan;

  static bool isFirstRun = true;

  if (perspective_)
    {

      // --------------------------------------------------------------------
      // Allocate space for the undistorted images
      // --------------------------------------------------------------------


      // --------------------------------------------------------------------
      // Create Look-Up-Table for perspective undistortion
      // SF is kind of distance from the undistorted image to the camera
      // (it is not meters, it is justa zoom fator)
      // Try to change SF to see how it affects the result
      // The undistortion is done on a  plane perpendicular to the camera axis
      // --------------------------------------------------------------------
      cv::Mat dst_persp(img_in.size(), img_in.type()); // undistorted perspective and panoramic image
      if (isFirstRun || zoom_update_)
        {
          mapx_persp = cvCreateMat(img_in.rows, img_in.cols, CV_32FC1);
          mapy_persp = cvCreateMat(img_in.rows, img_in.cols, CV_32FC1);
          create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, &o_cata_,
              zoom_factor_pers_);

          zoom_update_ = false;
        }

      //TODO: test different border types
      // --------------------------------------------------------------------
      // Undistort using specified interpolation method
      // Other possible values are (see OpenCV doc):
      // CV_INTER_NN - nearest-neighbor interpolation,
      // CV_INTER_LINEAR - bilinear interpolation (used by default)
      // CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
      // CV_INTER_CUBIC - bicubic interpolation.
      // --------------------------------------------------------------------
      cv::remap(img_in, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR,
          cv::BORDER_REFLECT, cv::Scalar::all(0));
      //          cv::BORDER_TRANSPARENT, cv::Scalar::all(0));

      img_out = dst_persp;
    }
  else
    {
      // --------------------------------------------------------------------
      // Create Look-Up-Table for panoramic undistortion
      // The undistortoin is just a simple cartesia-to-polar transformation
      // Note, only the knowledge of image center (xc,yc) is used to undisort the image
      // xc, yc are the row and column coordinates of the image center
      // Note, if you would like to flip the image, just inverte the sign of theta in this function
      // --------------------------------------------------------------------
      cv::Size size_pan_image(800, 600); // TODO: size of the undistorted panoramic image
      cv::Mat dst_pan(size_pan_image, img_in.type()); // undistorted panoramic image
      if (isFirstRun)
        {
          float Rmax = 470; // the maximum radius of the region you would like to undistort into a panorama
          float Rmin = 20; // the minimum radius of the region you would like to undistort into a panorama
          mapx_pan = cvCreateMat(img_in.rows, img_in.cols, CV_32FC1);
          mapy_pan = cvCreateMat(img_in.rows, img_in.cols, CV_32FC1);
          create_panoramic_undistortion_LUT(mapx_pan, mapy_pan, Rmin, Rmax,
              o_cata_.xc, o_cata_.yc);
        }

      // TODO: border type
      cv::remap(img_in, dst_pan, mapx_pan, mapy_pan, CV_INTER_LINEAR,
          cv::BORDER_CONSTANT, cv::Scalar::all(0));
      img_out = dst_pan;

    }

  isFirstRun = false;

}

void
Sphereo::captureVideo()
{
  help();

  bool processing = true;
  while (processing)
    {
      if (capture_.isOpened() && !sub_to_img_topic_)
        {
          if (!capture_.grab())
            continue;
          capture_ >> frameCap_;
        }
      else
        {
          if (!sub_to_img_topic_)
            {
              ROS_ERROR("Failed to open camera or file!");
              break;
            }
        }

      processing = processCVFrames();

    }
  ROS_WARN("No longer capturing video");

}
void
Sphereo::doLaneDetection(cv::Mat & img_bin)
{
  static line_segments segments;
  static struct lanes Lanes;

  segments.init();

  IplImage img_bin_ipl = img_bin;
  getSegmentSequence(&img_bin_ipl, &segments);

  segments.compute_weights(Lanes);

  int numIterations = 200; //TODO
  Lanes = findLanesRANSAC(&img_bin_ipl, &segments, RELEASE_MODE, numIterations, 5); // TODO: set RANSAC parameters
  cv::Mat lanes_img(&img_bin_ipl);

  drawLanes(lanes_img, Lanes);

}

void
Sphereo::drawLanes(cv::Mat &img, struct lanes Lanes)
{
  cv::Mat img_color(img.size(), CV_8UC3);
  cv::cvtColor(img, img_color, CV_GRAY2BGR);

  PointCloudT cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "omni_cloud_frame";

  bool hasLanes = false;

  for (int i = 0; i < Lanes.num_of_lanes; i++)
    {
      if (Lanes.Lane[i].isVisible)
        {
          hasLanes = true;
          drawLane(img_color, Lanes.Lane[i], COLOR_MAGENTA, cloud);
        }

    }

  cv::imshow(RANSAC_WINDOW, img_color);

  if (hasLanes && (cloud_pub_.getNumSubscribers() > 0))
    {
      cloud_pub_.publish(cloud);
    }
  //drawDirectionArrow(img, cvPoint(200, 120), 100, Lanes.direction);

}

void
Sphereo::drawLane(cv::Mat &img, lane L, CvScalar color, PointCloudT &cloud)
{
  cv::Point2i p = L.fulcrum;
  cv::Point2i pt1, pt2;

  float c0 = cos(L.t0);
  float s0 = sin(L.t0);
  float c1 = cos(L.t1);
  float s1 = sin(L.t1);
  // TODO:change lane sizes
  pt1.x = lane_size_px_ *cvRound(p.x + (-c0));
  pt1.y = lane_size_px_ *cvRound(p.y + (s0));

  cv::line(img, p, pt1, color, 2, 8);
  //  drawDirectionArrow(img, p, 50, L.t0, color);

//  pt2.x = cvRound(p.x + lane_size_px_ * (-c1)); // TODO: not sure if broken with merge, which one is good??
//  pt2.y = cvRound(p.y + lane_size_px_ * (s1));
  pt2.x = lane_size_px_ *cvRound(p.x + (-c1));
  pt2.y = lane_size_px_ *cvRound(p.y +  (s1));
//  pt2.x = cvRound(p.x + 100 * (-c1));
//  pt2.y = cvRound(p.y + 100 * (s1));

  cv::line(img, p, pt2, color, 2, 8);
  //  drawDirectionArrow(img, p, 50, L.t1, color);

  cv::circle(img, p, 5, color, 3, 8, 0);

  if (cloud_pub_.getNumSubscribers() > 0)
    {
      // TODO: scale down points
      samplePointsOnLineWithFullcrum(pt1, pt2, p, point_line_res_, cloud); // TODO:merge the 2 lanes (this is only working for 1 line
    }
}

/** \brief Sample points on a line (formed by 3 points, midle is the fullcrum). Using a given resolution <res>
 * \param p1 the first point
 * \param p2 the second point
 * \param res the resolution of the line
 * \param pts the resultant points
 */
void
Sphereo::samplePointsOnLineWithFullcrum(cv::Point p1, cv::Point p2, cv::Point fullcrum,
    double res, PointCloudT &cloud)
{
  float z_value = 0.1; // TODO: setup properly

  cv::Point3f axis_x(1.0, 0.0, 0.0), axis_y(0.0, 1.0, 0.0), axis_z(0.0, 0.0,
      1.0);

  cv::Point dpA = p1 - fullcrum;

  // Compute the distance between the two points
  float distA = cv::norm(dpA);
  //    dp.normalize (); // TODO
//#ifdef DEVELOP
  ROS_INFO("DistA = %f", distA);
//#endif

  cv::Point3f dp3DA(dpA.x, dpA.y, z_value);

  // Compute the axis increments
  float dxA = res * dp3DA.dot(axis_x);
  float dyA = res * dp3DA.dot(axis_y);
  //  float dzA = res * dp3DA.dot(axis_z);

  int nr_ptsA = distA / res;

  cv::Point dpB = p2 - fullcrum;

  // Compute the distance between the two points
  float distB = cv::norm(dpB);
//#ifdef DEVELOP
  ROS_INFO("DistB = %f", distB);
//#endif
  //    dp.normalize (); // TODO

  cv::Point3f dp3DB(dpB.x, dpB.y, z_value);

  // Compute the axis increments
  float dxB = res * dp3DB.dot(axis_x);
  float dyB = res * dp3DB.dot(axis_y);
  //  float dzB = res * dp3DB.dot(axis_z);

  int nr_ptsB = distB / res;

  int nr_pts = nr_ptsA + nr_ptsB;
//#ifdef DEVELOP
  ROS_INFO("Number of Points = %d", nr_pts);
//#endif
  if (nr_pts > 1)
    {
      cloud.is_dense = false;
      cloud.width = nr_pts;
      cloud.height = 1;
      cloud.points.resize(nr_pts);
      int ptIndex = 0;
      for (int m = 0; m < nr_ptsA; m++, ptIndex++)
        {
          // x,y,z
          PointT pointTemp;
          pointTemp.x = ((m + 1) * dxA + p1.x);
          pointTemp.y = ((m + 1) * dyA + p1.y);
          pointTemp.z = z_value; //(m + 1) * dzA + p1.z;
          //PointT pointTemp(x, y, z);
          cloud.points[ptIndex] = pointTemp;
#ifdef OMNI_DEVELOP
          ROS_INFO("P[%d]= (%f, %f, %f)", ptIndex, pointTemp.x, pointTemp.y, pointTemp.z);
#endif
        }

      for (int m = 0; m < nr_ptsB; m++, ptIndex++)
        {
          // x,y,z
          PointT pointTemp;
          pointTemp.x = ((m + 1) * dxB + p2.x);
          pointTemp.y = ((m + 1) * dyB + p2.y);
          pointTemp.z = z_value; //(m + 1) * dzB + p.z;
          //          PointT pointTemp(x, y, z);
          cloud.points[ptIndex] = pointTemp;
#ifdef OMNI_DEVELOP
          ROS_INFO("P[%d]= (%f, %f, %f)", ptIndex, pointTemp.x, pointTemp.y, pointTemp.z);
#endif
        }
    }
}
/** \brief Sample points on a line (formed from p1 and p2) using a given
 * resolution <res>
 * \param p1 the first point
 * \param p2 the second point
 * \param res the resolution of the line
 * \param pts the resultant points
 */
/*
 void
 Sphereo::samplePointsOnLine(cv::Point p1, cv::Point p2, double res,
 PointCloudT &points)
 {
 float z_value = 0.0;
 cv::Point dp = p2 - p1;

 // Compute the distance between the two points
 float dist = cv::norm(dp);
 //    dp.normalize (); // TODO

 cv::Point3f dp3D(dp.x, dp.y, 0.01);
 cv::Point3f axis_x(1.0, 0.0, 0.0), axis_y(0.0, 1.0, 0.0), axis_z(0.0, 0.0,
 1.0);
 // Compute the axis increments
 float dx = res * dp3D.dot(axis_x);
 float dy = res * dp3D.dot(axis_y);
 //  float dz = res * dp3D.dot(axis_z);

 int nr_pts = dist / res;
 if (nr_pts > 1)
 {
 points.fields.resize(3);
 points.fields[0].name = "x";
 points.fields[0].offset = 0;
 points.fields[0].count = 1;
 points.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
 points.fields[1].name = "y";
 points.fields[1].offset = 4;
 points.fields[1].count = 1;
 points.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
 points.fields[2].name = "z";
 points.fields[2].offset = 8;
 points.fields[2].count = 1;
 points.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
 //        points.fields[3].name = "rgb";
 //        points.fields[3].offset = 12;
 //        points.fields[3].count = 1;
 //        points.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
 //points.is_bigendian = false; ???
 points.point_step = 16;
 points.row_step = points.point_step * points.width;
 points.data.resize(points.row_step * points.height);
 //        points.is_dense = false; // there may be invalid points
 points.is_dense = true; // all points are valid
 points.data.resize(nr_pts);
 points.width = nr_pts;
 points.height = 1;
 for (int m = 0; m < nr_pts; ++m)
 {
 // x,y,z
 float x = (m + 1) * dx + p1.x;
 float y = (m + 1) * dy + p1.y;
 float z = z_value; //(m + 1) * dz + p1.z;
 memcpy(&points.data[m * points.point_step + 0], &x, sizeof(float));
 memcpy(&points.data[m * points.point_step + 4], &y, sizeof(float));
 memcpy(&points.data[m * points.point_step + 8], &z, sizeof(float));

 }
 }
 }
 */
bool
Sphereo::processCVFrames()
{
  char chFileName[20];
  static int fileSequenceCounter = 0;

  if (!frameCap_.empty())
    {
      //              if (!motion.empty())
      //                {
      //                  motion
      //                      = cv::Mat::zeros(frameCap_.rows, frameCap_.cols, CV_8UC3); // Color
      //                    motion =  cv::Mat::zeros(frameCap_.rows,frameCap_.cols, CV_8UC1); // Grayscale
      //                    motion.origin = frameCap_.origin; // not sure if necessary to align the origins

      //                }

      //                          update_mhi( frameCap_, motion, 30 );
      //              update_mhi_buffered( frameCap_, motion, 30 );
      //              update_mhi_simple(frameCap_, motion, 30);
      //              cv::imshow("Motion", motion);
#ifdef DEVELOP
      cv::imshow(OMNI_WINDOW, frameCap_);
#endif
      cv::Mat img_omni_masked;
      applySelfMask(frameCap_, img_omni_masked);
#ifdef DEVELOP
      cv::imshow(MASKED_WINDOW, img_omni_masked);
#endif
      cv::Mat img_white_thresh;
//      collectSamples(frameCap_, img_bin_sampled); // Not doing histogram matching
      getWhiteThreshold(img_omni_masked, img_white_thresh);
      if (has_calib_file_minor)
        {
          cv::Mat img_undistorted_masked;
          undistort(img_white_thresh, img_undistorted_masked);
#ifdef DEVELOP
          cv::imshow(PERSPECTIVE_WINDOW, img_undistorted_masked);
#endif
          if (do_lane_detection_)
            {
              doLaneDetection(img_undistorted_masked);
            }
        }

      char chPressedWaitKey;
      if (data_collect_)
        chPressedWaitKey = cv::waitKey(0);
      else
        chPressedWaitKey = cv::waitKey(CAMERA_HOLD_TIME);
      //        chPressedWaitKey = cv::waitKey(0);

      if ((chPressedWaitKey & 255) == 27)
        {
          return false;
        }
      else
        {
          if (((chPressedWaitKey & 255) == 's') && data_collect_)
            {
              // Write data to file
              sprintf(chFileName, "image_%d.jpg", fileSequenceCounter); // Use jpg for calibration!!!
              if (cv::imwrite(chFileName, frameCap_))
                {
                  ROS_INFO("Saved %s", chFileName);
                  fileSequenceCounter++;
                }
            }

          if (pub_img_topic_ && !sub_to_img_topic_) // Should not subscribe and publish to the same topic
            {
              sensor_msgs::ImagePtr ros_img_ptr;

              cv_bridge::CvImage cv_img_frame;
              cv_img_frame.header.frame_id = "omni_img_frame"; // Same timestamp and tf frame as input image
              cv_img_frame.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
              //                  cv_img_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
              cv_img_frame.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
              cv_img_frame.image = frameCap_; // Your cv::Mat
              ros_img_ptr = cv_img_frame.toImageMsg();

              img_pub_.publish(ros_img_ptr);
              // It can be viewed in the command line as following:
              // $ rosrun image_view image_view image:=/sphereo_node/omni_img
            }

        }
    }
  else
    {
      ROS_WARN("Empty capture frame!");
    }

  return true;
}

void
Sphereo::collectSamples(cv::Mat& img_src, cv::Mat& img_masked_out)
{
  //  static bool isFirstRun = true;
  cv::Mat mask = cv::Mat::zeros(img_src.size(), CV_8UC1); // Black, single channel mask

  //  if (isFirstRun)
    {
      // Create halo (ring) around robot's body
      // Paint outer circumference:
      int halo_thickness = 10;
      int halo_outer_radius = inner_radius_ + halo_thickness;
      cv::circle(mask, cv::Point(ox_, oy_), halo_outer_radius,
          CV_RGB(255, 255, 255), CV_FILLED, 8, 0);
      // Paint inner circumference:
      cv::circle(mask, cv::Point(ox_, oy_), inner_radius_, CV_RGB(0, 0, 0),
          CV_FILLED, 8, 0);
      // Mask the tail (above halo: e-stop and wires)
      int rect_width = halo_thickness * 4;
      int rect_height = halo_thickness * 2;
      cv::Rect tail(ox_ - rect_width / 2, oy_ - inner_radius_ - halo_thickness,
          rect_width, rect_height); // (_Tp _x, _Tp _y, _Tp _width, _Tp _height);
      cv::rectangle(mask, tail, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
      // Mask the nose (below halo: laser rangefinder)
      rect_width = halo_thickness * 2;
      rect_height = halo_thickness / 3 * 2;
      cv::Rect nose(ox_ - rect_width / 2, oy_ + inner_radius_, rect_width,
          rect_height); // (_Tp _x, _Tp _y, _Tp _width, _Tp _height);
      cv::rectangle(mask, nose, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);

      //      isFirstRun = false;
    }

  getHistogram(img_src, img_masked_out, mask);

#ifdef DEVELOP
  cv::Mat maskedImg;
  cv::bitwise_and(frameCap_, frameCap_, maskedImg, mask); // Apply mask
  cv::imshow(SAMPLE_WINDOW, maskedImg);
#endif
}

void
Sphereo::getHistogram(const cv::Mat& img_src, cv::Mat& img_masked_out,
    const cv::Mat& img_mask)
{
  //static bool isFirstRun = true; // TODO: is running all the time because it should segment on ramps,too
  // We should have colors for grass and ramps, and dirt?

  cv::Mat img_hsv(img_src.size(), CV_8UC3);
  cv::cvtColor(img_src, img_hsv, CV_BGR2HSV);

  // let's quantize the hue to 30 levels
  // and the saturation to 32 levels
  int hbins = 30, sbins = 32;
  int histSize[] =
    { hbins, sbins };

  // hue varies from 0 to 179, see cvtColor ???
  // hue varies from 0 (~0 deg red) to 180 (~360 deg red again)
  float hranges[] =
    { 0, 180 };
  // saturation varies from 0 (black-gray-white) to
  // 255 (pure spectrum color)
  float sranges[] =
    { 0, 256 };
  const float* ranges[] =
    { hranges, sranges };
  cv::MatND hist;
  // we compute the histogram from the 0-th (hue) and 1-st (sat.) channels
  int channels[] =
    { 0, 1 };

  //      cv::calcHist( &img_hsv, 1, channels, cv::Mat(), // do not use mask
  cv::calcHist(&img_hsv, 1, channels, img_mask, // do not use mask
      hist, 2, histSize, ranges, true, // the histogram is uniform
      false);

  double maxVal = 0;
  cv::Point maxPoint;
  minMaxLoc(hist, 0, &maxVal, 0, &maxPoint);
#ifdef OMNI_DEVELOP
  ROS_INFO("Max value: %f at hist(%d,%d)", maxVal, maxPoint.x, maxPoint.y);
#endif

  int scale = 10;
  cv::Mat img_hist = cv::Mat::zeros(sbins * scale, hbins * scale, CV_8UC3);

  for (int h = 0; h < hbins; h++)
    for (int s = 0; s < sbins; s++)
      {
        float binVal = hist.at<float> (h, s);
        int intensity = cvRound(binVal * 255 / maxVal);
        cv::rectangle(img_hist, cv::Point2d(h * scale, s * scale),
            cv::Point2d((h + 1) * scale - 1, (s + 1) * scale - 1),
            cv::Scalar::all(intensity), // hsv
            //                           CV_RGB(intensity,255,(int)((float)h/hbins*180.0)), //vsh
            CV_FILLED);
      }
  cv::Mat img_back;
  cv::calcBackProject(&img_hsv, 1, channels, hist, img_back, ranges, 1, true);
  // Calculate back projection


  //**********************************************************************************************
  // img_back contains the probability map (0-255) of obstacles excluding lines and white objects
  //**********************************************************************************************
  cv::GaussianBlur(img_back, img_back, cv::Size(5, 5), 2);
  //  cv::dilate(img_back, img_back, cv::Mat());
  cv::bitwise_not(img_back, img_back);

  //***********************************************************************************************
  //probImg_white contains probability map (0-255) of white objects including lanes and white obstacles
  //***********************************************************************************************

//  getWhiteThreshold(img_hsv, img_back); //obtain the white prob. image by simple thresholding
  //cvNot(probImg_white, probImg_white);
  //cvErode(probImg_white, probImg_white, 0, 1);
  //cvDilate(probImg_white, probImg_white, 0, 5);
  //cvSmooth(probImg_white, probImg_white, 2, 3);

  //***********************************************************************************************
  //probImg_white_nolanes contains probability map (0-255) of white obstacles EXCLUDING lanes
  //***********************************************************************************************
  //  cvErode(probImg_white, probImg_nolanes, 0, 2); //erode so that thin lines dissappear
  //  cvDilate(probImg_nolanes, probImg_nolanes, 0, 8); //now blow it up to fatten the remaining obstacles

  img_masked_out = img_back;

#ifdef DEVELOP
  cv::namedWindow("H-S Histogram", CV_WINDOW_NORMAL);
  cv::imshow("H-S Histogram", img_hist);

  // Show back projection
  cv::namedWindow("Back Projection", CV_WINDOW_NORMAL);
  cv::imshow("Back Projection", img_back);
#endif
}

void
Sphereo::getWhiteThreshold(cv::Mat& img_src, cv::Mat& img_bin)
{
  cv::Mat img_hsv(img_src.size(), CV_8UC3);
  cv::cvtColor(img_src, img_hsv, CV_BGR2HSV);

  cv::Mat img_h_plane(img_hsv.size(), CV_8UC1);
  cv::Mat img_s_plane(img_hsv.size(), CV_8UC1);
  cv::Mat img_v_plane(img_hsv.size(), CV_8UC1);
  cv::Mat img_hsv_planes_array[] =
    { img_h_plane, img_s_plane, img_v_plane };
  cv::split(img_hsv, img_hsv_planes_array);
  //        static cv::Mat img_b(img_color.size(), CV_8UC1);
  //        static cv::Mat img_g(img_color.size(), CV_8UC1);
  //        static cv::Mat img_r(img_color.size(), CV_8UC1);
  //        static cv::Mat ptr_images[] = {img_b, img_g, img_r};
  //        cv::split(img_color,ptr_images);
  //        cv::threshold(img_b,img_bin,threshold,255,CV_THRESH_BINARY); //apply static threshold to blue plane
  cv::threshold(img_v_plane, img_bin, thresholdValue_, threshLevels_,
      CV_THRESH_BINARY); //apply static threshold to saturation plane
  cv::dilate(img_bin, img_bin, cv::Mat());
  cv::medianBlur(img_bin, img_bin, 7);
#ifdef DEVELOP
  cv::imshow(THRESH_WINDOW, img_bin);
#endif
}

void
Sphereo::applySelfMask(cv::Mat &img_input, cv::Mat &img_masked)
{
  static bool isFirstRun = true;

  static cv::Mat mask = cv::Mat::zeros(img_input.size(), CV_8UC1); // Black, single channel mask

  if (isFirstRun)
    {
      // Paint outer perimeter:
      cv::circle(mask, cv::Point(ox_, oy_), outer_radius_,
          CV_RGB(255, 255, 255), CV_FILLED, 8, 0);
      // Paint inner perimeter:
      cv::circle(mask, cv::Point(ox_, oy_), inner_radius_, CV_RGB(0, 0, 0),
          CV_FILLED, 8, 0);
      // Mask the tail (above halo: e-stop and wires)
      int rect_width = 20;
      int rect_height = img_input.rows / 6 * 5;
      cv::Rect tail(ox_ - rect_width / 2, oy_ - rect_height, rect_width,
          rect_height); // (_Tp _x, _Tp _y, _Tp _width, _Tp _height);
      cv::rectangle(mask, tail, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
      // Mask the nose (below halo: laser rangefinder)
      rect_width = rect_width / 2;
      rect_height = rect_width;
      cv::Rect nose(ox_ - rect_width / 2, oy_ + (outer_radius_-inner_radius_), rect_width,
          rect_height); // (_Tp _x, _Tp _y, _Tp _width, _Tp _height);
      cv::rectangle(mask, nose, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
    }
  isFirstRun = false;

  cv::bitwise_and(img_input, img_input, img_masked, mask); // Apply mask

}

void
Sphereo::setVideoCaptureProperties(cv::VideoCapture &cap)
{
  cap.set(CV_CAP_PROP_FRAME_WIDTH, camera_width_);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera_height_);
  ROS_INFO("NEW WxH = %f x %f\n", cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  // NOTE: if resolution does NOT change, you may have to use the newer libv4l-based wrapper and recompile opencv2 with V4L=ON
  //  >>>> install libv4l-dev (this is how it's called in Ubuntu)

  //TODO:
  //  cap.set(CV_CAP_PROP_FPS, 20);
}

// %Tag(MAIN)%
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "sphereo_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Sphereo sphereo_sense(nh, nh_private);

  ros::spin();
}
// %EndTag(MAIN)%


