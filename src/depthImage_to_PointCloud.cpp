/*  ROS Node for converting a Depth Image to a PointCloud
 *  using OpenCV and ROS depth_conversions
 *
 * 2021.5.
 * Ahn, Jeeho
 */

//ROS
#include <ros/ros.h>
//#include <ros/package.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
//#include <ros/service.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
//#include <eigen_conversions/eigen_msg.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>


image_geometry::PinholeCameraModel* cam_model_ptr = nullptr;

sensor_msgs::PointCloud2::Ptr depth_to_pointcloud(const sensor_msgs::ImageConstPtr& dimg)
{
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = dimg->header;
  cloud_msg->height = dimg->height;
  cloud_msg->width  = dimg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  cv_bridge::CvImagePtr imPtr = cv_bridge::toCvCopy(dimg);
  //realsense depth images seem to be rectified beforehand
  /*
  std::cout << "Rectify Depth Image" << std::endl;
  cv::Mat rect;
  cam_model_ptr->rectifyImage(imPtr->image,rect,0);
  */

  //back to ROS msg
  //cv_bridge::CvImage rectCV(imPtr->header,imPtr->encoding,rect);
  cv_bridge::CvImage rectCV(imPtr->header,imPtr->encoding,imPtr->image);
  auto rectMsg = rectCV.toImageMsg();

  std::cout << "Converting Depth Image to PointCloud" << std::endl;
  //auto time_start = std::chrono::system_clock::now();

  if (dimg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || dimg->encoding == sensor_msgs::image_encodings::MONO16)
  {
    depth_image_proc::convert<uint16_t>(rectMsg, cloud_msg, *cam_model_ptr);
  }
  else if (dimg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    depth_image_proc::convert<float>(rectMsg, cloud_msg, *cam_model_ptr);
  }
  else
  {
    std::cout << "ERROR dealing with depth image - encoding problem - returning nullptr" << std::endl;
    return nullptr;
  }
  //auto time_finished = std::chrono::system_clock::now();

  //approx. 2 ms
  //std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(time_finished - time_start).count()/1000 << " us" << std::endl;

  return cloud_msg;
}

int main(int argc, char** argv)
{
  // ros initialization
  std::cout << "\tDepth Image to PointCloud" << std::endl;
  ros::init(argc, argv, "Depth2PCD");
  ros::NodeHandle nh;
  ros::Rate rate(10); //10 hz
  ros::Duration(1.0).sleep(); //for debug attach

  //for visualize purpose
  ros::Publisher pubPCD = nh.advertise<sensor_msgs::PointCloud2>
          ("/depthImg_pcd",2);

  // Need to use the topic name that's to be subscribed
  const sensor_msgs::CameraInfoConstPtr& cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info",nh);
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info);
  cam_model_ptr = &cam_model;

  //get depth image
  auto depthMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/depth/image_rect_raw",nh);
  auto pcd = depth_to_pointcloud(depthMsg);

  //for visualize purpose
  while(ros::ok())
  {
    ros::spinOnce();
    pubPCD.publish(pcd);
    rate.sleep();
  }

  return 0;
}

