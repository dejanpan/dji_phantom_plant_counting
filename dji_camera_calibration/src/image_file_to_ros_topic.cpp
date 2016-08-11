#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <vector>
#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"

int main(int argc, char** argv)
{
  if (argc != 4)
    {
      ROS_ERROR("Usage ./image_file_to_ros_topic <input_dir> <extension (e.g. .png)> <camera_calibration_file.yml> ");
      return -1;
    }
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("dji_camera/image_raw", 1);
  ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("dji_camera/camera_info", 1);


  std::vector<std::string> files_to_process;
  boost::filesystem::directory_iterator end_iter;
  for (boost::filesystem::directory_iterator iter(argv[1]); iter != end_iter; iter++)
    {
      boost::filesystem::path file(*iter);
	  if (file.extension() == argv[2])
	    {
	      files_to_process.push_back(file.c_str());
	    }
    }

  //CameraInfo
  sensor_msgs::CameraInfo cam_info;
  std::string cam_info_file(argv[3]);
  std::string cam_name;
  bool calib_file_read_success = camera_calibration_parsers::readCalibration(cam_info_file,
									     cam_name,
									     cam_info);

  if (calib_file_read_success)
    {
      ROS_INFO_STREAM("calib file read successfully");
    }
  else
    {
      ROS_WARN_STREAM(cam_info_file << " not valid, won't publish CameraInfo");
    }

  std::sort(files_to_process.begin(), files_to_process.end());
  ros::Rate loop_rate(25);
  for (size_t i = 0; i < files_to_process.size(); i++)
    {
      ROS_INFO_STREAM("Publishing " << files_to_process.at(i));
      
      cv::Mat image = cv::imread(files_to_process.at(i), CV_LOAD_IMAGE_COLOR);
      cv::waitKey(30);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "world";
      cam_info.header.stamp = ros::Time::now();

      pub.publish(msg);
      if(calib_file_read_success)
	{
	  cam_info_pub.publish(cam_info);
	}

      ros::spinOnce();
      loop_rate.sleep();
    }
}
