#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <vector>

int main(int argc, char** argv)
{
  if (argc != 2)
    {
      ROS_ERROR("Usage ./image_file_to_ros_topic input_dir");
      return -1;
    }
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("dji_camera/image_raw", 1);


  std::vector<std::string> files_to_process;
  boost::filesystem::directory_iterator end_iter;
  for (boost::filesystem::directory_iterator iter(argv[1]); iter != end_iter; iter++)
    {
      //      boost::filesystem::path class_dir_path(*iter);
      // for (boost::filesystem::directory_iterator iter2(class_dir_path); iter2 != end_iter; iter2++)
      //	{
	  boost::filesystem::path file(*iter);
	  if (file.extension() == ".JPG")
	    {
	      files_to_process.push_back(file.c_str());
	      ROS_INFO("Read file \t%s\n", file.c_str());
	    }
	  //	}

    }

  std::sort(files_to_process.begin(), files_to_process.end());
  ros::Rate loop_rate(1);
  for (size_t i = 0; i < files_to_process.size(); i++)
    {
      //      ROS_INFO_STREAM(files_to_process.at(i));
      
      cv::Mat image = cv::imread(files_to_process.at(i), CV_LOAD_IMAGE_COLOR);
      cv::waitKey(30);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
}
