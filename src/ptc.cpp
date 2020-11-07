
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
using namespace sensor_msgs;
ros::Publisher depth_pub ;
ros::Publisher cloud_pub;
bool cam_info_taken = false;
tf2_ros::Buffer tf_buffer_;
tf::TransformListener *tf_listener0;
tf::TransformListener *tf_listener1;
tf::TransformListener *tf_listener2;
tf::TransformListener *tf_listener3;
tf::TransformListener *tf_listener4;
float centre_x;
float centre_y;
float focal_x;
float focal_y;
float height;
float width;
std::string target_frame;
CameraInfo info [5];

bool got_info [5] = {false,false,false,false,false};

void camera_info1(const CameraInfoConstPtr& info_msg){
  //if (got_info[0]) return;
  info[0] = *info_msg;
  tf_listener0->waitForTransform("velodyne", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(10.0));
  got_info[0] = true;
}
void camera_info2(const CameraInfoConstPtr& info_msg){
  //if (got_info[1]) return;
  info[1] = *info_msg;
  tf_listener1->waitForTransform("velodyne", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(10.0));
  got_info[1] = true;
}
void camera_info3(const CameraInfoConstPtr& info_msg){
  //if (got_info[2]) return;
  info[2] = *info_msg;
  tf_listener2->waitForTransform("velodyne", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(10.0));
  got_info[2] = true;
}
void camera_info4(const CameraInfoConstPtr& info_msg){
  //if (got_info[3]) return;
  info[3] = *info_msg;
  tf_listener3->waitForTransform("velodyne", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(10.0));
  got_info[3] = true;
}
void camera_info5(const CameraInfoConstPtr& info_msg){
  //if (got_info[4]) return;
   info[4] = *info_msg;
  tf_listener4->waitForTransform("velodyne", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(10.0));
  got_info[4] = true;
}

void callback(const ImageConstPtr& img0,const ImageConstPtr& img1,const ImageConstPtr& img2,const ImageConstPtr& img3,const ImageConstPtr& img4, const PointCloud2ConstPtr& cld)
{
  for  (int i = 0; i < 5; i++){
    if (!got_info[i]) return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cld, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
  pcl::copyPointCloud(*cloud, cloud_color);

  sensor_msgs::PointCloud2 msgt;
  geometry_msgs::TransformStamped transform;
  for (int cam = 0; cam <5; cam++){
    pcl::PointCloud<pcl::PointXYZRGB> cloud_cam;
    sensor_msgs::PointCloud2 msgt;
   

  
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      switch(cam){
      case 0:
        cv_ptr = cv_bridge::toCvShare(img0); 
        pcl_ros::transformPointCloud(info[cam].header.frame_id, cloud_color, cloud_cam, *tf_listener0);
        break;
      case 1:
        cv_ptr = cv_bridge::toCvShare(img1);
        pcl_ros::transformPointCloud(info[cam].header.frame_id, cloud_color, cloud_cam, *tf_listener1);
        break;
      case 2:
        cv_ptr = cv_bridge::toCvShare(img2);
        pcl_ros::transformPointCloud(info[cam].header.frame_id, cloud_color, cloud_cam, *tf_listener2);
        break;
      case 3:
        cv_ptr = cv_bridge::toCvShare(img3);
        pcl_ros::transformPointCloud(info[cam].header.frame_id, cloud_color, cloud_cam, *tf_listener3);
        break;
      case 4:
        cv_ptr = cv_bridge::toCvShare(img4);
        pcl_ros::transformPointCloud(info[cam].header.frame_id, cloud_color, cloud_cam, *tf_listener4);
        break;
      }
    }
    catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        continue;
      }


    float centre_x = info[cam].K[2];
    float centre_y = info[cam].K[5];
    float focal_x = info[cam].K[0];
    float focal_y =info[cam].K[4];
    float height = info[cam].height;
    float width = info[cam].width;
    for (int i=0; i<cloud_cam.points.size();i++){
      if (cloud_cam.points[i].z == cloud_cam.points[i].z){
        float X = cloud_cam.points[i].x;
        float Y = cloud_cam.points[i].y;
        float Z = cloud_cam.points[i].z;
        if (Z < 0) { //speed
          continue;
        }
        float z = Z*1000.0;
        float u = (X*1000.0*focal_x) / z;
        float v = (Y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);
        if (pixel_pos_x < 0 || (pixel_pos_x > (width-1))){
          continue;
        }
        if (pixel_pos_y < 0 ||(pixel_pos_y > (height-1))){
          continue;
        }
        const cv::Point3_<uchar>* p  = cv_ptr->image.ptr<cv::Point3_<uchar> >(pixel_pos_y,pixel_pos_x);
        cloud_color.points[i].r = p->z;
        cloud_color.points[i].g = p->y;
        cloud_color.points[i].b = p->x;
      }
    }
  }
  sensor_msgs::PointCloud2 object_msg;
  pcl::toROSMsg(cloud_color,object_msg );
  object_msg.header.stamp = cld->header.stamp;
  cloud_pub.publish(object_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc_to_img");

  ros::NodeHandle nh;
  ros::NodeHandle nh_("/");

  tf_listener0 = new tf::TransformListener(); 
  tf_listener1 = new tf::TransformListener();
  tf_listener2 = new tf::TransformListener();
  tf_listener3 = new tf::TransformListener();
  tf_listener4 = new tf::TransformListener();

  message_filters::Subscriber<sensor_msgs::Image> cam1_sub(nh,"camera_0/image_raw",2);
  message_filters::Subscriber<sensor_msgs::Image> cam2_sub(nh,"camera_1/image_raw",2);
  message_filters::Subscriber<sensor_msgs::Image> cam3_sub(nh,"camera_2/image_raw",2);
  message_filters::Subscriber<sensor_msgs::Image> cam4_sub(nh,"camera_3/image_raw",2);
  message_filters::Subscriber<sensor_msgs::Image> cam5_sub(nh,"camera_4/image_raw",2);
  message_filters::Subscriber<PointCloud2> point_sub(nh,"/velodyne_points",1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cam1_sub,cam2_sub,cam3_sub,cam4_sub,cam5_sub, point_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3,_4,_5,_6));

  ros::Subscriber img_sub1 =  nh_.subscribe( "camera_0/camera_info",1, camera_info1 );
  ros::Subscriber img_sub2 =  nh_.subscribe( "camera_1/camera_info",1, camera_info2 );
  ros::Subscriber img_sub3 =  nh_.subscribe( "camera_2/camera_info",1, camera_info3 );
  ros::Subscriber img_sub4 =  nh_.subscribe( "camera_3/camera_info",1, camera_info4 );
  ros::Subscriber img_sub5 =  nh_.subscribe( "camera_4/camera_info",1, camera_info5 );

  cloud_pub =  nh.advertise<PointCloud2>( "cloud/colored",1);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
