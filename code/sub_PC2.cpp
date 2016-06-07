
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/io/pcd_io.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include <string>
int iter=0;
ros::Publisher pub;

void cloudrecived(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
if (iter==0){

        ROS_INFO_STREAM("Listening for incomming data on topic ");



        sensor_msgs::PointCloud2 cloudmsg = *cloud;


        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpcl (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB> cloudpcl;

        pcl::fromROSMsg (cloudmsg,cloudpcl);

        pcl::io::savePCDFileASCII ("praticalC.pcd", cloudpcl);



        //int savePCDFile (desired_view , const Pointcloud2& cloud,const Vector4f& origin, const Quaternionf& orientation , bool binary_mode);
        // create a container for the data
        sensor_msgs::PointCloud2 output;

        // do the data processing here....
        output= *cloud;

        //publish the data
        pub.publish(output);
        iter=1;
}


ros::shutdown();


}

int main (int argc, char **argv){
    //initialize the ROS system
    ros::init(argc,argv ,"sub_PC2");

    ros:: NodeHandle nh;


    //create a subscriber object

    ros::Subscriber sub= nh.subscribe ("camera/depth_registered/points",1, cloudrecived);


    // create a ros publisher for the output point cloud
    pub= nh.advertise<sensor_msgs::PointCloud2>("output",1);

    // spin
    ros::spin();


}
