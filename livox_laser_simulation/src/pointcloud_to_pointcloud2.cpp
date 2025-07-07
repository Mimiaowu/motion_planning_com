#include <ros/ros.h>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


ros::Subscriber subMMWCloud;
ros::Publisher pubLaserCloud;
std::mutex mBuf;

void mmwHandler(const sensor_msgs::PointCloudConstPtr& mmwCloudMsg)
{
    mBuf.lock();
    // rawMMWCloudQueue.push_back(mmwCloudMsg);
    sensor_msgs::PointCloud2 laserCloudMsg;
    convertPointCloudToPointCloud2(*mmwCloudMsg, laserCloudMsg);
    laserCloudMsg.header.stamp = mmwCloudMsg->header.stamp;
    laserCloudMsg.header.frame_id = "radar_mid360";
    pubLaserCloud.publish(laserCloudMsg);
    // cout << "hello LASER: " << laserCloudMsg.header.stamp << endl;
    mBuf.unlock();
}
int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_to_pointcloud2");
    ros::NodeHandle nh;

    subMMWCloud = nh.subscribe<sensor_msgs::PointCloud>("/pointcloud", 2000, &mmwHandler);
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud2", 2000);
    ros::spin();
}