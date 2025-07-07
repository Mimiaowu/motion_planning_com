# include <ros/ros.h>
# include <nav_msgs/Odometry.h>
# include <gazebo_msgs/ModelStates.h>
# include <tf2_ros/transform_broadcaster.h>
# include "tf2/LinearMath/Quaternion.h"
# include <rosgraph_msgs/Clock.h>

geometry_msgs::Pose last_received_pose;
geometry_msgs::Twist last_received_twist;
ros::Time last_received_stamp;
bool is_last_stamp;

size_t model_index = -1;

// odom related
std::string reference_frame, child_frame;
ros::Publisher odom_pub;

//--------模型状态回调函数
void ModelStates_callback(const gazebo_msgs::ModelStatesConstPtr &msg){
    // 找到model对应的索引
    for(size_t i = 0; i < msg->name.size(); i++){
        if(msg->name[i] == "hunter_se"){
            model_index = i;
        }
    }
    last_received_pose = msg->pose[model_index];
    last_received_twist = msg->twist[model_index];
    if(last_received_stamp == ros::Time::now())
    {
        is_last_stamp = false;
    }
    else
    {
        last_received_stamp = ros::Time::now();
        is_last_stamp = true;

    }

    // ROS_INFO_STREAM("已经收到model的状态, model的索引为: "<< model_index);

    // 里程计
    // std::cout << "时间：" << last_received_stamp.toSec() << std::endl;
    nav_msgs::Odometry odom;
    odom.header.frame_id = reference_frame;
    odom.header.stamp = last_received_stamp;
    // odom.header.stamp = ros::Time::now();
    odom.child_frame_id = child_frame;
    odom.pose.pose = last_received_pose;
    odom.twist.twist = last_received_twist;

    // tf广播
    static tf2_ros::TransformBroadcaster broadcaster;
    // 创建广播数据
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = reference_frame; // 父坐标系
    tfs.header.stamp = odom.header.stamp;  // 时间戳
    tfs.child_frame_id = child_frame;  // 子坐标系

    //坐标系相对信息设置
    tfs.transform.translation.x = last_received_pose.position.x;
    tfs.transform.translation.y = last_received_pose.position.y;
    tfs.transform.translation.z = last_received_pose.position.z;
    //四元素设置
    tfs.transform.rotation = last_received_pose.orientation;
    if(is_last_stamp)
    {
        odom_pub.publish(odom); //发布里程计消息
        broadcaster.sendTransform(tfs); //发布tf关系: car/base_link到 odom
    }

}

/* 
    发布里程计消息：
        base_link既不在小车的几何中心，也不在小车的后轴中心
*/


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "gazebo_odom");
    ros::NodeHandle nh, pri_nh("~");

    std::string odom_topic; //发布的里程计话题
    // 获取参数
    pri_nh.param<std::string>("odom_topic", odom_topic, "hunter_se/gazebo_odom"); // 里程计话题
    pri_nh.param<std::string>("reference_frame", reference_frame, "odom");  //里程计的参考坐标系
    pri_nh.param<std::string>("child_frame", child_frame, "base_link");  //里程计的子坐标系


    std::cout << "odom_topic: " << odom_topic<<std::endl;
    std::cout << "reference_frame: " << reference_frame<<std::endl;
    std::cout << "child_frame: " << child_frame<<std::endl;

    ros::Subscriber pose_sub = nh.subscribe("/gazebo/model_states", 10, ModelStates_callback);
    // ros::Subscriber sub = nh.subscribe("/clock", 1000, timeCallback);

    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10, true);


    ros::spin();

    return 0;
}
