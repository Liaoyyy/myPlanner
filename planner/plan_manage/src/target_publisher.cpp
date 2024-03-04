#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

bool enable_data_send=0;

void reachTargetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("enable next send");
    if(msg->data==true)
    {
        enable_data_send=1;
    }
}

struct Point
{
    Point(float x_, float y_):x(x_),y(y_) {} 
    float x,y;
};

//存储目标地点数据集
std::vector<Point> pointSet{Point(0,0),Point(5,0),Point(5,5),Point(0,5),Point(0,0)};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_publisher");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(1);
    int init_flag=1;
    int pub_times=3;

    //创建节点发布目标地址信息
    ros::Publisher target_pub_=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::Subscriber get_target=nh.subscribe<std_msgs::Bool>("reach_target_feedback",1,&reachTargetCallback);

    //初始化
    std::vector<geometry_msgs::PoseStamped> target_vec_;
    for(auto it:pointSet)
    {
        geometry_msgs::PoseStamped pos;
        pos.pose.position.x=it.x;
        pos.pose.position.y=it.y;

        target_vec_.push_back(pos);
    }

    while(ros::ok())
    {
        auto iter = target_vec_.begin();
        geometry_msgs::PoseStamped cur_target_;
        if(init_flag==1)
        {
            cur_target_ = *iter;
            for(int i=0;i<pub_times;i++)
            {
                target_pub_.publish(cur_target_);
                ROS_INFO("Send Data sucessfully");
                loop_rate.sleep();
            }
            iter++;
            init_flag=0;
        }
        else
        {
            if(enable_data_send==1)
            {
                while(iter!=target_vec_.end())
                {
                    ROS_INFO("Start next data send");
                    cur_target_ = *iter;
                    for(int i=0;i<pub_times;i++)
                    {
                        target_pub_.publish(cur_target_);
                        loop_rate.sleep();
                    }
                    iter++;
                } 
                enable_data_send=0;
            }
        }
        ros::spinOnce();
    }

    return 0;
}