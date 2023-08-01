#include <ros/ros.h>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "param_test_node");
//     ros::NodeHandle nh;
//     int parameter1, parameter2;
//     std::string parameter3, parameter4, parameter5;

//     // 两种方式加载全局参数
//     bool ifget1 = ros::param::get("param1", parameter1);
//     bool ifget2 = ros::param::get("/param1", parameter2);

//     if (ifget1)
//         ROS_INFO("Get param1 = %d", parameter1);
//     else
//         ROS_WARN("Didn't retrieve param1");
//     if (ifget2)
//         ROS_INFO("Get param1 = %d", parameter1);
//     else
//         ROS_WARN("Didn't retrieve param1");

//     // 两种方式加载局部参数
//     bool ifget3 = ros::param::get("name", parameter3);
//     bool ifget4 = ros::param::get("/param_node/name", parameter4);
//     bool ifget5 = ros::param::get("/name", parameter5); // 补充测试

//     if (ifget3)
//         ROS_INFO("Get name = %s", parameter3.c_str());
//     else
//         ROS_WARN("Didn't retrieve name");
//     if (ifget4)
//         ROS_INFO("Get name = %s", parameter4.c_str());
//     else
//         ROS_WARN("Didn't retrieve name");
//     if (ifget5)
//         ROS_INFO("Get name = %s", parameter5.c_str());
//     else
//         ROS_WARN("Didn't retrieve name");

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "param_test_node");
    ros::NodeHandle nh;
    int parameter1, parameter2, parameter3;

    // ① ros::param::get()获取参数“param1”的value，写入到parameter1上
    bool ifget1 = ros::param::get("param1", parameter1);

    // ② ros::NodeHandle::getParam()获取参数，与①作用相同
    bool ifget2 = nh.getParam("param2", parameter2);

    // ③ ros::NodeHandle::param()类似于①和②, 如果get不到指定的param，设置为默认值(如33333)
    nh.param("param3", parameter3, 33333);
    // parameter3 = nh.param("param3", 33333); //这两种方式都可以获取参数

    if (ifget1)
        ROS_INFO("Get param1 = %d", parameter1);
    else
        ROS_WARN("Didn't retrieve param1");
    if (ifget2)
        ROS_INFO("Get param2 = %d", parameter2);
    else
        ROS_WARN("Didn't retrieve param2");
    if (nh.hasParam("param3"))
        ROS_INFO("Get param3 = %d", parameter3);
    else
        ROS_WARN("Didn't retrieve param3");

}
