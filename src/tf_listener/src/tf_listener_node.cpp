#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener_test_node");

    /*
    一旦创建了监听器，它就开始通过线路接收tf转换，没有缓存器缓存时间，将会警告
    "turtle2" passed to lookupTransform argument target_frame does not exist.
    需要将其缓冲10秒。
    */

    tf::TransformListener listener;
    // ros::Duration(10.0).sleep();    // 可设置等待10s缓冲

    // rardar_pipline radar;
    ros::NodeHandle node; // 节点名称
    ros::Rate rate(10.0); // 循环速率
    while (node.ok())
    {
        // 创建一个StampedTransform对象存储变换结果数据
        tf::StampedTransform transform;

        // 监听包装在一个try-catch块中以捕获可能的异常
        try
        {
            /*
            向侦听器查询特定的转换，(想得到/link2到/link1的变换)，
            想要转换的时间ros::Time(0)提供了最新的可用转换。
            */

            listener.lookupTransform("/link1", "/link2",
                                     ros::Time(0), transform);
            printf("get tf success\n");
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        printf("get tf success x is %f , y is %f , z is %f\n",
               transform.getOrigin().x(),
               transform.getOrigin().y(),
               transform.getOrigin().z());
               
        rate.sleep();
    }
    ros::spin();

    return 0;
}