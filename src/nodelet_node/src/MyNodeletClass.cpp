#include <pluginlib/class_list_macros.h> //我们使用了一个名为 "PLUGINLIB_EXPORT_CLASS" 的宏来注册我们的插件。这个宏来自于 "pluginlib/class_list_macros.h" 头文件

// 它允许我们将插件注册到 ROS 的插件库中，使得 ROS 可以在运行时动态加载插件

// Include your header
#include <nodelet_node/MyNodeletClass.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(nodelet_node::MyNodeletClass, nodelet::Nodelet) // nodelet_node::MyNodeletClass插件，继承自 nodelet::Nodelet基类

namespace nodelet_node
{
    void MyNodeletClass::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
    }
}