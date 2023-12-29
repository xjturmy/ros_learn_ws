#include <nodelet/nodelet.h>

namespace nodelet_node
{

    class MyNodeletClass : public nodelet::Nodelet // 定义的类MyNodeletClass继承自nodelet::Nodelet类，
    {
    public:
        virtual void onInit(); // 函数onInit()是类nodelet::Nodelet的纯虚函数，子类必须重新实现

        // 并且manager动态加载MyNodeletClass类后，onInit()函数会自动执行。
    };

}