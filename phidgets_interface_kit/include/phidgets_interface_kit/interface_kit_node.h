#ifndef PHIDGETS_INTERFACE_KIT_ROS_H
#define PHIDGETS_INTERFACE_KIT_ROS_H

#include <ros/ros.h>
#include <phidgets_api/interface_kit.h>

namespace phidgets {

    class InterfaceKitNode
    {
    public:
        InterfaceKitNode(ros::NodeHandle nh);
        ~InterfaceKitNode();

        // define callbacks for ros services
        

    private:
        // not implemented
        InterfaceKitNode(const InterfaceKitNode& other);
        InterfaceKitNode(InterfaceKitNode&& other);
        InterfaceKitNode& operator=(const InterfaceKitNode& other);
        InterfaceKitNode& operator=(InterfaceKitNode&& other);

        ros::NodeHandle _nh;

        InterfaceKit _device;

    };

}

#endif // PHIDGETS_INTERFACE_KIT_ROS_H