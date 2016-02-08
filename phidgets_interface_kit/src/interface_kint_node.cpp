#include <phidgets_interface_kit/interface_kit_node.h>

namespace phidgets {

    InterfaceKitNode::InterfaceKitNode(ros::NodeHandle nh):
        _nh(nh)
    {
        device.open(344136);

        int result = device.waitForAttachment(1000);
        if (result)
        {
            const char *error_description;
            CPhidget_getErrorDescription(result, &error_description);
            ROS_FATAL_STREAM("The following issue arised while waiting to"
                "attach the interface kit: " << error_description);
        }

        // declare ROS services
    }

}