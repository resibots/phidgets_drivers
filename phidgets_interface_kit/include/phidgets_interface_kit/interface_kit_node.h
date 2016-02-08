#ifndef PHIDGETS_INTERFACE_KIT_ROS_H
#define PHIDGETS_INTERFACE_KIT_ROS_H

#include <ros/ros.h>
#include <phidgets_api/interface_kit.h>

#include "phidgets_interface_kit/getState.h"
#include "phidgets_interface_kit/getStates.h"
#include "phidgets_interface_kit/setState.h"
#include "phidgets_interface_kit/setStates.h"

namespace phidgets {

    /** Class for the node interfaceing the Phidget's interface kit to ROS.
        Our use being limited to the 4 relays interface board, we don't
        implement the rest of phidget's API for interface kits.

        As a result, the contructor will only attemt to connect with devices having
        the serial number 388136, corresponding to the 4 relays board.
    **/
    class InterfaceKitNode
    {
    public:
        InterfaceKitNode(ros::NodeHandle nh);

        // Callbacks for ros services to retrieve and define digital outputs'
        // states
        bool get_state(phidgets_interface_kit::getState::Request &req,
                       phidgets_interface_kit::getState::Response &res);
        bool get_states(phidgets_interface_kit::getStates::Request &req,
                       phidgets_interface_kit::getStates::Response &res);

        bool set_state(phidgets_interface_kit::setState::Request &req,
                       phidgets_interface_kit::setState::Response &res);
        bool set_states(phidgets_interface_kit::setStates::Request &req,
                       phidgets_interface_kit::setStates::Response &res);

    private:
        // Not implemented, to prevent use of default methods
        InterfaceKitNode(const InterfaceKitNode& other);
        InterfaceKitNode(InterfaceKitNode& other);
        InterfaceKitNode& operator=(const InterfaceKitNode& other);
        InterfaceKitNode& operator=(InterfaceKitNode& other);

        ros::NodeHandle _nh;

        // the object used to talk to the interface kit
        InterfaceKit _device;

        // Handles for the service advertisement
        // we need to hold them in the class because the service is unadvertised
        // as soon as they are destroyed
        ros::ServiceServer _get_state_service;
        ros::ServiceServer _get_states_service;

        ros::ServiceServer _set_state_service;
        ros::ServiceServer _set_states_service;

    };

}

#endif // PHIDGETS_INTERFACE_KIT_ROS_H