#include <phidgets_interface_kit/interface_kit_node.h>

namespace phidgets {

    InterfaceKitNode::InterfaceKitNode(ros::NodeHandle nh):
        _nh(nh)
    {
        // Open connection to a 4-relay interface kit
        _device.open(388136);

        // Wait for the device to be attached to an USB port, or fail
        int result = _device.waitForAttachment(10000);
        if (result)
        {
            const char *error_description;
            CPhidget_getErrorDescription(result, &error_description);
            ROS_FATAL_STREAM("The following issue arised while waiting to"
                "attach the interface kit: " << error_description << std::endl
                << "You need to restart the node to try again.");
            return;
        }

        // Advertise ROS services
        _get_state_service = _nh.advertiseService("get_state",
            &InterfaceKitNode::get_state, this);
        _get_states_service = _nh.advertiseService("get_states",
            &InterfaceKitNode::get_states, this);

        _set_state_service = _nh.advertiseService("set_state",
            &InterfaceKitNode::set_state, this);
        _set_states_service = _nh.advertiseService("set_states",
            &InterfaceKitNode::set_states, this);

    }

    bool InterfaceKitNode::get_state(
        phidgets_interface_kit::getState::Request &req,
        phidgets_interface_kit::getState::Response &res)
    {
        res.state = _device.get_state(req.index);
        ROS_INFO_STREAM("Asking state for " << req.index << ", it is " << _device.get_state(req.index));
        return true;
    }

    bool InterfaceKitNode::get_states(
        phidgets_interface_kit::getStates::Request &req,
        phidgets_interface_kit::getStates::Response &res)
    {
        std::vector<int> indices(req.indices.begin(), req.indices.end());
        std::vector<bool> b_states = _device.get_states(indices);
        std::vector<uint8_t> ui_states(b_states.begin(), b_states.end());
        res.states = ui_states;
        return true;
    }

    bool InterfaceKitNode::set_state(
        phidgets_interface_kit::setState::Request &req,
        phidgets_interface_kit::setState::Response &res)
    {
        _device.set_state(req.index, req.state);
        return true;
    }

    bool InterfaceKitNode::set_states(
        phidgets_interface_kit::setStates::Request &req,
        phidgets_interface_kit::setStates::Response &res)
    {
        std::vector<int> indices(req.indices.begin(), req.indices.end());
        std::vector<bool> states(req.states.begin(), req.states.end());
        _device.set_states(indices, states);
        return true;
    }

}