#ifndef PHIDGETS_API_INTERFACE_KIT_H
#define PHIDGETS_API_INTERFACE_KIT_H

#include "phidgets_api/phidget.h"

#include "ros/ros.h"

#include <vector>
#include <map>

namespace phidgets {

    class InterfaceKit: public Phidget
    {
    public:

        InterfaceKit();

        // Get the output state of one or more chanels
        bool get_state(int index);
        std::vector<bool> get_states(std::vector<int> &indices);
        // to be compatible with the types in ROS messages and services
        std::vector<uint16_t> get_states(std::vector<uint16_t> &indices);

        // Set the state of one or more (digital) channels
        void set_state(int index, bool state);
        void set_states(const std::vector<int> &indices, const std::vector<bool> &states);
        // to be compatible with the types in ROS messages and services
        void set_states(const std::vector<uint16_t> &indices, const std::vector<uint16_t> &states);

    protected:

        CPhidgetInterfaceKitHandle _interface_kit_handle;

        virtual void attachHandler();

        // store the state of each output
        std::vector<bool> _digital_output_state;
        // called by _output_change_callback to update _digital_output_state
        void _update_output_state(int index, bool state);

    private:

        static int _output_change_callback(
            CPhidgetInterfaceKitHandle interface_kit,
            void* user_data,
            int index,
            int state);
    };

} //namespace phidgets

#endif // PHIDGETS_API_INTERFACE_KIT_H
