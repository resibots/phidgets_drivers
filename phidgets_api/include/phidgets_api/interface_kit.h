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

        // set the output state of one or more chanels
        bool get_state(int index);
        std::vector<bool> get_states(std::vector<int> &indices);
        // to be compatible to how ROS handle types in messages and services
        std::vector<uint16_t> get_states(std::vector<uint16_t> &indices);

        // get the state of one or more (digital) channels
        void set_state(int index, bool state);
        void set_states(std::vector<int> &indices, std::vector<bool> &states);
        // to be compatible to how ROS handle types in messages and services
        void set_states(std::vector<uint16_t> &indices, std::vector<uint16_t> &states);

    protected:

        CPhidgetInterfaceKitHandle _interface_kit_handle;

        // store the state of each output, indexed by their output index
        // provided by Phidgets
        std::vector<bool> _digital_output_state;

        void update_output_state(int index, bool state);

    private:

        static int output_change_callback(
            CPhidgetInterfaceKitHandle interface_kit,
            void* user_data,
            int index,
            int state);
    };

} //namespace phidgets

#endif // PHIDGETS_API_INTERFACE_KIT_H
