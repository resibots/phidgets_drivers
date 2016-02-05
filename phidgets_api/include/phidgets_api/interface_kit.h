#ifndef PHIDGETS_API_INTERFACE_KIT_H
#define PHIDGETS_API_INTERFACE_KIT_H

#include "phidgets_api/phidget.h"

#include <vector>
#include <map>

namespace phidgets {

    class InterfaceKit: public Phidget
    {
    public:

        InterfaceKit();
        ~InterfaceKit();

        // set the output state of one or more chanels
        bool get_state(int index);
        std::vector<bool> get_states(std::vector<int> indices);

        // get the state of one or more (digital) channels
        void set_state(int index, bool state);
        void set_states(std::map<int, bool> states);

    protected:

        CPhidgetInterfaceKitHandle _interface_kit_handle;

        // store the state of each output, indexed by there index (as provided by Phidgets)
        std::vector<bool> _digital_output_state;

        void output_change(int index, bool state);

    private:

        static int output_change_callback(
            CPhidgetInterfaceKitHandle interface_kit,
            void* user_data,
            int index,
            int state);
    };

} //namespace phidgets

#endif // PHIDGETS_API_INTERFACE_KIT_H
