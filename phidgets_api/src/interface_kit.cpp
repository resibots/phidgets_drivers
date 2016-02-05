#include "phidgets_api/interface_kit.h"

namespace phidgets {

InterfaceKit::InterfaceKit():
    Phidget(), _interface_kit_handle(0)
{
    // create the handle
    CPhidgetInterfaceKit_create(&_interface_kit_handle);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)_interface_kit_handle);

    // register base class callbacks
    // they report attach, detach and error (from the device) events
    Phidget::registerHandlers();

    // resize state vector to the number of outputs
    int output_count = 0;
    CPhidgetInterfaceKit_getOutputCount(_interface_kit_handle, &output_count);
    _digital_output_state.resize(output_count);

    // register ir data callback
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(_interface_kit_handle, output_change_callback, this);
}

bool InterfaceKit::get_state(int index)
{
    if (index < _digital_output_state.size())
        return _digital_output_state[index];
    else
        return false;
}

std::vector<bool> InterfaceKit::get_states(std::vector<int> indices)
{
    std::vector<bool> states;
    states.resize(indices.size());

    std::vector<int>::iterator index; // iterator over the indices

    for(index=indices.begin(); index!=indices.end(); index++)
    {
        states[*index] = _digital_output_state[*index];
    }

    return states;
}

void InterfaceKit::set_state(int index, bool state)
{
    CPhidgetInterfaceKit_setOutputState(_interface_kit_handle,
        index,
        (state?PTRUE:PFALSE));
}

int InterfaceKit::output_change_callback(
    CPhidgetInterfaceKitHandle interface_kit,
    void* user_data,
    int index,
    int state)
{
    // convert state value to boolean
    state = (state==PTRUE?true:false);
    ((InterfaceKit*)user_data)->output_change(index, state);
    return 0;
}

void InterfaceKit::output_change(int index, bool state)
{
    _digital_output_state[index] = state;
}

} // namespace phidgets
