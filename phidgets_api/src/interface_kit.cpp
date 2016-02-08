#include <stdexcept>
#include <sstream>

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

    // register output state change callback
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(_interface_kit_handle, &InterfaceKit::output_change_callback, this);
}

bool InterfaceKit::get_state(int index)
{
    if (index < _digital_output_state.size())
        return _digital_output_state[index];
    else
        return false;
}

std::vector<bool> InterfaceKit::get_states(std::vector<int> &indices)
{
    std::vector<bool> states;
    states.resize(indices.size());

    std::vector<int>::iterator index; // iterator over the indices
    for(index=indices.begin(); index!=indices.end(); index++)
        states.push_back(get_state(*index));

    return states;
}

std::vector<uint16_t> InterfaceKit::get_states(std::vector<uint16_t> &indices)
{
    std::vector<uint16_t> states;
    states.resize(indices.size());

    std::vector<uint16_t>::iterator index; // iterator over the indices
    for(index=indices.begin(); index!=indices.end(); index++)
        states.push_back((uint16_t)get_state((int)*index));

    return states;
}

void InterfaceKit::set_state(int index, bool state)
{
    CPhidgetInterfaceKit_setOutputState(_interface_kit_handle,
        index,
        (state?PTRUE:PFALSE));
}

void InterfaceKit::set_states(std::vector<int> &indices, std::vector<bool> &states)
{
    if (indices.size() != states.size())
    {
        std::stringstream message;
        message << "indices and states vector have different size:"
            "respectfully " << indices.size() << " and " << states.size();
        throw std::runtime_error(message.str());
    }

    for (int i=0; i<indices.size(); i++)
    {
        set_state(indices[i], states[i]);
    }
}

void InterfaceKit::set_states(std::vector<uint16_t> &indices, std::vector<uint16_t> &states)
{
    if (indices.size() != states.size())
    {
        std::stringstream message;
        message << "indices and states vector have different size:"
            "respectfully " << indices.size() << " and " << states.size();
        throw std::runtime_error(message.str());
    }

    for (int i=0; i<indices.size(); i++)
    {
        set_state((int)indices[i], (bool)states[i]);
    }
}

void InterfaceKit::update_output_state(int index, bool state)
{
    _digital_output_state[index] = state;
}

int InterfaceKit::output_change_callback(
    CPhidgetInterfaceKitHandle interface_kit,
    void* user_data,
    int index,
    int state)
{
    // convert state value to boolean
    bool b_state = (state==PTRUE?true:false);
    ((InterfaceKit*)user_data)->update_output_state(index, b_state);
    return 0;
}

} // namespace phidgets
