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

    // register output state change callback
    CPhidgetInterfaceKit_set_OnOutputChange_Handler(_interface_kit_handle, &InterfaceKit::_output_change_callback, this);
}

bool InterfaceKit::get_state(int index)
{
    if (index < _digital_output_state.size())
        return _digital_output_state[index];
    else
    {
        std::stringstream message;
        message << "Requesting state for output " << index << " which id is higher"
            " than # of outputs :" << _digital_output_state.size()
            << std::endl;
        throw std::runtime_error(message.str());
    }
}

std::vector<bool> InterfaceKit::get_states(std::vector<int> &indices)
{
    std::vector<bool> states;

    std::vector<int>::iterator index; // iterator over the indices
    for(index=indices.begin(); index!=indices.end(); index++)
        states.push_back(get_state(*index));

    return states;
}

std::vector<uint16_t> InterfaceKit::get_states(std::vector<uint16_t> &indices)
{
    std::vector<uint16_t> states;

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

void InterfaceKit::set_states(const std::vector<int> &indices, const std::vector<bool> &states)
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

void InterfaceKit::set_states(const std::vector<uint16_t> &indices, const std::vector<uint16_t> &states)
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

void InterfaceKit::attachHandler()
{
    Phidget::attachHandler();
    // resize state vector to the number of outputs
    int output_count = 0;
    CPhidgetInterfaceKit_getOutputCount(_interface_kit_handle, &output_count);
    _digital_output_state.resize(output_count);
}

void InterfaceKit::_update_output_state(int index, bool state)
{
    if (index >= _digital_output_state.size())
    {
        std::stringstream message;
        message << "state updated for output " << index << " which id is higher"
            " than declared # of outputs :" << _digital_output_state.size()
            << std::endl;
        throw std::runtime_error(message.str());
    }

    _digital_output_state[index] = state;
}

int InterfaceKit::_output_change_callback(
    CPhidgetInterfaceKitHandle interface_kit,
    void* user_data,
    int index,
    int state)
{
    // convert state value to boolean
    bool b_state = (state==PTRUE?true:false);
    ((InterfaceKit*)user_data)->_update_output_state(index, b_state);
    return 0;
}

} // namespace phidgets
