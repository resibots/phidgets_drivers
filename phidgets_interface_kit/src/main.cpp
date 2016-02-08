#include "phidgets_interface_kit/interface_kit_node.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsIK");
  ros::NodeHandle nh;

  // Connect to the Interface Kit and advertise services
  phidgets::InterfaceKitNode interface_kit(nh);

  ros::spin();

  return 0;
}
