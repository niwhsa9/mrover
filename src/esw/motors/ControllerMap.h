#ifndef CONTROLLER_MAP_H
#define CONTROLLER_MAP_H

#include "Controller.h"

// #include <fstream>
// #include <stdint.h>
// #include <string>

#include <unordered_map> // for unordered_map

/*
The ControllerMap class creates a hash table of virtual Controller objects from the config file
located at "config/motors.yaml". These virtual Controllers are used to contact the physical
controller on the rover, across both RA/SA configurations.
*/
class ControllerMap {
public:
    // Map of virtual controller names to virtual Controller objects
    inline static std::unordered_map<std::string, Controller*> controllersByName;

    // REQUIRES: root is created from calling ros::param::get("motors/controllers", root)
    // MODIFIES: controllersByName
    // EFFECTS: Creates all the controller objects based on data in root.
    static void init(XmlRpc::XmlRpcValue& root);
};

#endif