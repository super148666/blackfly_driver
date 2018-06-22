#ifndef NODE_MAP_INFO_H
#define NODE_MAP_INFO_H
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>

#define MAX_CHARS 35
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
// Use the following enum and global constant to select whether nodes are read
// as 'value' nodes or their individual types.
enum readType
{
        VALUE,
        INDIVIDUAL
};
const readType chosenRead = VALUE;
string indent(unsigned int level);
int printValueNode(CNodePtr node, unsigned int level);
int printStringNode(CNodePtr node, unsigned int level);
int printIntegerNode(CNodePtr node, unsigned int level);
int printFloatNode(CNodePtr node, unsigned int level);
int printBooleanNode(CNodePtr node, unsigned int level);
int printCommandNode(CNodePtr node, unsigned int level);
int printEnumerationNodeAndCurrentEntry(CNodePtr node, unsigned int level);
int printCategoryNodeAndAllFeatures(CNodePtr node, unsigned int level);
int RunSingleCamera(CameraPtr cam);



#endif
