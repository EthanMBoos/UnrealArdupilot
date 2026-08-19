// UnrealBuildTool does not consume the CMake target. Compile the same source
// files into the plugin so Unreal and the headless CLI share one force model.
#include "fixed_wing.cpp"
#include "geodesy.cpp"
#include "rigid_body.cpp"
