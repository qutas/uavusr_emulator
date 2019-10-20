#include <ros/ros.h>
#include <uavusr_emulator/uavusr_emulator.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "uavusr_emulator");
	uavusr_emulator::UAVUSREmulator uavusr;

	ros::spin();

	return 0;
}
