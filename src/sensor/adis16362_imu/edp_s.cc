/*!
 * @file
 * @brief File containing methods of the adis16362_imu imu sensor class.
 *
 * @author yoyek
 *
 */

#include <cstdio>
#include <exception>
#include <ctime>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"

#include "base/lib/sr/srlib.h"
#include "edp_s.h"

#include "base/edp/edp_e_manip.h"
// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

adis16362_imu::adis16362_imu(common::manip_effector &_master) :
		imu(_master)
{
	printf("ADIS16362 IMU created !!! \n");
	sr_msg->message("ADIS16362 IMU sensor being created");
}

void adis16362_imu::connect_to_hardware(void)
{
	std::string device_path;
	if (master.config.exists("imu_device_path")) {
		device_path = master.config.value <std::string>("imu_device_path");
		std::cout << "imu_device_path" << device_path << std::endl;
		ai = new IMU(device_path);
	} else {
		std::cout << "imu_device_path not set" << std::endl;
	}

}

adis16362_imu::~adis16362_imu(void)
{
	if (!imu_sensor_test_mode) {
		disconnect_from_hardware();
	}
	printf("Destruktor ADIS16362 IMU sensor\n");
}

void adis16362_imu::disconnect_from_hardware(void)
{
	delete ai;
}

void adis16362_imu::configure_particular_sensor(void)
{
	sr_msg->message("ADIS16362 IMU sensor being configured");
}

void adis16362_imu::wait_for_particular_event()
{
//	usleep(1000);
}

void adis16362_imu::get_particular_reading(void)
{
	static int i = 1;

	ldata = ai->getReading();
/*
	if (((i++) % 1000) == 0) {
		std::cout << "adis imu:" << i << std::endl;
	}
*/
//	std::cout << "Ang:\n" << ldata.angularVelocity << "\nLin:\n" << ldata.linearAcceleration << "\n";
}

imu* return_created_edp_imu_sensor(common::manip_effector &_master)
{
	return new adis16362_imu(_master);
}

/*****************************  *****************************/

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

