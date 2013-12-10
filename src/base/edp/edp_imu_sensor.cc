#include "signal.h"
#include "unistd.h"

#include <iostream>
#include <exception>

#include "edp_typedefs.h"
#include "edp_e_manip.h"
#include "base/lib/mis_fun.h"
#include "reader.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "edp_imu_sensor.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace sensor {

void imu::operator()()
{
	if (!master.robot_test_mode) {
		lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 1);
	}

	try {
		if (!imu_sensor_test_mode) {
			connect_to_hardware();
		}

		thread_started.command();
		//	first_measure_synchroniser.command();
	}

	catch (lib::exception::se_sensor & error) {
		std::cerr << "sensor_error w imu thread EDP" << std::endl;

		uint64_t error0 = 0;

		if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
			error0 = *tmp;
		}

		switch (error0)
		{
			case SENSOR_NOT_CONFIGURED:
				//		from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				//		from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, error0);

	}

	catch (std::exception & e) {
		printf("imu sensor exception: %s\n", e.what());
		sr_msg->message(lib::FATAL_ERROR, e.what());
		exit(EXIT_SUCCESS);
	}

	catch (...) {
		std::cerr << "unidentified error imu thread w EDP" << std::endl;
	}

	while (!boost::this_thread::interruption_requested()) {

		try {
			if (new_edp_command) {
				//sr_msg->message("new_edp_command");
				boost::mutex::scoped_lock lock(mtx);
				// TODO: this should be handled with boost::bind functor parameter
				switch (command)
				{
					case (common::IMU_CONFIGURE):
						configure_sensor();
						break;
					default:
						break;
				}
				set_command_execution_finish();
			} else {

				//sr_msg->message("else");
				wait_for_event();
				//	sr_msg->message("za wait_for_event");
				get_reading();
				//	sr_msg->message("za get_reading");
				first_measure_synchroniser.command();
			}

		} //!< koniec TRY

		catch (lib::exception::se_sensor & error) {
			std::cerr << "sensor_error w imu thread EDP" << std::endl;

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			switch (error0)
			{
				case SENSOR_NOT_CONFIGURED:
					//	from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
					break;
				case READING_NOT_READY:
					//	from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
					break;
			}
			sr_msg->message(lib::FATAL_ERROR, error0);

		} catch (std::runtime_error & e) {
			std::cerr << "std runtime_error in EDP imu thread" << std::endl;
			sr_msg->message(lib::SYSTEM_ERROR, "std runtime_error in EDP imu thread: see console");
			raise(SIGUSR2);

		} catch (...) {
			std::cerr << "unidentified error in EDP imu thread" << std::endl;
		}

		//	sr_msg->message("imu operator() in while");
	}
	sr_msg->message("imu operator() interruption_requested");
}

/**************************** inicjacja czujnika ****************************/
void imu::configure_sensor(void)
{ // by Y

	//  printf("edp Sensor configured\n");
	sr_msg->message("IMU sensor being configured");

	if (!imu_sensor_test_mode) {
		configure_particular_sensor();
	}

}

imu::imu(common::manip_effector &_master) :
		imu_sensor_test_mode(true), imu_buffer_length(1), master(_master), new_edp_command(false)
{

	sr_msg =
			boost::shared_ptr <lib::sr_vsp>(new lib::sr_vsp(lib::EDP, "i_" + master.config.robot_name, master.config.get_sr_attach_point()));

	sr_msg->message("imu constructor");

	if (master.config.exists(common::IMU_BUFFER_LENGTH)) {
		imu_buffer_length = master.config.value <int>(common::IMU_BUFFER_LENGTH);
	}

	cb = new boost::circular_buffer <lib::Xyz_Angle_Axis_vector>(imu_buffer_length);

	if (master.config.exists(common::IMU_SENSOR_TEST_MODE)) {
		imu_sensor_test_mode = master.config.exists_and_true(common::IMU_SENSOR_TEST_MODE);
	}

	if (imu_sensor_test_mode) {
		sr_msg->message("IMU sensor test mode activated");
	}

	lib::Xyz_Angle_Axis_vector zero_data;

	cb->clear();

	for (int i = 0; i < imu_buffer_length; i++) {
		cb->push_back(zero_data);
	}

}

imu::~imu()
{
	sr_msg->message("~imu destructor");
}

void imu::wait_for_event()
{
	if (!imu_sensor_test_mode) {

		wait_for_particular_event();

	} else {
		usleep(1000);
	}
}

/***************************** odczyt z czujnika *****************************/
void imu::get_reading(void)
{
	if (!imu_sensor_test_mode) {
		get_particular_reading();

	} else {

	}

	lib::Xyz_Angle_Axis_vector imu_acc;
	imu_acc[0] = ldata.linearAcceleration[0];
	imu_acc[1] = ldata.linearAcceleration[1];
	imu_acc[2] = ldata.linearAcceleration[2];
	imu_acc[3] = ldata.angularAcceleration[0];
	imu_acc[4] = ldata.angularAcceleration[1];
	imu_acc[5] = ldata.angularAcceleration[2];

	// dodanie nowej sily do bufora dla celow usredniania (filtracji dolnoprzepustowej)
	cb->push_back(imu_acc);

	lib::Xyz_Angle_Axis_vector imu_out;
	// usredniamy za FORCE_BUFFER_LENGHT pomiarow
	for (int j = 0; j < 6; j++) {
		imu_out[j] = 0.0;
	}

	for (int i = 0; i < imu_buffer_length; i++) {
		for (int j = 0; j < 6; j++) {
			imu_out[j] += ((*cb)[i][j]) / (double) imu_buffer_length;
		}
	}

	master.imu_acc_dp.write(imu_out);

}

void imu::set_command_execution_finish() // podniesienie semafora
{
	new_edp_command = false;
	new_command_synchroniser.command();
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
