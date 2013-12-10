/*!
 * @file
 * @brief File contains edge_follow_mr mp_task class definition of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include <iostream>
#include <sstream>

#include "base/mp/mp_task.h"

#include "mp_t_imu.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/irp6_tfg/dp_tfg.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
//#include "generator/ecp/ecp_mp_g_tfg.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new imu(_config);
}

imu::imu(lib::configurator &_config) :
		task(_config)
{
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void imu::create_robots()
{

	ACTIVATE_MP_ROBOT(irp6ot_m);

	ACTIVATE_MP_ROBOT(irp6p_m);
	sr_ecp_msg->message("MP IMU LOADED");

}

void imu::main_task_algorithm(void)
{

	sr_ecp_msg->message("MP IMU STARTED");

	// wybor manipulatora do sterowania na podstawie konfiguracji

	lib::robot_name_t manipulator_name;

	// ROBOT IRP6_ON_TRACK
	if (config.exists_and_true("is_active", "[edp_irp6ot_m]")) {
		manipulator_name = lib::irp6ot_m::ROBOT_NAME;

	} else if (config.exists_and_true("is_active", "[edp_irp6p_m]")) {
		manipulator_name = lib::irp6p_m::ROBOT_NAME;

	} else {
		// TODO: throw
	}

	// sekwencja generator na wybranym chwytaku

	char tmp_string[lib::MP_2_ECP_SERIALIZED_DATA_SIZE];

	lib::irp6_tfg::mp_to_ecp_parameters mp_ecp_command;

	mp_ecp_command.desired_position = 0.078;

	memcpy(tmp_string, &mp_ecp_command, sizeof(mp_ecp_command));
	/*

	 set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFG, (int) 5, tmp_string, sizeof(mp_ecp_command), gripper_name);

	 wait_for_task_termination(false, 1, gripper_name.c_str());

	 */

	// sekwencja generator na wybranym manipulatorze
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, (int) 5, "", manipulator_name);

	wait_for_task_termination(false, manipulator_name);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) 0, "", manipulator_name);

	wait_for_task_termination(false, manipulator_name);

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
