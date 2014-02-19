/*
 * mp_t_block_putdown.cc
 *
 *  Created on: 17-02-2014
 *      Author: spiatek
 */

#include <iostream>

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "application/block_move/ecp_mp_g_position_board.h"

#include "base/lib/logger.h"

#include "mp_t_block_putdown.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_t_block_putdown(_config);
}

mp_t_block_putdown::mp_t_block_putdown(lib::configurator &_config) :
		task(_config)
{
}

mp_t_block_putdown::~mp_t_block_putdown()
{
}

void mp_t_block_putdown::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6ot_m);

	int name = config.value <int>("robot_name", lib::MP_SECTION);
	if(name == 1) {
		robot_name = lib::irp6ot_m::ROBOT_NAME;
	}
	else if(name == 2) {
		robot_name = lib::irp6p_m::ROBOT_NAME;
	}
	else {
		throw std::runtime_error("MP: Robot not supported");
	}
}

void mp_t_block_putdown::main_task_algorithm()
{
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/pos_build_start_track.trj", robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	sr_ecp_msg->message("mp_t_clg_planner::move() - basic move done");

	mp_block_putdown_string = "[block_putdown]";
	int position_int = config.value <int>("position_int", mp_block_putdown_string);
	std::string action = config.value <std::string>("action", mp_block_putdown_string);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_POSITION_BOARD, position_int, "", robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	sr_ecp_msg->message("mp_t_clg_planner::move() - additional move done");

	if(action == "sinew") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - rotation");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_rotate.trj", robot_name);
		wait_for_task_termination(false, robot_name);
	}
	else if(action == "douew") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - rotation");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_rotate_double.trj", robot_name);
		wait_for_task_termination(false, robot_name);
	}

	if(action == "douew") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition EW single");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_sintr.trj", robot_name);
	}
	else if(action == "douns") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition NS single");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ns_sintr.trj", robot_name);
	}
	else if(action == "triew") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition EW double");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_doutr.trj", robot_name);
	}
	else if(action == "sinns") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition EW double");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ns_tr.trj", robot_name);
	}
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.01, 300, 1), robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	sr_ecp_msg->message("mp_t_clg_planner::move() - tff gripper approach done");

	if(action == "douns") {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/build_double.trj", lib::irp6ot_m::ROBOT_NAME);
	}
	else if(action == "douew") {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/build_double_ns.trj", lib::irp6ot_m::ROBOT_NAME);
	}
	else if(action == "sinns") {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/build_single_ns.trj", lib::irp6ot_m::ROBOT_NAME);
	}
	else {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/build.trj", lib::irp6ot_m::ROBOT_NAME);
	}
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	wait_ms(1000);

	sr_ecp_msg->message("Force approach");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.01, 600, 1), robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	sr_ecp_msg->message("Raising up...");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/up_after_pushing.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("mp_t_clg_planner::pickup() - block putdown done");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
