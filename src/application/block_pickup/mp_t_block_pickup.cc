/*
 * mp_t_block_pickup.cc
 *
 *  Created on: 19-02-2014
 *      Author: spiatek
 */

#include <iostream>

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "application/block_move/ecp_mp_g_block_reaching.h"

#include "base/lib/logger.h"

#include "mp_t_block_pickup.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_t_block_pickup(_config);
}

mp_t_block_pickup::mp_t_block_pickup(lib::configurator &_config) :
		task(_config)
{
}

mp_t_block_pickup::~mp_t_block_pickup()
{
}

void mp_t_block_pickup::create_robots()
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

void mp_t_block_pickup::main_task_algorithm()
{
	sr_ecp_msg->message("mp_t_clg_planner::pickup() - object is unknown");

	mp_block_pickup_string = "[block_pickup]";
	int color_int = config.value <int>("color_int", mp_block_pickup_string);
	int max_number_of_servo_tries = config.value <int>("max_number_of_servo_tries", mp_block_pickup_string);
	int view = config.value <int>("view", mp_block_pickup_string);

	std::string trj_file_str = get_trajectory_file_name(view);

	int flag = 0;
	for(int number_of_servo_tries = 0; number_of_servo_tries < max_number_of_servo_tries; ++number_of_servo_tries) {

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, trj_file_str, robot_name);
		wait_for_task_termination(false, robot_name);
		wait_ms(1000);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_BLOCK_REACHING, color_int, "", robot_name);
		wait_for_task_termination(false, robot_name);

		if(robot_m[robot_name]->ecp_reply_package.variant == 1) {		/* POWODZENIE W WYSZUKIWANIU */

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/move_before_pickup.trj", robot_name);
			wait_for_task_termination(false, robot_name);

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.01, 600, 3), robot_name);
			wait_for_task_termination(false, robot_name);

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/up_after_pickup.trj", robot_name);
			wait_for_task_termination(false, robot_name);

			flag = 1;
			sr_ecp_msg->message("mp_t_clg_planner::pickup() - servo computed true value");
			break;
		}
	}
	if(flag == 0) {
		throw std::runtime_error("mp_t_clg_planner::pickup() - servo failed");
	}
}

std::string mp_t_block_pickup::get_trajectory_file_name(int view) {
	std::string return_str = "";
	switch(view) {
		case 0:
			return_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_1.trj";
			break;
		case 1:
			return_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_2.trj";
			break;
		case 2:
			return_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_3.trj";
			break;
		case 3:
			return_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_4.trj";
			break;
		default:
			throw std::runtime_error("mp_t_block_pickup::get_trajectory_file_name() error");
	}
	return return_str;
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
