/*
 * action_manager.cc
 *
 *  Created on: 20-12-2013
 *      Author: spiatek
 */

#include <iostream>

#include "base/lib/impconst.h"
#include "base/lib/configurator.h"

#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "application/block_move/ecp_mp_g_position_board.h"
#include "application/block_move/ecp_mp_g_block_reaching.h"

#include "action_manager.h"

using namespace boost::interprocess;
using namespace mrrocpp::mp::task;

namespace mrrocpp {
namespace mp {
namespace task {

action_manager::action_manager() {
}

action_manager::~action_manager() {
}

bool action_manager::check_position(std::string start_position) {
	//TODO: run getPosition() generator
	//check if start_position is similar
	return true;
}

bool action_manager::check_object(std::string clg_object) {
	//TODO: check if clg_object is currently in use
	return true;
}

bool action_manager::check_color(std::string clg_color) {
	//TODO: check if currently processing object has clg_color
	return true;
}

/**
 * compute_position_for_position_board_generator()
 * @position_string - string received from CLG Planner
 * returns integer param for POSITION_BOARD generator
 */
int action_manager::compute_position_for_position_board_generator(std::string position_string)
{
	std::vector<int> position_to;
	if(position_to.size() == 4) {
		position_to[0] = position_string[1];
		position_to[1] = position_string[2];
		position_to[2] = position_string[3];
	}
	if(position_to.size() == 3) {
		position_to[0] = position_string[1];
		position_to[1] = position_string[2];
	}
	else {
		throw std::runtime_error("CLG::move(): bad position");
	}

	int position_int = 100 * position_to[0] + 10 * position_to[1] + position_to[2];
	return position_int;
}

/**
 * get_trajectory_file_name()
 * @robot_name - name of an active robot
 * @area_str - string specifying absolute position that robot have to reach (P - building, TABLE - searching)
 * @step - number of table view (0 for P)
 * returns name of a trajectory file which will be sent to SMOOTH_JOINT_FILE_FROM_MP generator
 */
std::string action_manager::get_trajectory_file_name(lib::robot_name_t robot_name, std::string area_str, int step)
{
	std::string trj_file_str;
	if(robot_name == lib::irp6ot_m::ROBOT_NAME) {
		if(area_str == "TABLE" && step == 0) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_1.trj";
		}
		if(area_str == "TABLE" && step == 1) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_2.trj";
		}
		else if(area_str == "P") {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_build_start_track.trj";
		}
		else {
			throw std::runtime_error("Action Manager: bad area type received from CLG Planner");
		}
	}
	else if(robot_name == lib::irp6p_m::ROBOT_NAME) {
		if(area_str == "TABLE" && step == 0) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_postument_1.trj";
		}
		if(area_str == "TABLE" && step == 1) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_postument_2.trj";
		}
		else if(area_str == "P") {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_build_start_postument.trj";
		}
		else {
			throw std::runtime_error("Action Manager: bad area type received from CLG Planner");
		}
	}
	else {
		throw std::runtime_error("MP: Robot not supported");
	}
	return trj_file_str;
}

/**
 * color_string_to_int()
 * @color_string string
 * returns integer value of string
 */
int action_manager::color_string_to_int(std::string color_string)
{
	int color_int;
	if(color_string == "BLUE" || color_string == "SINGLE_BLUE") {
		color_int = SINGLE_BLUE;
	}
	else if(color_string == "RED" || color_string == "SINGLE_RED") {
		color_int = SINGLE_RED;
	}
	else if(color_string == "GREEN" || color_string == "SINGLE_GREEN") {
		color_int = SINGLE_GREEN;
	}
	else if(color_string == "YELLOW" || color_string =="SINGLE_YELLOW") {
		color_int = SINGLE_YELLOW;
	}
	else {
		throw std::runtime_error("Action Manager: bad color received from CLG Planner");
	}
	return color_int;
}

/**
 * observe_color()
 * Search for object with specified color in an image
 * Omits objects which have already been checked
 * Uses SMOOTH_JOINT_FILE_FROM_MP generator to reach search start position
 * Uses GEN_BLOCK_REACHING generator to run visual servoing
 * Corresponding to observe_color() CLG Planner action
 * @p - reference to the parameter vector
 * 		p[0] - robot name
 * 		p[1] - color string
 * 		p[2] - object name
 * 		p[3] - return value (boolean)
 * If there is no such object in any of image areas, fills p[3] with "false"
 */
bool action_manager::observe_color(std::vector<std::string> &p)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	std::cout << "OBSERVE_COLOR PROCESS" << std::endl;

	/* preparing shared arrays */
	managed_shared_memory segment(open_only, "ClgSharedMemory");

	std::pair<int*, size_t> sm_cont_o = segment.find<int>("ObjectsArray");
	int* objects_array;
	if(sm_cont_o.second != 0) {
		objects_array = sm_cont_o.first;
	}

	std::pair<double*, size_t> sm_cont_c = segment.find<double>("CoordinatesArray");
	double* coordinates_array;
	if(sm_cont_c.second != 0) {
		coordinates_array = sm_cont_c.first;
	}

	/* computing color integer value */
	std::string color_string = p.at(1);
	int color_int = UNKNOWN;
	color_int = color_string_to_int(color_string);

	/* computing object integer value */
	std::string object_name = p.at(2);
	int object_int = atoi(object_name.substr(2).c_str());

	/* checks if color is already known and if it matches */
	if(objects_array[object_int] == color_int) {
		p.push_back("true");
	}
	/* checks if color is already known and if it doesn't match */
	else if(objects_array[object_int] != UNKNOWN) {
		p.push_back("false");
	}
	/* color is unknown */
	else {
		std::string trj_file_str;
		lib::robot_name_t robot_name = p.front();
		objects_array[0] = object_int;
		int flag = 0;
		int number_of_servo_tries;					/* DOCELOWO STALA - ile prob odpalenia serwomechanizmu */
		for(int step = 0; step < 1; ++step) {		/* DOCELOWO STALA - tyle krokow ile mozliwych obrazow startowych */
			for(number_of_servo_tries = 0; number_of_servo_tries < 5; ++number_of_servo_tries) {

				trj_file_str = get_trajectory_file_name(robot_name, "TABLE", step);

				wait_ms(1000);

				set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, trj_file_str, robot_name);
				wait_for_task_termination(false, robot_name);
				wait_ms(1000);

				set_next_ecp_state(ecp_mp::generator::ECP_GEN_BLOCK_REACHING, color_int, "", robot_name);
				wait_for_task_termination(false, robot_name);

				if(robot_m[robot_name]->ecp_reply_package.variant == 1) {		/* POWODZENIE W WYSZUKIWANIU */
					flag = 1;
					break;
				}
			}
			if(srv_i != 4 || flag == 1) {	/* SPRAWDZENIE CZY POTRZEBNE JEST WYSZUKIWANIE W INNYM WIDOKU */
				break;
			}
		}
		if(flag == 1) {
			objects_array[object_int] = color_int;
			p.push_back("true");
		}
		else {
			p.push_back("false");
		}
	}
	return true;
}

/* Niepotrzebne */
bool action_manager::observ(std::vector<std::string> &p)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	std::cout << "OBSERV PROCESS" << std::endl;
	return true;
}

/**
 * move()
 * Moves manipulator arm from one position to another
 * Uses SMOOTH_JOINT_FILE_FROM_MP to reach the table
 * Uses SMOOTH_JOINT_FILE_FROM_MP and ECP_GEN_POSITION_BOARD to reach position on board
 * Corresponding to move() CLG Planner action
 * @p - reference to the parameter vector
 * 		p[0] - robot name
 * 		p[1] - start position (it should be equal to present robot position)
 * 		p[2] - finish positions
 */
bool action_manager::move(std::vector<std::string> &p)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	std::string trj_file_str;
	lib::robot_name_t robot_name = p.front();
	std::string position_from = p.at(1);
	std::string position_to = p.at(2);

	std::cout << "MOVE PROCESS" << std::endl;

	if(check_position(position_from) == false) {
		throw std::runtime_error("Action Manager: bad position received from CLG Planner");
	}

	trj_file_str = get_trajectory_file_name(robot_name, position_to, 0);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, trj_file_str, robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	/** additional move due to reach more specified position_to **/
	if(position_to[0] == "P") {

		int position_int;
		position_int = compute_position_for_smooth_generator(position_to);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_POSITION_BOARD, position_int, "", robot_name);
		wait_for_task_termination(false, robot_name);
		wait_ms(1000);
	}

	return true;
}

/**
 * pickup()
 * Picks up specified object from specified position
 * Uses TFF_GRIPPER_APPROACH generator to go down
 * Uses REACH_ALREADY_LOCALIZED_BLOCK generator to perform a specific absolute move
 * Corresponding to pickup() CLG Planner action
 * @p - reference to the parameter vector
 * 		p[0] - robot name
 * 		p[1] - object name
 * 		p[2] - position
 */
bool action_manager::pickup(std::vector<std::string> &p)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	std::cout << "PICKUP PROCESS" << std::endl;

	/* preparing shared arrays */
	boost::interprocess::managed_shared_memory segment(open_only, "ClgSharedMemory", read_write);

	std::pair<int*, size_t> sm_cont_o = segment.find<int>("ObjectsArray");
	if(sm_cont_o.second != 0) {
		int* objects_array = sm_cont_o.first;
	}

	std::pair<double*, size_t> sm_cont_c = segment.find<double>("CoordinatesArray");
	if(sm_cont_c.second != 0) {
		double* coordinates_array = sm_cont_c.first;
	}

	lib::robot_name_t robot_name = p.front();

	std::string object_name = p.at(1);
	int object_int = atoi(object_name.substr(2).c_str());
	objects_array[0] = object_int;

	std::string position_str = p.at(2);
	if(position_str != "TABLE") {
		throw std::runtime_error("Action Manager: unknown position received from CLG Planner::pickup");
	}

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_REACH_ALREADY_LOCALIZED_BLOCK, 0, "", robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.03, 800, 3), robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/up_after_pickup.trj", robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	return true;
}

/**
 * putdown()
 * Puts currently held block on specified position
 * Uses TFF_GRIPPER_APPROACH generator to go down
 * Uses SMOOTH_ANGLE_AXIS_FILE_FROM_MP generator to perform a specific relative move
 * Corresponding to putdown() CLG Planner action
 * @p - reference to the parameter vector
 * 		p[0] - robot name
 * 		p[1] - object name
 * 		p[2] - position (it should be equal to present robot position)
 * 		p[3] - color string
 */
bool action_manager::putdown(std::vector<std::string> &p)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	std::cout << "PUTDOWN PROCESS" << std::endl;

	lib::robot_name_t robot_name = p.front();
	std::string object_name = p.at(1);
	std::string position_from = p.at(2);
	std::string color_string = p.at(3);

	if(check_position(position_from) == false) {
		throw std::runtime_error("Action Manager: bad position received from CLG Planner");
	}

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 600, 2), robot_name);
	wait_for_task_termination(false, robot_name);

	wait_ms(4000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/build.trj", robot_name);
	wait_for_task_termination(false, robot_name);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, BUILDING, "", robot_name);
	wait_for_task_termination(false, robot_name);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/up_after_pushing.trj", robot_name);
	wait_for_task_termination(false, robot_name);

	wait_ms(1000);

	return true;
}

}
}
}
