/*
 * ecp_g_reach_already_localized_block.cc
 *
 *  Created on: 12-01-2014
 *      Author: spiatek
 */

#include <vector>
#include <iostream>

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include "ecp_g_reach_already_localized_block.h"

using namespace boost::interprocess;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

reach_already_localized_block::reach_already_localized_block(task::task & _ecp_t) :
		common::generator::newsmooth(_ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 7)
{
	generator_name = ecp_mp::generator::ECP_GEN_REACH_ALREADY_LOCALIZED_BLOCK;
}

void reach_already_localized_block::conditional_execution() {

	sr_ecp_msg.message("executing reach_already_localized_block generator...");

	ecp_ralb_config_section_name = "[ecp_reached_already_localized_block]";
	std::string robot_name = ecp_t.config.value <std::string>("robot_name", ecp_ralb_config_section_name);

	int coordinates_num;
	if(robot_name == lib::irp6p_m::ROBOT_NAME) {
		coordinates_num = 6;
	}
	else if(robot_name == lib::irp6ot_m::ROBOT_NAME) {
		coordinates_num = 7;
	}
	else {
		throw std::runtime_error("ECP GENERATOR: reach_already_localized_block unknown robot");
	}
	std::vector <double> coordinates_vector(coordinates_num);

	reset();
	set_absolute();

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

	int object_int = objects_array[0];
	int index = ((object_int - 1) * 7);
	for(size_t i = 0; i < 6; ++i) {
		coordinates_vector[i] = coordinates_array[index];
		index++;
	}

	load_absolute_angle_axis_trajectory_pose(coordinates_vector);

	if(calculate_interpolate()) {
		Move();
	}
}

}
}
}
}
