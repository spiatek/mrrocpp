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

	std::vector <double> coordinates_vector(6);

	reset();
	set_absolute();

	float pos;
	int mp_array[7];
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_array);

	std::cout << "REACHING ALREADY LOCALIZED BLOCK ON POSITION: ";
	for(size_t i = 0; i < 6; ++i) {
		pos = (float) ((mp_array[i]/10000.0) - 5.0);
		coordinates_vector[i] = pos;
		std::cout << pos << " ";
	}
	std::cout << std::endl;

	load_absolute_angle_axis_trajectory_pose(coordinates_vector);

	if(calculate_interpolate()) {
		Move();
	}
}

}
}
}
}
