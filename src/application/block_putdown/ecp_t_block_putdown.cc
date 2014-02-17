/*
 * ecp_t_block_putdown.cc
 *
 *  Created on: 17-02-2014
 *      Author: spiatek
 */

#include "ecp_t_block_putdown.h"

#include "base/lib/logger.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_t_block_putdown::ecp_t_block_putdown(lib::configurator &_config) :
	common::task::task(_config) {

	if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	}
	else {
		throw std::runtime_error("ECP: Robot not supported");
	}

	logger::log_dbg_enabled = true;

	register_generator(new common::generator::tff_gripper_approach(*this, 8));
	register_generator(new common::generator::smooth_file_from_mp(*this, lib::ECP_JOINT, ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, true));
	register_generator(new common::generator::smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, true));
	register_generator(new common::generator::position_board(*this));

	sr_ecp_msg->message("ecp BLOCK PUTDOWN loaded");
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::ecp_t_block_putdown(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
