/*
 * ecp_t_clg_planner.h
 *
 *  Created on: 05-01-2014
 *      Author: spiatek
 */

#ifndef ECP_T_CLG_PLANNER_H_
#define ECP_T_CLG_PLANNER_H_

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "base/ecp/ecp_task.h"

#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"

#include "application/block_move/ecp_g_position_board.h"
#include "application/block_move/ecp_g_block_reaching.h"

#include "generators/ecp_g_reach_already_localized_block.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class clg_planner : public common::task::task
{
public:
	clg_planner(lib::configurator &_config);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_T_CLG_PLANNER_H_ */
