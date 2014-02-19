/*
 * ecp_t_block_pickup.h
 *
 *  Created on: 19-02-2014
 *      Author: spiatek
 */

#ifndef ECP_T_BLOCK_PICKUP_H_
#define ECP_T_BLOCK_PICKUP_H_

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "base/ecp/ecp_task.h"

#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"

#include "application/block_move/ecp_g_block_reaching.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class ecp_t_block_pickup : public common::task::task
{
public:
	ecp_t_block_pickup(lib::configurator &_config);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_T_BLOCK_PICKUP_H_ */
