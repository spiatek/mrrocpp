#if !defined(_ECP_T_BLOCK_MOVE_H)
#define _ECP_T_BLOCK_MOVE_H

#include "base/ecp/ecp_task.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

#include "ecp_g_position_board.h"
#include "ecp_g_block_reaching.h"

#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class block_move : public common::task::task
{
public:

	block_move(lib::configurator &_config);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
