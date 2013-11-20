#if !defined(_ECP_T_BLOCK_MOVE_H)
#define _ECP_T_BLOCK_MOVE_H

#include "base/ecp/ecp_task.h"
<<<<<<< HEAD
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
=======
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/get_position/ecp_g_get_position.h"

#include "../visual_servoing/visual_servo.h"
#include "../visual_servoing/single_visual_servo_manager.h"
#include "../visual_servoing/ib_eih_visual_servo.h"
#include "../visual_servoing/visual_servo_regulator_p.h"
#include "../visual_servoing/cubic_constraint.h"
#include "../visual_servoing/object_reached_termination_condition.h"
#include "../visual_servoing/timeout_termination_condition.h"
#include "../visual_servoing/IBReading.h"

#include "../visual_servoing/visual_servoing.h"
#include "../visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"
>>>>>>> 39ea5d2768963843437c746f344e0d32d87ebd2b

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
