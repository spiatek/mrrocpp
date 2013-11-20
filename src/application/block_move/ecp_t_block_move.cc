#include "ecp_t_block_move.h"

<<<<<<< HEAD
=======
#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"

//#include "subtask/ecp_mp_st_gripper_opening.h"

#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_multiple_position.h"

#include "sensor/discode/discode_sensor.h"
#include "../visual_servoing/visual_servoing.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
//#include "robot/irp6p_tfg/const_irp6p_tfg.h"

#define BOARD_COLOR 5

#define BLOCK_WIDTH 0.0325
#define BLOCK_HEIGHT 0.0193

#define BLOCK_REACHING 0
#define BUILDING 1

using namespace mrrocpp::ecp_mp::sensor::discode;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using namespace std;

>>>>>>> 39ea5d2768963843437c746f344e0d32d87ebd2b
namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
block_move::block_move(lib::configurator &_config) :
		common::task::task(_config)
{
	if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		throw std::runtime_error("Robot not supported");
	}

	logger::log_dbg_enabled = true;

	// utworzenie generatorow do uruchamiania dispatcherem
	register_generator(new common::generator::tff_gripper_approach(*this, 8));
	register_generator(new common::generator::bias_edp_force(*this));
	register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_JOINT, ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, true));
	register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, true));
	register_generator(new common::generator::position_board(*this));
	register_generator(new common::generator::block_reaching(*this));

	sr_ecp_msg->message("ecp BLOCK MOVE loaded");
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::block_move(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
