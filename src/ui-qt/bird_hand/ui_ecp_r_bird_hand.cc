#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_bird_hand.h"

namespace mrrocpp {
namespace ui {
namespace bird_hand {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		EcpRobotDataPort(_ui_robot)
{
	the_robot = (boost::shared_ptr <robot_t>) new ecp::bird_hand::robot(*(ui_robot.interface.config), *(ui_robot.msg));

	common::EcpRobot::ecp = (ecp::common::robot::common_buffers_ecp_robot*) (the_robot.get());
}

// do odczytu stanu poczatkowego robota
void EcpRobot::get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
{

//	printf("bird_hand get_controller_state \n");

// Zlecenie odczytu numeru modelu i korektora kinematyki
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = CONTROLLER_STATE_DEFINITION;

	direct_execute_motion();

	robot_controller_initial_state_l = the_robot->reply_package.controller_state;
	the_robot->synchronised = robot_controller_initial_state_l.is_synchronised;
}

}
} //namespace ui
} //namespace mrrocpp
