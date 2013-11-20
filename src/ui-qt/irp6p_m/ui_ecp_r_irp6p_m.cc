#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_irp6p_m.h"

namespace mrrocpp {
namespace ui {
namespace irp6p_m {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		common_012::EcpRobot(_ui_robot)
{
	ecp = new ecp::irp6p_m::robot(*(ui_robot.interface.config), *(ui_robot.msg));

	for (int j = 0; j < ecp->number_of_servos; j++) {
		MOTOR_STEP[j] = 0.05;
		JOINT_STEP[j] = 0.0004;
	}

	END_EFFECTOR_LINEAR_STEP = 0.00002;
	END_EFFECTOR_ANGULAR_STEP = 0.0002;

	init();
}

}
} //namespace ui
} //namespace mrrocpp
