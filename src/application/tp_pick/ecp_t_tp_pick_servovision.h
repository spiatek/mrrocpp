/*
 * ecp_t_tp_pick_servovision.h
 *      Author: tpokorsk
 */

#ifndef ECP_T_TP_PICK_SERVOVISION_H_
#define ECP_T_TP_PICK_SERVOVISION_H_

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "base/lib/logger.h"
#include "application/visual_servoing/visual_servoing.h"
#include "application/visual_servoing/object_reached_termination_condition.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "sensor/discode/discode_sensor.h"

#include "base/ecp/ecp_task.h"
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/get_position/ecp_g_get_position.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/** @addtogroup servovision
 *  @{
 */

class ecp_t_tp_pick_servovision : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_tp_pick_servovision(mrrocpp::lib::configurator& config);

	void mp_2_ecp_next_state_string_handler(void);
	//void main_task_algorithm(void);

protected:
	//common::generator::tff_gripper_approach* gtga;
	common::generator::newsmooth* sg;
	common::generator::get_position* gp;

	boost::shared_ptr<visual_servo_regulator> reg;
	boost::shared_ptr<single_visual_servo_manager> sm;
	boost::shared_ptr<visual_servo> vs;

	boost::shared_ptr<termination_condition> object_reached_term_cond;

	std::vector <double> position_vector[7];
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_PB_H_ */
