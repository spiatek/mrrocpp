/*
 * ecp_g_block_reaching.h
 *
 *  Created on: 2012-02-13
 *      Author: spiatek
 */

#ifndef ECP_G_BLOCK_REACHING_H_
#define ECP_G_BLOCK_REACHING_H_

#include <boost/shared_ptr.hpp>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/lib/typedefs.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_task.h"

#include "ecp_mp_g_block_reaching.h"
#include "BReading.h"

#include "sensor/discode/discode_sensor.h"

#include "base/ecp/ecp_generator.h"
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

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class block_reaching : public common::generator::generator
{
protected:

	boost::shared_ptr<common::generator::get_position> gp;

	boost::shared_ptr<common::generator::single_visual_servo_manager> sm;
	boost::shared_ptr<servovision::visual_servo> vs;
	boost::shared_ptr<servovision::visual_servo_regulator> reg;
	boost::shared_ptr<ecp_mp::sensor::discode::discode_sensor> ds, ds_rpc;

	boost::shared_ptr<servovision::termination_condition> object_reached_term_cond;
	boost::shared_ptr<servovision::termination_condition> timeout_term_cond;

	std::string ds_config_section_name;
	std::string vs_config_section_name;
	std::string ecp_br_config_section_name;

	std::vector <double> position_vector;

public:

	block_reaching(task::task & _ecp_t);
	void conditional_execution();
};

}
}
}
}

#endif /* ECP_G_BLOCK_REACHING_H_ */
