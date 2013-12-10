/*
 * ecp_g_conveyor_sinus.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_CONVEYOR_SINUS_H_
#define ECP_G_CONVEYOR_SINUS_H_

#include <string>

#include "base/ecp/ecp_generator.h"

#include "base/lib/logger_client/logger_client.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class ecp_g_conveyor_sinus : public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_conveyor_sinus(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	virtual ~ecp_g_conveyor_sinus();

	bool first_step();
	bool next_step();

	boost::shared_ptr<logger::logger_client> log_client;
private:

	int motion_steps;
	double dt;
	double A;
	double f;
	double t;

	bool initial_position_saved;
	double initial_position;

	logger::log_message msg;
};

/** @} */

}//namespace

}

}

}

#endif /* ECP_G_CONVEYOR_SINUS_H_ */
