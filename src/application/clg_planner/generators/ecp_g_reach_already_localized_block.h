/*
 * ecp_g_reach_already_localized_block.h
 *
 *  Created on: 12-01-2014
 *      Author: spiatek
 */

#ifndef ECP_G_REACH_ALREADY_LOCALIZED_BLOCK_H_
#define ECP_G_REACH_ALREADY_LOCALIZED_BLOCK_H_

#define POSTUMENT_INT	0
#define TRACK_INT		1

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/lib/typedefs.h"
#include "base/ecp/ecp_task.h"

#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

#include "ecp_mp_g_reach_already_localized_block.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class reach_already_localized_block : public common::generator::newsmooth
{
private:

	std::string ecp_ralb_config_section_name;

public:

	reach_already_localized_block(task::task & _ecp_t);
	void conditional_execution();
};

}
}
}
}

#endif /* ECP_G_REACH_ALREADY_LOCALIZED_BLOCK_H_ */
