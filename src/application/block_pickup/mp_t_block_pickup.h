/*
 * mp_t_block_pickup.h
 *
 *  Created on: 19-02-2014
 *      Author: spiatek
 */

#ifndef MP_T_BLOCK_PICKUP_H_
#define MP_T_BLOCK_PICKUP_H_

#include "base/ecp/ecp_task.h"
#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"
#include "base/lib/impconst.h"
#include "base/lib/configurator.h"

#include "base/ecp/ecp_generator.h"
#include "generator/ecp/get_position/ecp_g_get_position.h"

namespace mrrocpp {
namespace mp {
namespace task {

class mp_t_block_pickup : public task
{
public:
	mp_t_block_pickup(lib::configurator &_config);
	virtual ~mp_t_block_pickup();

	void create_robots();
	void main_task_algorithm();
	std::string get_trajectory_file_name(int);

private:
	std::string mp_block_pickup_string;
	lib::robot_name_t robot_name;
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_T_BLOCK_PICKUP_H_ */
