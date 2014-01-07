/*
 * mp_t_clg_planner.cc
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "base/mp/mp_robot.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

#include "clg_proxy.h"
#include "mp_t_clg_planner.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_t_clg_planner(_config);
}

mp_t_clg_planner::mp_t_clg_planner(lib::configurator &_config) :
		task(_config)
{
}

mp_t_clg_planner::~mp_t_clg_planner()
{
}

void mp_t_clg_planner::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6ot_m);

	int name = config.value <int>("robot_name", "[mp]");
	if(name == 1) {
		robot_name = lib::irp6ot_m::ROBOT_NAME;
	}
	else if(name == 2) {
		robot_name = lib::irp6p_m::ROBOT_NAME;
	}
	else {
		throw std::runtime_error("MP: Robot not supported");
	}
}

void mp_t_clg_planner::main_task_algorithm()
{
	int clg_port = config.value <int>("board_localization", "[mp_block_move]");
	proxy = (boost::shared_ptr<clg_proxy>) new clg_proxy();
	//boost::thread::attributes attrs;
	//attrs.set_stack_size(4096*10);
	//pthread_attr_setschedpolicy(attrs.get_native_handle(), SCHED_RR);
	boost::thread t1(&clg_proxy::execute, proxy, boost::ref(clg_port));
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
