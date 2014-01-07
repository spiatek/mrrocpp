/*
 * mp_t_clg_planner.h
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#ifndef MP_T_CLG_PLANNER_H_
#define MP_T_CLG_PLANNER_H_

#include "base/mp/mp_task.h"
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace mp {
namespace task {

class mp_t_clg_planner : public task
{
public:
	mp_t_clg_planner(lib::configurator &_config);
	virtual ~mp_t_clg_planner();

	void create_robots();
	void main_task_algorithm();

private:
	std::string robot_name;
	boost::shared_ptr<clg_proxy> proxy;
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_T_CLG_PLANNER_H_ */
