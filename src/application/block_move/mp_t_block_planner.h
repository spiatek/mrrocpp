/*
 * mp_t_block_planner.h
 *
 *  Created on: 29-06-2013
 *      Author: spiatek
 */

#ifndef MP_T_BLOCK_PLANNER_H_
#define MP_T_BLOCK_PLANNER_H_

#include <list>
#include <string>

namespace mrrocpp {
namespace mp {
namespace task {

class block_planner : public task
{

protected:

	int present_color;
	std::vector <int> present_position;

public:

	block_planner(lib::configurator &_config);

	void create_robots(void);
	void main_task_algorithm(void);

	/* actions */
	void observ_action(void);
	void observe_color_action(void);
	void move_action(void);
	void putdown_action(void);
	void pickup_action(void);

	void localize_board(void);
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_T_BLOCK_PLANNER_H_ */
