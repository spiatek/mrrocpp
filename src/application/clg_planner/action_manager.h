/*
 * action_manager.h
 *
 *  Created on: 20-12-2013
 *      Author: spiatek
 */

#ifndef ACTION_MANAGER_H_
#define ACTION_MANAGER_H_

#include <string>
#include <vector>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include "base/lib/impconst.h"
#include "base/mp/mp_task.h"

#define UNKNOWN			0
#define	SINGLE_BLUE		1
#define SINGLE_RED		2
#define SINGLE_GREEN	3
#define SINGLE_YELLOW	4
#define	DOUBLE_BLUE		5
#define DOUBLE_RED		6
#define DOUBLE_GREEN	7
#define DOUBLE_YELLOW	8

namespace mrrocpp {
namespace mp {
namespace task {

class action_manager {
public:
	action_manager();
	virtual ~action_manager();

	bool check_position(std::string);
	bool check_object(std::string);
	bool check_color(std::string);

	int compute_position_for_position_board_generator(std::string);
	std::string get_trajectory_file_name(lib::robot_name_t, std::string, int);
	int color_string_to_int(std::string);

	bool observe_color(std::vector<std::string> &p);
	bool observ(std::vector<std::string> &p);
	bool move(std::vector<std::string> &p);
	bool pickup(std::vector<std::string> &p);
	bool putdown(std::vector<std::string> &p);

	boost::mutex mp_mutex;
};

}
}
}

#endif /* ACTION_MANAGER_H_ */
