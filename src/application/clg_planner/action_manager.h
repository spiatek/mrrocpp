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

class action_manager {
public:
	action_manager();
	virtual ~action_manager();

	bool observe_color(std::vector<std::string> &p);
	bool observ(std::vector<std::string> &p);
	bool move(std::vector<std::string> &p);
	bool pickup(std::vector<std::string> &p);
	bool putdown(std::vector<std::string> &p);

	boost::mutex mp_mutex;
};

#endif /* ACTION_MANAGER_H_ */
