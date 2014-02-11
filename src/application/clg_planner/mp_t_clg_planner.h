/*
 * mp_t_clg_planner.h
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#ifndef MP_T_CLG_PLANNER_H_
#define MP_T_CLG_PLANNER_H_

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"
#include "base/lib/impconst.h"
#include "base/lib/configurator.h"

#include "base/ecp/ecp_generator.h"
#include "generator/ecp/get_position/ecp_g_get_position.h"

#define NOTHING			-1
#define	OBSERVATION		0
#define ACTION			1
#define SUCCESS			2
#define FAIL			3

#define UNKNOWN			0
#define	SINGLE_BLUE		1
#define SINGLE_RED		2
#define SINGLE_GREEN	3
#define SINGLE_YELLOW	4
#define	DOUBLE_BLUE		5
#define DOUBLE_RED		6
#define DOUBLE_GREEN	7failed
#define DOUBLE_YELLOW	8

#define MAX_STR_LEN			16
#define MAX_PARAM_NUMBER	12

namespace mrrocpp {
namespace mp {
namespace task {

typedef struct message {
     int type;
     char action[MAX_STR_LEN];
     char params[MAX_PARAM_NUMBER][MAX_STR_LEN];
} Message;

class ArgumentClass {
public:
	ArgumentClass() {}
	~ArgumentClass() {}

	bool get_return_value() { return return_value; }
	void set_return_value(bool value) { return_value = value; }

	void add_parameter(std::string param) { parameters.push_back(param); }
	std::string get_parameter(int index) { return parameters.at(index); }
	int parameter_vector_size() { return parameters.size(); }

	void set_action_type(std::string type) { action_type = type; }
	std::string get_action_type() { return action_type; }

	std::string get_robot_name() { return robot_name; }
	void set_robot_name(std::string name) { robot_name = name; }

private:
	std::string robot_name;
	std::string action_type;
	std::vector<std::string> parameters;
	bool return_value;
};

class clg_exception : public std::runtime_error
{
public:
	explicit clg_exception(const std::string& arg);
};

class clg_connection_exception : public clg_exception
{
public:
	explicit clg_connection_exception(const std::string& arg);
};

class mp_t_clg_planner : public task
{
public:
	mp_t_clg_planner(lib::configurator &_config);
	virtual ~mp_t_clg_planner();

	void connect(int);
	void communicate();
	int process(Message msg);
	void close_connection();

	bool observe_color(ArgumentClass &args);
	bool observ(ArgumentClass &args);
	bool move(ArgumentClass &args);
	bool pickup(ArgumentClass &args);
	bool putdown(ArgumentClass &args);

	void create_robots();
	void main_task_algorithm();

private:
	bool check_position(std::string);
	bool check_object(std::string);
	bool check_color(std::string);

	int compute_position_for_position_board_generator(std::string);
	std::string get_trajectory_file_name(lib::robot_name_t, char, int);
	int color_string_to_int(ArgumentClass args);

	int sockfd;					/* connection socket descriptor */
	int comm_sockfd;			/* read/write socket descriptor */
	int mp_execution_flag;		/* flag which specify if there is any active generator at the moment	*/
	int *objects_sma;
	double *coordinates_sma;

	lib::robot_name_t robot_name;
	boost::mutex mp_mutex;
	boost::thread_group tgroup;
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /* MP_T_CLG_PLANNER_H_ */
