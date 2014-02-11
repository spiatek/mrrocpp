/*
 * clg_proxy.h
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#ifndef CLG_PROXY_H_
#define CLG_PROXY_H_

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdexcept>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string>

#include "base/lib/impconst.h"
#include "base/mp/mp_task.h"

#include "mp_t_clg_planner.h"

#define NOTHING		-1
#define	OBSERVATION	0
#define ACTION		1

#define MAX_STR_LEN	32

namespace mrrocpp {
namespace mp {
namespace task {

typedef struct message {
     int type;
     char action[MAX_STR_LEN];
     char params[4][MAX_STR_LEN];
} Message;

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

class clg_proxy {
public:
	clg_proxy();
	virtual ~clg_proxy();

	void set_task(mp::task::task);
	void set_robot_name(lib::robot_name_t);
	void connect(int);
	void communicate();
	int process(Message msg);
	void close_connection();

private:
	boost::shared_ptr<mp::task::mp_t_clg_planner> mrrocpp_task;
	boost::thread_group tgroup;
	lib::robot_name_t robot_name;
	int sockfd;					/* connection socket descriptor */
	int comm_sockfd;			/* read/write socket descriptor */
	int mp_execution_flag;		/* flag which specify if there is any active generator at the moment	*/
	int *objects_sma;
	double *coordinates_sma;
};

}
}
}

#endif /* CLG_PROXY_H_ */
