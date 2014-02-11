/*
 * clg_proxy.cc
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#include <string>
#include <vector>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include "base/lib/logger.h"

#include "clg_proxy.h"

namespace mrrocpp {
namespace mp {
namespace task {

clg_proxy::clg_proxy() {
	//manager = (boost::shared_ptr<action_manager>) new action_manager();
}

clg_proxy::~clg_proxy() {
}

clg_exception::clg_exception(const std::string& arg) :
	std::runtime_error("clg exception: " + arg) {
}

clg_connection_exception::clg_connection_exception(const std::string& arg) :
	clg_exception("clg connection exception: " + arg) {
}

void clg_proxy::set_task(mrrocpp::mp::task::mp_t_clg_planner mrrocpp_clg_task) {
	mrrocpp_task = mrrocpp_clg_task;
}

void clg_proxy::set_robot_name(lib::robot_name_t name) {
	robot_name = name;
}

int clg_proxy::process(Message msg) {
	std::string action = std::string(msg.action);
	std::vector<std::string> parameters;
	int result;

	printf("clg_proxy::process()\n");

	parameters.push_back(robot_name);

	for(int i = 0; i < 4; i++) {
		parameters.push_back(std::string(msg.params[i]));
	}
	if(msg.type == OBSERVATION) {
		if(action == "OBSERVE-COLOR") {
			tgroup.create_thread(boost::bind(&mrrocpp_task::observe_color, manager, boost::ref(parameters)));
			result = (parameters.back() == "true") ? 1 : ((parameters.back() == "false") ? 0 : -1);
		}
		else {
			throw clg_exception("clg_proxy()::process(): unknown observation type");
		}
	}
	else if(msg.type == ACTION) {
		result = -1;
		if(action == "OBSERV") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::observ, mrrocpp_task, boost::ref(parameters)));
		}
		else if(action == "MOVE") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::move, mrrocpp_task, boost::ref(parameters)));
		}
		else if(action == "PICKUP") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::pickup, mrrocpp_task, boost::ref(parameters)));
		}
		else if(action == "PUTDOWN") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::putdown, mrrocpp_task, boost::ref(parameters)));
		}
		else {
			throw clg_exception("clg_proxy()::process(): unknown action type");
		}
	}
	else {
		throw clg_exception("clg_proxy()::process(): bad message type");
	}
	return result;
}

void clg_proxy::communicate()
{
	using namespace boost::interprocess;

	int n, anwser;
	Message msg;

	printf("clg_proxy::communicate()\n");

	shared_memory_object::remove("ClgSharedMemory");
	managed_shared_memory segment(create_only, "ClgSharedMemory", 65536);

	objects_sma = segment.construct<int>("ObjectsArray")[12](UNKNOWN);
	coordinates_sma = segment.construct<double>("CoordinatesArray")[12*7](-1.0);

	while(1) {
    	msg.type = NOTHING;
		strcpy(msg.action, "");

		printf("clg_proxy::communicate() - loop begin\n");

		for(int i = 0; i < 4; i++) {
			strcpy(msg.params[i], "");
		}
		n = read(comm_sockfd, (void*) &msg, sizeof(msg));
		if(n < 0) {
		   throw clg_connection_exception("clg_proxy(): read() " + std::string(strerror(errno)));
		}

		printf("clg_proxy::communicate() - after read()\n");

		if(msg.type != NOTHING) {
		   std::cout << "Here is the message: " << msg.type << " " << msg.action << " ";
		   for(int i = 0; i < 4; i++) {
			   std::cout << msg.params[i] << " ";
		   }
		   std::cout << std::endl;
		}

		anwser = process(msg);

		if(msg.type == OBSERVATION) {
			n = write(comm_sockfd, (void*) &anwser, sizeof(int));
			if(n < 0) {
			   throw clg_connection_exception("clg_proxy(): write() " + std::string(strerror(errno)));
			}

			printf("clg_proxy::communicate() - after write()\n");
		}
	}

	shared_memory_object::remove("ClgSharedMemory");
}


void clg_proxy::connect(int portnr)
{
	logger::log_dbg("clg_proxy::execute(): Server started\n");
	printf("clg_proxy::execute()\n");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0) {
       throw clg_connection_exception("clg_proxy(): socket() " + std::string(strerror(errno)));
    }

	printf("clg_proxy::execute() - after socket\n");

    sockaddr_in serv_addr;
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portnr);
    int n = bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if(n < 0) {
    	throw clg_connection_exception("clg_proxy(): bind() " + std::string(strerror(errno)));
    }

	printf("clg_proxy::execute() - after bind\n");

    n = listen(sockfd,5);
    if(n < 0) {
    	throw clg_connection_exception("clg_proxy(): listen() " + std::string(strerror(errno)));
    }

	printf("clg_proxy::execute() - after listen\n");

    sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    comm_sockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if(comm_sockfd < 0) {
    	throw clg_connection_exception("clg_proxy(): accept() " + std::string(strerror(errno)));
    }

	printf("clg_proxy::execute() - after accept\n");
}


void clg_proxy::close_connection()
{
	close(sockfd);
	close(comm_sockfd);
	tgroup.join_all();
}

}
}
}
