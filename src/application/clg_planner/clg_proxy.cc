/*
 * clg_proxy.cc
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#include <string>
#include <vector>
#include "base/lib/logger.h"

#include "clg_proxy.h"

clg_proxy::clg_proxy() {
	manager = (boost::shared_ptr<action_manager>) new action_manager();
}

clg_proxy::~clg_proxy() {
}

clg_exception::clg_exception(const std::string& arg) :
	std::runtime_error("clg exception: " + arg) {
}

clg_connection_exception::clg_connection_exception(const std::string& arg) :
	clg_exception("clg connection exception: " + arg) {
}

int clg_proxy::process(Message msg) {
	std::string action = std::string(msg.action);
	std::vector<std::string> parameters;
	for(int i = 0; i < 4; i++) {
		parameters.push_back(std::string(msg.params[i]));
	}
	if(msg.type == OBSERVATION) {
		if(action == "OBSERVE-COLOR") {
			//boost::thread* thr = new boost::thread(&action_manager::observe_color, p);
			//tgroup.add_thread(thr);
			tgroup.create_thread(boost::bind(&action_manager::observe_color, manager, boost::ref(parameters)));
		}
		else {
			throw clg_exception("clg_proxy()::process(): unknown observation type");
		}
	}
	else if(msg.type == ACTION) {
		if(action == "OBSERV") {
			//boost::thread* thr = new boost::thread(&action_manager::observ, p);
			//tgroup.add_thread(thr);
			tgroup.create_thread(boost::bind(&action_manager::observ, manager, boost::ref(parameters)));
		}
		else if(action == "MOVE") {
			//boost::thread* thr = new boost::thread(&action_manager::move, p);
			//tgroup.add_thread(thr);
			tgroup.create_thread(boost::bind(&action_manager::move, manager, boost::ref(parameters)));
		}
		else if(action == "PICKUP") {
			//boost::thread* thr = new boost::thread(&action_manager::pickup, p);
			//tgroup.add_thread(thr);
			tgroup.create_thread(boost::bind(&action_manager::pickup, manager, boost::ref(parameters)));
		}
		else if(action == "PUTDOWN") {
			//boost::thread* thr = new boost::thread(&action_manager::putdown, p);
			//tgroup.add_thread(thr);
			tgroup.create_thread(boost::bind(&action_manager::putdown, manager, boost::ref(parameters)));
		}
		else {
			throw clg_exception("clg_proxy()::process(): unknown action type");
		}
	}
	else {
		throw clg_exception("clg_proxy()::process(): bad message type");
	}
	return 0;
}

void clg_proxy::communicate(int sockfd)
{
	int n, anwser;
	Message msg;
    while(1) {
    	msg.type = NOTHING;
		strcpy(msg.action, "");
		for(int i = 0; i < 4; i++) {
			strcpy(msg.params[i], "");
		}
		n = read(sockfd, (void*) &msg, sizeof(msg));
		if(n < 0) {
		   throw clg_connection_exception("clg_proxy(): read() " + std::string(strerror(errno)));
		}
		if(msg.type != NOTHING) {
		   std::cout << "Here is the message: " << msg.type << " " << msg.action << " ";
		   for(int i = 0; i < 4; i++) {
			   std::cout << msg.params[i] << " ";
		   }
		   std::cout << std::endl;
		}
		anwser = 1;//process(msg);
		if(msg.type == OBSERVATION) {
		   n = write(sockfd, (void*) &anwser, sizeof(int));
		}
		if(n < 0) {
		   throw clg_connection_exception("clg_proxy(): write() " + std::string(strerror(errno)));
		}
	}

}


void clg_proxy::execute(int portnr)
{
	logger::log_dbg("clg_proxy::execute(): Server started\n");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0) {
       throw clg_connection_exception("clg_proxy(): socket() " + std::string(strerror(errno)));
    }

    sockaddr_in serv_addr;
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portnr);
    int n = bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if(n < 0) {
    	throw clg_connection_exception("clg_proxy(): bind() " + std::string(strerror(errno)));
    }

    n = listen(sockfd,5);
    if(n < 0) {
    	throw clg_connection_exception("clg_proxy(): listen() " + std::string(strerror(errno)));
    }

    sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    comm_sockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if(comm_sockfd < 0) {
    	throw clg_connection_exception("clg_proxy(): accept() " + std::string(strerror(errno)));
    }
}


void clg_proxy::close_connection()
{
	close(sockfd);
	close(comm_sockfd);
	tgroup.join_all();
}
