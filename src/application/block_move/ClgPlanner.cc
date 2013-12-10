/*
 * ClgPlanner.cc
 *
 *  Created on: 18-06-2013
 *      Author: spiatek
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ClgPlanner.h"

void ClgPlanner::error(const char *msg)
{
    perror(msg);
    exit(1);
}

int ClgPlanner::getPlan()
{
	return plan;
}

int ClgPlanner::getCurrentAction()
{
	return current_action;
}

void ClgPlanner::initializeConnection(int port_nr)
{
	socklen_t cli_len;
    struct sockaddr_in serv_addr, cli_addr;

    sockfd_s = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd_s < 0) {
    	error("ERROR opening socket");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port_nr);

    if(bind(sockfd_s, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    	error("ERROR on binding");
    }

    listen(sockfd_s, 5);
    cli_len = sizeof(cli_addr);

	sockfd = accept(sockfd_s, (struct sockaddr *) &cli_addr, &cli_len);
    if(sockfd < 0) {
    	error("ERROR on accept");
    }

    return;
}

void ClgPlanner::checkConnection()
{
	int n;
    message msg;

	n = read(sockfd,(void*) &msg, sizeof(msg));
	if(n < 0) {
		error("ERROR reading from socket");
	}

	n = write(sockfd, "OK", 2);
	if(n < 0) {
		error("ERROR writing to socket");
	}

	return;
}

int ClgPlanner::clgMrrocppCommunication()
{
	int i, n;
	message msg;

	bzero(buffer,256);

	n = read(sockfd, (void*) &msg, sizeof(msg));
    if(n < 0) {
    	error("ERROR reading from socket");
	}

	/* check if search algorithm finished */
	if(msg.number == -1) {
		return 1;
	}

	printf("Here is the message: %d %s %s %s %s %s\n",msg.number, msg.action, msg.params[0], msg.params[1], msg.params[2], msg.params[3]);

	/* powinno zapisywać numer akcji do wykonania, czekanie powinno być wywoływane później */

	executeAction(msg);

	/* check if observation received */
	if(strcmp(msg.action, "OBSERVE") == 0) {
        msg_res.result = executeSenseAction(msg);
        n = write(sockfd, (void*) &msg_res, sizeof(msg_res));
	}

	i++;

	return 0;
}

void ClgPlanner::closeConnection()
{
    close(sockfd);
    close(sockfd_s);
    return 0;
}

int ClgPlanner::executeAction(message msg)
{
	current_action.first = getAction(msg.action);
	current_action.second = getParams(msg.params);

	plan.push_back(current_action);

	/* execute */

	return 1;
}

int ClgPlanner::executeSenseAction(message msg)
{
	current_action.first = getAction(msg.action);
	current_action.second = getParams(msg.params);

	plan.push_back(current_action);

	/* execute */

	return 1;
}

ACTION ClgPlanner::getAction(const char *anm)
{
	ACTION a = ACT_NOACTION;

	if(strcmp(anm, "OBSERV") == 0) {
		a = ACT_OBSERVE;
	}
	if(strcmp(anm, "PICKUP") == 0) {
		a = ACT_PICKUP;
	}
	if(strcmp(anm, "MOVE") == 0) {
		a = ACT_MOVE;
	}
	if(strcmp(anm, "PUTDOWN") == 0) {
		a = ACT_PUTDOWN;
	}
	if(strcmp(anm, "OBSERVETYPE") == 0) {
		a = ACT_OBSERVE_TYPE;
	}
	if(strcmp(anm, "OBSERVEBLOCK") == 0) {
		a = ACT_OBSERVE;
	}
	if(strcmp(anm, "PUTDOWNSINGLE") == 0) {
		a = ACT_PUTDOWN_S;
	}
	if(strcmp(anm, "PUTDOWNDOUBLE") == 0) {
		a = ACT_PUTDOWN_D;
	}
	if(strcmp(anm, "PUTDOWNTRIPLE") == 0) {
		a = ACT_PUTDOWN_T;
	}
	if(strcmp(anm, "PDSEW") == 0) {
		a = ACT_PUTDOWN_SEW;
	}
	if(strcmp(anm, "PDSNS") == 0) {
		a = ACT_PUTDOWN_SNS;
	}
	if(strcmp(anm, "OBSERVE") == 0) {
		a = ACT_OBSERVE_COLOR;
	}

	return a;
}

PARAMS ClgPlanner::getParams(char **pms)
{
	PARAMS params;
	std::string str;

	for(int i = 0; i < PARAMS_MAX_NR; i++) {
		if(strcmp(pms[i],"a") == 0) {
			break;
		}
		params.push_back(str(pms[i]));
	}

	return params;
}
