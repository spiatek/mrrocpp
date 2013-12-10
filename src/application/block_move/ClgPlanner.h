/*
 * ClgPlanner.h
 *
 *  Created on: 18-06-2013
 *      Author: spiatek
 */

#ifndef CLGPLANNER_H_
#define CLGPLANNER_H_

#define ACT_NOACTION		-1
#define ACT_OBSERVE			0
#define ACT_OBSERVE_COLOR	14
#define ACT_OBSERVE_TYPE	1
#define ACT_PICKUP			2
#define ACT_MOVE			3
#define ACT_PUTDOWN			10
#define ACT_PUTDOWN_S		11
#define ACT_PUTDOWN_D		12
#define ACT_PUTDOWN_T		13
#define ACT_PUTDOWN_SNS		4
#define ACT_PUTDOWN_DNS		5
#define ACT_PUTDOWN_TNS		6
#define ACT_PUTDOWN_SEW		7
#define ACT_PUTDOWN_DEW		8
#define ACT_PUTDOWN_TEW		9

#define PARAMS_MAX_NR		8

typedef int ACTION;
typedef std::vector<std::string> PARAMS;
typedef std::vector<std::pair<ACTION, PARAMS> > PLAN;

struct message {
     int number;
     char action[16];
     char params[8][16];
};

struct res_msg {
     int index;
     int result;
};

class ClgPlanner {

private:

	int sockfd_s, sockfd;
	PLAN plan;
	std::pair<ACTION, PARAMS> current_action;

	ClgPlanner(void);

	void error(const char *msg);
	ACTION getAction(const char *anm);
	PARAMS getParams(char **pms);

public:

	void initializeConnection(int port_nr);
	void checkConnection();
	void closeConnection();

	int clgMrrocppCommunication();

	int executeAction(message msg);
	int executeSenseAction(message msg);
};


#endif /* CLGPLANNER_H_ */
