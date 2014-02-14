/*
 * mp_t_clg_planner.cc
 *
 *  Created on: 19-12-2013
 *      Author: spiatek
 */

#include <iostream>

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "application/block_move/ecp_mp_g_position_board.h"
#include "application/block_move/ecp_mp_g_block_reaching.h"
#include "generators/ecp_mp_g_reach_already_localized_block.h"

#include "base/lib/logger.h"

#include "mp_t_clg_planner.h"

using namespace boost::interprocess;

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_t_clg_planner(_config);
}

clg_exception::clg_exception(const std::string& arg) :
	std::runtime_error("clg exception: " + arg) {
}

clg_connection_exception::clg_connection_exception(const std::string& arg) :
	clg_exception("clg connection exception: " + arg) {
}

mp_t_clg_planner::mp_t_clg_planner(lib::configurator &_config) :
		task(_config)
{
}

mp_t_clg_planner::~mp_t_clg_planner()
{
}

void mp_t_clg_planner::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6ot_m);

	int name = config.value <int>("robot_name", lib::MP_SECTION);
	if(name == 1) {
		robot_name = lib::irp6ot_m::ROBOT_NAME;
	}
	else if(name == 2) {
		robot_name = lib::irp6p_m::ROBOT_NAME;
	}
	else {
		throw std::runtime_error("MP: Robot not supported");
	}
}

void mp_t_clg_planner::main_task_algorithm()
{
	int clg_port = config.value <int>("clg_port", "[mp_clg_planner]");
	sr_ecp_msg->message("Clg_Planner run...");
	connect(clg_port);
	communicate();
	close_connection();
}

void mp_t_clg_planner::connect(int portnr)
{
	sr_ecp_msg->message("mp_t_clg_planner::connect()");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0) {
       throw clg_connection_exception("clg_proxy(): socket() " + std::string(strerror(errno)));
    }

    sr_ecp_msg->message("mp_t_clg_planner::connect() - after socket\n");

    sockaddr_in serv_addr;
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portnr);
    int n = bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if(n < 0) {
    	throw clg_connection_exception("clg_proxy(): bind() " + std::string(strerror(errno)));
    }

    sr_ecp_msg->message("mp_t_clg_planner::connect() - after bind\n");

    n = listen(sockfd,5);
    if(n < 0) {
    	throw clg_connection_exception("clg_proxy(): listen() " + std::string(strerror(errno)));
    }

    sr_ecp_msg->message("mp_t_clg_planner::connect() - after listen\n");

    sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    comm_sockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if(comm_sockfd < 0) {
    	throw clg_connection_exception("clg_proxy(): accept() " + std::string(strerror(errno)));
    }

    sr_ecp_msg->message("mp_t_clg_planner::connect() - after accept\n");
}

void mp_t_clg_planner::communicate()
{
	int n, anwser;
	Message msg;

	sr_ecp_msg->message("mp_t_clg_planner::communicate()");

	shared_memory_object::remove("ClgSharedMemory");
	managed_shared_memory segment(create_only, "ClgSharedMemory", 65536);

	objects_sma = segment.construct<int>("ObjectsArray")[MAX_BLOCKS_NUMBER](UNKNOWN);
	coordinates_sma = segment.construct<int>("CoordinatesArray")[MAX_BLOCKS_NUMBER*COORDINATES_NUMBER](-1.0);
	counters_sma = segment.construct<int>("CountersArray")[BLOCK_TYPE_NUMBER](0);

	while(1) {
    	msg.type = NOTHING;
		strcpy(msg.action, "");

		sr_ecp_msg->message("mp_t_clg_planner::communicate() - loop begin");

		for(int i = 0; i < 4; i++) {
			strcpy(msg.params[i], "");
		}
		n = read(comm_sockfd, (void*) &msg, sizeof(msg));
		if(n < 0) {
		   throw clg_connection_exception("clg_proxy(): read() " + std::string(strerror(errno)));
		}

		sr_ecp_msg->message("mp_t_clg_planner::communicate() - after read()");

		if(msg.type == SUCCESS) {
			sr_ecp_msg->message("mp_t_clg_planner::communicate() - plan found");
			return;
		}
		else if(msg.type == FAIL) {
			sr_ecp_msg->message("mp_t_clg_planner::communicate() - unable to find plan");
			return;
		}
		else if(msg.type != NOTHING) {
		   std::cout << "Here is the message: " << msg.type << " " << msg.action << " ";
		   for(int i = 0; i < MAX_PARAM_NUMBER; i++) {
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

			sr_ecp_msg->message("mp_t_clg_planner::communicate() - after write()");
		}
	}

	shared_memory_object::remove("ClgSharedMemory");
}

int mp_t_clg_planner::process(Message msg) {
	std::string action = std::string(msg.action);
	int result;

	sr_ecp_msg->message("mp_t_clg_planner::process()");

	ArgumentClass* args = new ArgumentClass();
	args->set_robot_name(robot_name);
	args->set_action_type(action);

	std::cout << "Adding: ";
	for(int i = 0; i < MAX_PARAM_NUMBER; i++) {
		std::cout <<  msg.params[i] << " ";
		args->add_parameter(msg.params[i]);
	}
	std::cout << std::endl;

	if(msg.type == OBSERVATION) {
		if(action == "OBSERVE-COLOR" || action == "OBSERVE-TYPE") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::observe_color, this, boost::ref(*args)));
			tgroup.join_all();
			result = (args->get_return_value()) ? 1 : 0;
		}
		else {
			throw clg_exception("clg_proxy()::process(): unknown observation type");
		}
	}
	else if(msg.type == ACTION) {
		result = -1;
		if(action == "OBSERV") {
			//tgroup.create_thread(boost::bind(&mp_t_clg_planner::observ, this, boost::ref(parameters)));
		}
		else if(action == "MOVE") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::move, this, boost::ref(*args)));
		}
		else if(action == "PICKUP") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::pickup, this, boost::ref(*args)));
		}
		else if(action == "PUTDOWN" || action == "PUTDOWN-SINGLE" || action == "PUTDOWN-DOUBLE" || action == "PUTDOWN-TRIPLE"
				    || action == "PDSEW" || action == "PDSNS" || action == "EW" || action == "NS"
				    || action == "EWD" || action == "NSD") {
			tgroup.create_thread(boost::bind(&mp_t_clg_planner::putdown, this, boost::ref(*args)));
		}
		else {
			throw clg_exception("clg_proxy()::process(): unknown action type");
		}
		tgroup.join_all();
	}
	else {
		throw clg_exception("clg_proxy()::process(): bad message type");
	}
	return result;
}

void mp_t_clg_planner::close_connection()
{
	close(sockfd);
	close(comm_sockfd);
	tgroup.join_all();
}

/**
 * observe_color()
 * Search for object with specified color in an image
 * Omits objects which have already been checked
 * Uses SMOOTH_JOINT_FILE_FROM_MP generator to reach search start position
 * Uses GEN_BLOCK_REACHING generator to run visual servoing
 * Corresponding to observe_color() CLG Planner action
 * @args - reference to ArgumentClass:
 * 		action_type: (1) OBSERVE-COLOR (2) OBSERVE-TYPE
 * 		parameters[0] - object (1)(2)
 * 		parameters[1] - color (1)(2)
 * 		parameters[2] - length (2)
 * If there is no such object in any of image areas, fills return_value with "false"
 */
bool mp_t_clg_planner::observe_color(ArgumentClass &args)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	sr_ecp_msg->message("mp_t_clg_planner::observe_color()");

	/* preparing shared arrays */
	managed_shared_memory segment(open_only, "ClgSharedMemory");

	std::pair<int*, size_t> sm_cont_o = segment.find<int>("ObjectsArray");
	int* objects_array;
	if(sm_cont_o.second != 0) {
		objects_array = sm_cont_o.first;
	}

	std::pair<int*, size_t> sm_cont_c = segment.find<int>("CoordinatesArray");
	int* coordinates_array;
	if(sm_cont_c.second != 0) {
		coordinates_array = sm_cont_c.first;
	}

	std::pair<int*, size_t> sm_cont_l = segment.find<int>("CountersArray");
	int* counters_array;
	if(sm_cont_l.second != 0) {
		counters_array = sm_cont_l.first;
	}

	sr_ecp_msg->message("mp_t_clg_planner::observe_color() - after preparing shared arrays");

	printf("%d\n", (int) args.parameter_vector_size());
	/* computing color integer value */
	int color_int = UNKNOWN;
	color_int = color_string_to_int(args);

	/* computing object integer value */
	std::string object_name = args.get_parameter(1);
	int object_int = atoi(object_name.substr(1).c_str());
	std::cout << "PARAMETRY: " << color_int << ", " << object_name << std::endl;

	sr_ecp_msg->message("mp_t_clg_planner::observe_color() - after computing values");

	std::cout << "OBIEKTY WSPOLDZIELONE: ";
	for(int i = 0; i < 12; i++) {
		std::cout << objects_array[i] << " ";
	}
	std::cout << std::endl;

	/* checks if color is already known and if it matches */
	if(objects_array[object_int] == color_int) {
		sr_ecp_msg->message("mp_t_clg_planner::observe_color() - color is already known and it matches");
		args.set_return_value(true);
	}
	/* checks if color is already known and if it doesn't match */
	else if(objects_array[object_int] != UNKNOWN) {
		sr_ecp_msg->message("mp_t_clg_planner::observe_color() - color is already known and it doesn't match");
		args.set_return_value(false);
	}
	/* color is unknown */
	else {
		sr_ecp_msg->message("mp_t_clg_planner::observe_color() - color is unknown");
		std::string trj_file_str;
		lib::robot_name_t robot_name = args.get_robot_name();

		int view = counters_array[object_int];
		if(view >= MAX_VIEWS_NUMBER) {
			sr_ecp_msg->message("mp_t_clg_planner::observe_color() - computed false value (max nr of views exceeded)");
			args.set_return_value(false);
		}

		int flag = 0;
		int number_of_servo_tries;					/* DOCELOWO STALA - ile prob odpalenia serwomechanizmu */
		for(number_of_servo_tries = 0; number_of_servo_tries < MAX_NUMBER_OF_SERVO_TRIES; ++number_of_servo_tries) {

			trj_file_str = get_trajectory_file_name(robot_name, 'T', view);

			sr_ecp_msg->message("mp_t_clg_planner::observe_color() - servo run");

			wait_ms(1000);

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, trj_file_str, robot_name);
			wait_for_task_termination(false, robot_name);
			wait_ms(1000);

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_BLOCK_REACHING, color_int, "", robot_name);
			wait_for_task_termination(false, robot_name);

			if(robot_m[robot_name]->ecp_reply_package.variant == 1) {		/* POWODZENIE W WYSZUKIWANIU */
				flag = 1;
				break;
			}
		}
		if(flag == 1) {
			sr_ecp_msg->message("mp_t_clg_planner::observe_color() - computed true value");

			std::cout << "POSITION FROM ECP:" << std::endl;
			int index;
			float position;
			for(int i = 0; i < 7; ++i) {
				int ihku = robot_m[robot_name]->ecp_reply_package.sg_buf.data[i];
				position = (ihku/10000.0) - 5.0;
				std::cout << position << " ";
				index = ((object_int - 1) * 7) + i;
				coordinates_array[index] = ihku;
			}
			std::cout << std::endl;

			++counters_array[object_int];

			objects_array[object_int] = color_int;
			args.set_return_value(true);
		}
		else {
			sr_ecp_msg->message("mp_t_clg_planner::observe_color() - computed false value");
			args.set_return_value(false);
		}
	}
	return true;
}

/* Niepotrzebne */
bool mp_t_clg_planner::observ(ArgumentClass &args)
{
	boost::mutex::scoped_lock lock(mp_mutex);
	return true;
}

/**
 * move()
 * Moves manipulator arm from one position to another
 * Uses SMOOTH_JOINT_FILE_FROM_MP to reach the table
 * Uses SMOOTH_JOINT_FILE_FROM_MP and ECP_GEN_POSITION_BOARD to reach position on board
 * Corresponding to move() CLG Planner action
 * @args - reference to ArgumentClass:
 * 		action_type: MOVE
 * 		parameters[0] - start position
 * 		parameters[1] - finish position
 */
bool mp_t_clg_planner::move(ArgumentClass &args)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	std::string trj_file_str;
	lib::robot_name_t robot_name = args.get_robot_name();
	std::string position_from = args.get_parameter(0);
	std::string position_to = args.get_parameter(1);

	sr_ecp_msg->message("mp_t_clg_planner::move()");

	if(check_position(position_from) == false) {
		throw std::runtime_error("Action Manager: bad position received from CLG Planner");
	}

	char zero_position = position_to[0];
	trj_file_str = get_trajectory_file_name(robot_name, zero_position, 0);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, trj_file_str, robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	sr_ecp_msg->message("mp_t_clg_planner::move() - basic move done");

	/** additional move due to reach more specified position_to **/
	if(zero_position == 'P') {

		int position_int;
		position_int = compute_position_for_position_board_generator(position_to);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_POSITION_BOARD, position_int, "", robot_name);
		wait_for_task_termination(false, robot_name);
		wait_ms(1000);

		sr_ecp_msg->message("mp_t_clg_planner::move() - additional move done");
	}

	return true;
}

/**
 * pickup()
 * Picks up specified object from specified position
 * Uses TFF_GRIPPER_APPROACH generator to go down
 * Uses REACH_ALREADY_LOCALIZED_BLOCK generator to perform a specific absolute move
 * Corresponding to pickup() CLG Planner action
 * @args - reference to ArgumentClass:
 * 		action_type: MOVE
 * 		parameters[0] - object
 * 		parameters[1] - position
 */
bool mp_t_clg_planner::pickup(ArgumentClass &args)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	sr_ecp_msg->message("mp_t_clg_planner::pickup()");

	/* preparing shared arrays */
	managed_shared_memory segment(open_only, "ClgSharedMemory");

	std::pair<int*, size_t> sm_cont_o = segment.find<int>("ObjectsArray");
	int* objects_array;
	if(sm_cont_o.second != 0) {
		objects_array = sm_cont_o.first;
	}

	std::pair<int*, size_t> sm_cont_c = segment.find<int>("CoordinatesArray");
	int* coordinates_array;
	if(sm_cont_c.second != 0) {
		coordinates_array = sm_cont_c.first;
	}

	std::pair<int*, size_t> sm_cont_l = segment.find<int>("CountersArray");
	int* counters_array;
	if(sm_cont_l.second != 0) {
		counters_array = sm_cont_l.first;
	}

	sr_ecp_msg->message("mp_t_clg_planner::pickup() - after preparing shared arrays");

	lib::robot_name_t robot_name = args.get_robot_name();

	std::string object_name = args.get_parameter(0);
	int object_int = atoi(object_name.substr(1).c_str());
	int array_for_generator[7];

	if(objects_array[object_int] == UNKNOWN) {

		sr_ecp_msg->message("mp_t_clg_planner::pickup() - object is unknown");

		std::string trj_file_str;
		int view = counters_array[object_int];
		if(view >= MAX_VIEWS_NUMBER) {
			throw std::runtime_error("mp_t_clg_planner::pickup() - max number of views exceeded");
		}

		int flag = 0;
		for(int number_of_servo_tries = 0; number_of_servo_tries < MAX_NUMBER_OF_SERVO_TRIES; ++number_of_servo_tries) {

			trj_file_str = get_trajectory_file_name(robot_name, 'T', view);

			sr_ecp_msg->message("mp_t_clg_planner::pickup() - servo run");

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, trj_file_str, robot_name);
			wait_for_task_termination(false, robot_name);
			wait_ms(1000);

			//TODO: zaimplementować mechanizm pobierania wszystkich cech z clg i wyznaczania na tej podstawie tego brakującego
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_BLOCK_REACHING, SINGLE_BLUE, "", robot_name);
			wait_for_task_termination(false, robot_name);

			if(robot_m[robot_name]->ecp_reply_package.variant == 1) {		/* POWODZENIE W WYSZUKIWANIU */
				flag = 1;
				sr_ecp_msg->message("mp_t_clg_planner::pickup() - servo computed true value");
				++counters_array[object_int];
				objects_array[object_int] = SINGLE_BLUE;
				for(int i = 0; i < 7; ++i) {
					array_for_generator[i] = robot_m[robot_name]->ecp_reply_package.sg_buf.data[i];
				}
				break;
			}
		}
		if(flag == 0) {
			throw std::runtime_error("mp_t_clg_planner::pickup() - servo failed");
		}
	}
	else {		/* color is already observed */
		int index = ((object_int - 1) * 7);

		std::cout << "COORDINATES ARRAY: ";
		for(size_t i = 0; i < 7; ++i) {
			array_for_generator[i] = coordinates_array[index];
			std::cout << coordinates_array[index] << " ";
			//array_for_generator[i] = 10000 + i;
			index++;
		}
		std::cout << std::endl;
	}

	std::cout << "ALL COORDINATES ARRAY FOR OBJECT " << object_name << ", " << object_name.substr(1) << ", " << object_int << ": ";
	for(size_t i = 0; i < 12*7; ++i) {
		std::cout << coordinates_array[i] << " ";
	}
	std::cout << std::endl;

	std::string position_str = args.get_parameter(1);
	if(position_str != "TABLE") {
		throw std::runtime_error("Action Manager: unknown position received from CLG Planner::pickup");
	}

	sr_ecp_msg->message("mp_t_clg_planner::pickup() - after computing params");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_REACH_ALREADY_LOCALIZED_BLOCK, 0, array_for_generator, robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.03, 800, 3), robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/up_after_pickup.trj", robot_name);
	wait_for_task_termination(false, robot_name);
	wait_ms(1000);

	sr_ecp_msg->message("mp_t_clg_planner::pickup() - after activating generators");

	return true;
}

/**
 * putdown()
 * Puts currently held block on specified position
 * Uses TFF_GRIPPER_APPROACH generator to go down
 * Uses SMOOTH_ANGLE_AXIS_FILE_FROM_MP generator to perform a specific relative move
 * Corresponding to putdown() CLG Planner action
 * @p - reference to the parameter vector
 * 		p[0] - robot name
 * 		p[1] - object name
 * 		p[2] - position (it should be equal to present robot position)
 * 		p[3] - color string
 */
bool mp_t_clg_planner::putdown(ArgumentClass &args)
{
	boost::mutex::scoped_lock lock(mp_mutex);

	sr_ecp_msg->message("mp_t_clg_planner::putdown()");

	lib::robot_name_t robot_name = args.get_robot_name();
	std::string object_name = args.get_parameter(0);
	std::string position = args.get_parameter(1);
	std::string action_name = args.get_action_type();
	//std::string color_string = args.get_parameter(2);

	if(check_position(position) == false) {
		throw std::runtime_error("Action Manager: bad position received from CLG Planner");
	}

	sr_ecp_msg->message("mp_t_clg_planner::putdown() - after computing params");

	//jeśli trzeba, to przesunięcie o pół lub jeden klocek
	if(action_name == "EWD" || action_name == "PUTDOWN-DOUBLE") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition EW single");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_sintr.trj", robot_name);
		wait_for_task_termination(false, robot_name);

		wait_ms(1000);
	}
	else if(action_name == "NSD") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition NS single");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ns_sintr.trj", robot_name);
		wait_for_task_termination(false, robot_name);

		wait_ms(1000);
	}
	else if(action_name == "PUTDOWN-TRIPLE") {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - transition EW double");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_doutr.trj", robot_name);
		wait_for_task_termination(false, robot_name);

		wait_ms(1000);
	}

	//jeśli trzeba, to obrót o 90 stopni
	if(action_name.find("EW") != std::string::npos) {
		sr_ecp_msg->message("mp_t_clg_planner::putdown() - rotation");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/putdown_ew_rotate.trj", robot_name);
		wait_for_task_termination(false, robot_name);

		wait_ms(1000);
	}

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 600, 2), robot_name);
	wait_for_task_termination(false, robot_name);

	wait_ms(4000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/build.trj", robot_name);
	wait_for_task_termination(false, robot_name);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, 1, "", robot_name);
	wait_for_task_termination(false, robot_name);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/clg_planner/trjs/up_after_pushing.trj", robot_name);
	wait_for_task_termination(false, robot_name);

	wait_ms(1000);

	sr_ecp_msg->message("mp_t_clg_planner::putdown() - after activating generators");

	return true;
}

/**
 * compute_position_for_position_board_generator()
 * @position_string - string received from CLG Planner
 * returns integer param for POSITION_BOARD generator
 */
int mp_t_clg_planner::compute_position_for_position_board_generator(std::string position_string)
{
	sr_ecp_msg->message("mp_t_clg_planner::compute_position_for_position_board_generator()");

	std::vector<int> position_to;

	if(position_string.size() == 4) {
		position_to.push_back(static_cast<int>(position_string[1] - '0') - 1);
		position_to.push_back(static_cast<int>(position_string[2] - '0') - 1);
		position_to.push_back(static_cast<int>(position_string[3] - '0') - 1);
	}
	if(position_string.size() == 3) {
		position_to.push_back(0);
		position_to.push_back(static_cast<int>(position_string[1] - '0') - 1);
		position_to.push_back(static_cast<int>(position_string[2] - '0') - 1);
		std::cout << "Tutaj" << std::endl;
	}
	else {
		throw std::runtime_error("CLG::move(): bad position");
	}

	sr_ecp_msg->message("mp_t_clg_planner::compute_position_for_position_board_generator() - end");

	int position_int = 100 * position_to[0] + 10 * position_to[1] + position_to[2];

	std::cout << "POSITION INTEGER: " << position_int << std::endl;

	return position_int;
}

/**
 * get_trajectory_file_name()
 * @robot_name - name of an active robot
 * @area_str - string specifying absolute position that robot have to reach (P - building, TABLE - searching)
 * @step - number of table view (0 for P)
 * returns name of a trajectory file which will be sent to SMOOTH_JOINT_FILE_FROM_MP generator
 */
std::string mp_t_clg_planner::get_trajectory_file_name(lib::robot_name_t robot_name, char area, int step)
{
	sr_ecp_msg->message("mp_t_clg_planner::get_trajectory_file_name()");
	std::cout << "Area str: " << area << ", step: " << step << std::endl;

	std::string trj_file_str;
	if(robot_name == lib::irp6ot_m::ROBOT_NAME) {
		if(area == 'T' && step == 0) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_1.trj";
		}
		else if(area == 'T' && step == 1) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_track_2.trj";
		}
		else if(area == 'P') {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_build_start_track.trj";
		}
		else {
			throw std::runtime_error("Action Manager: bad area type received from CLG Planner");
		}
	}
	else if(robot_name == lib::irp6p_m::ROBOT_NAME) {
		if(area == 'T' && step == 0) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_postument_1.trj";
		}
		else if(area == 'T' && step == 1) {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_search_area_start_postument_2.trj";
		}
		else if(area == 'P') {
			trj_file_str = "../../src/application/clg_planner/trjs/pos_build_start_postument.trj";
		}
		else {
			throw std::runtime_error("Action Manager: bad area type received from CLG Planner");
		}
	}
	else {
		throw std::runtime_error("MP: Robot not supported");
	}
	return trj_file_str;
}

/**
 * color_string_to_int()
 * @args arguments of observe action
 * returns integer value of string
 */
int mp_t_clg_planner::color_string_to_int(ArgumentClass args)
{
	sr_ecp_msg->message("mp_t_clg_planner::color_string_to_int()");

	int color_int;
	std::string action_type = args.get_action_type();
	std::string color_string = args.get_parameter(0);
	std::string length_string = args.get_parameter(1);
	char color_char = color_string[0];
	std::cout << length_string << ", " << color_string << ", " << length_string[0] << ",  " << color_char << std::endl;

	if(action_type == "OBSERVE-TYPE") {
		if(length_string[0] == 'D') {
			if(color_char == 'B') {
				color_int = DOUBLE_BLUE;
			}
			else if(color_char == 'R') {
				color_int = DOUBLE_RED;
			}
			else if(color_char == 'G') {
				color_int = DOUBLE_GREEN;
			}
			else if(color_char == 'Y') {
				color_int = DOUBLE_YELLOW;
			}
			else {
				throw std::runtime_error("Action Manager: bad color received from CLG Planner");
				exit(1);
			}
			return color_int;
		}
	}

	if(color_char == 'B') {
		color_int = SINGLE_BLUE;
	}
	else if(color_char == 'R') {
		color_int = SINGLE_RED;
	}
	else if(color_char == 'G') {
		color_int = SINGLE_GREEN;
	}
	else if(color_char == 'Y') {
		color_int = SINGLE_YELLOW;
	}
	else {
		throw std::runtime_error("Action Manager: bad color received from CLG Planner");
		exit(1);
	}
	return color_int;
}

bool mp_t_clg_planner::check_position(std::string start_position) {
	//TODO: run getPosition() generator
	//check if start_position is similar
	return true;
}

bool mp_t_clg_planner::check_object(std::string clg_object) {
	//TODO: check if clg_object is currently in use
	return true;
}

bool mp_t_clg_planner::check_color(std::string clg_color) {
	//TODO: check if currently processing object has clg_color
	return true;
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
