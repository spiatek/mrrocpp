/*
 * ecp_g_block_reaching.cc
 *
 *  Created on: 2012-02-13
 *      Author: spiatek
 */

#include <iostream>

#include <boost/thread.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include "ecp_g_block_reaching.h"

#define BOARD_COLOR 5

using namespace boost::interprocess;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

block_reaching::block_reaching(task::task & _ecp_t) : common::generator::generator(_ecp_t)
{
	generator_name = ecp_mp::generator::ECP_GEN_BLOCK_REACHING;

	//getting configuration parameters
	ecp_br_config_section_name = "[block_reaching]";
	int timeout = _ecp_t.config.value <int>("sm_timeout", ecp_br_config_section_name);
	int block_localization = _ecp_t.config.value <int>("block_localization", ecp_br_config_section_name);
	int board_localization = _ecp_t.config.value <int>("board_localization", ecp_br_config_section_name);

	//defining a type of servovision
	sr_ecp_msg.message("Creating visual servo...");
	if (block_localization == 1) {
		vs_config_section_name = "[block_reaching_servovision]";
	} else if (board_localization == 1) {
		vs_config_section_name = "[board_localization_servovision]";
	}

	//creating servo generator
	boost::shared_ptr <servovision::position_constraint> cube(new servovision::cubic_constraint(_ecp_t.config, vs_config_section_name));

	reg = (boost::shared_ptr <servovision::visual_servo_regulator>)
			new servovision::regulator_p(_ecp_t.config, vs_config_section_name);
	sr_ecp_msg.message("Regulator ready");

	logger::log_dbg("Regulator ready\n");

	ds = (boost::shared_ptr <ecp_mp::sensor::discode::discode_sensor>)
			new ecp_mp::sensor::discode::discode_sensor(_ecp_t.config, vs_config_section_name);
	sr_ecp_msg.message("Sensor ready");

	logger::log_dbg("Sensor ready\n");

	vs = (boost::shared_ptr <servovision::visual_servo>)
			new servovision::ib_eih_visual_servo(reg, ds, vs_config_section_name, _ecp_t.config);
	sr_ecp_msg.message("Servovision ready");

	logger::log_dbg("Servovision ready\n");

	object_reached_term_cond = (boost::shared_ptr <servovision::termination_condition>)
			new servovision::object_reached_termination_condition(_ecp_t.config, vs_config_section_name);
	sr_ecp_msg.message("Object reached term condition ready");

	logger::log_dbg("Object reached term condition ready\n");

	timeout_term_cond = (boost::shared_ptr <servovision::termination_condition>)
			new servovision::timeout_termination_condition(timeout);
	sr_ecp_msg.message("Timeout condition ready");

	logger::log_dbg("Timeout condition ready\n");

	sm = (boost::shared_ptr <common::generator::single_visual_servo_manager>)
			new common::generator::single_visual_servo_manager(_ecp_t, vs_config_section_name.c_str(), vs);
	sr_ecp_msg.message("Servo manager ready");

	logger::log_dbg("Servo manager ready\n");

	sm->add_position_constraint(cube);
	sr_ecp_msg.message("Position constraint added");

	logger::log_dbg("Position constraint added\n");

	sm->configure();
	sr_ecp_msg.message("Servo manager configured");

	//creating discode sensor for rpc communication
	sr_ecp_msg.message("Creating discode sensor...");
	ds_config_section_name = "[discode_sensor]";

	logger::log_dbg("Servo manager configured\n");

	ds_rpc = (boost::shared_ptr <ecp_mp::sensor::discode::discode_sensor>)
					new ecp_mp::sensor::discode::discode_sensor(ecp_t.config, ds_config_section_name);

	logger::log_dbg("Discode sensor created\n");

	ds_rpc->configure_sensor();

	logger::log_dbg("Discode sensor configured\n");

	gp = (boost::shared_ptr<common::generator::get_position>) new get_position(ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);
	logger::log_dbg("GetPosition generator configured\n");
}

void block_reaching::conditional_execution() {

	logger::log_dbg("In block_reaching::conditional_execution()\n");

	try {
		ecp_mp::sensor::discode::discode_sensor::discode_sensor_state st;
		st = ds_rpc->get_state();

		if(st == 0) sr_ecp_msg.message("sensor state 0");
		if(st == 1) sr_ecp_msg.message("sensor state 1");
		if(st == 2) sr_ecp_msg.message("sensor state 2");
		if(st == 3) sr_ecp_msg.message("sensor state 3");
		if(st == 4) sr_ecp_msg.message("sensor state 4");

		logger::log_dbg("Discode sensor state: %d\n", st);
	}
	catch(mrrocpp::ecp_mp::sensor::discode::ds_exception e){
		sr_ecp_msg.message("configure_discode error");
		sr_ecp_msg.message(e.what());
	}

	//remote procedure call
	sr_ecp_msg.message("Calling remote procedure...");
	uint32_t param = (int) ecp_t.mp_command.ecp_next_state.variant;
	Types::Mrrocpp_Proxy::BReading br;
	br = ds_rpc->call_remote_procedure <Types::Mrrocpp_Proxy::BReading>((int) param);
	if(br.rpcReceived) {
		sr_ecp_msg.message("Rpc received");
	}

	logger::log_dbg("RPC called\n");

	//run servo generator
	sr_ecp_msg.message("Object reaching...");
	sm->add_termination_condition(object_reached_term_cond);
	sm->add_termination_condition(timeout_term_cond);
	sm->Move();

	logger::log_dbg("Servovision finished\n");

	//object reached termination condition
	if (object_reached_term_cond->is_condition_met()) {
		sr_ecp_msg.message("object_reached_term_cond is met");
		ecp_t.ecp_reply.variant = 1;

		logger::log_dbg("Object reached term condition met\n");

		gp->Move();
		position_vector = gp->get_position_vector();

		for (size_t i = 0; i < position_vector.size(); ++i) {
			ecp_t.ecp_reply.sg_buf.data[i] = 0;
		}

		//printing position
		std::cout << "POSITION" << std::endl;
		for (size_t i = 0; i < position_vector.size(); ++i) {
			double pvim = (position_vector[i] + 5.0) * 10000;
			std::cout << position_vector[i] << ", " << pvim << std::endl;
			ecp_t.ecp_reply.sg_buf.data[i] = (int) pvim;
		}
		std::cout << std::endl;
		ecp_t.ecp_reply.variant = 1;
	}
	else {
		sr_ecp_msg.message("object_reached_term_cond IS NOT MET");
	}

	logger::log_dbg("After object reached termination condition check\n");

	//obsługa warunku zakończenia pracy - timeout
	if (timeout_term_cond->is_condition_met()) {
		sr_ecp_msg.message("timeout_term_cond is met");
		ecp_t.ecp_reply.variant = 0;
		logger::log_dbg("Timeout termination condition is met\n");
	} else {
		sr_ecp_msg.message("timeout_term_cond IS NOT MET");
	}

	sr_ecp_msg.message("servovision end");
	logger::log_dbg("Servovision end\n");
}

}
}
}
}
