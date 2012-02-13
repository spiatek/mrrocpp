/*
 * ecp_g_block_reaching.cc
 *
 *  Created on: 2012-02-13
 *      Author: spiatek
 */

#include <iostream>

#include "ecp_g_block_reaching.h"

#define BOARD_COLOR 5

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
	reg = (boost::shared_ptr <servovision::visual_servo_regulator>) new servovision::regulator_p(_ecp_t.config, vs_config_section_name);
	ds = (boost::shared_ptr <ecp_mp::sensor::discode::discode_sensor>) new ecp_mp::sensor::discode::discode_sensor(_ecp_t.config, vs_config_section_name);
	vs = (boost::shared_ptr <servovision::visual_servo>) new servovision::ib_eih_visual_servo(reg, ds, vs_config_section_name, _ecp_t.config);
	object_reached_term_cond = (boost::shared_ptr <servovision::termination_condition>) new servovision::object_reached_termination_condition(_ecp_t.config, vs_config_section_name);
	timeout_term_cond = (boost::shared_ptr <servovision::termination_condition>) new servovision::timeout_termination_condition(timeout);
	sm = (boost::shared_ptr <common::generator::single_visual_servo_manager>) new common::generator::single_visual_servo_manager(_ecp_t, vs_config_section_name.c_str(), vs);
	sm->add_position_constraint(cube);
	sm->configure();
}

void block_reaching::conditional_execution() {

	gp = (boost::shared_ptr<common::generator::get_position>) new get_position(ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);

	//creating discode sensor for rpc communication
	sr_ecp_msg.message("Creating discode sensor...");
	ds_config_section_name = "[discode_sensor]";
	ds_rpc = (boost::shared_ptr <ecp_mp::sensor::discode::discode_sensor>) new ecp_mp::sensor::discode::discode_sensor(ecp_t.config, ds_config_section_name);
	ds_rpc->configure_sensor();

	//remote procedure call
	sr_ecp_msg.message("Calling remote procedure...");
	uint32_t param = (int) ecp_t.mp_command.ecp_next_state.variant;
	Types::Mrrocpp_Proxy::BReading br;
	br = ds_rpc->call_remote_procedure <Types::Mrrocpp_Proxy::BReading>((int) param);
	if (br.rpcReceived) {
		sr_ecp_msg.message("Rpc received");
	}

	//run servo generator
	sr_ecp_msg.message("Object reaching...");
	sm->add_termination_condition(object_reached_term_cond);
	sm->add_termination_condition(timeout_term_cond);
	sm->Move();

	//object reached termination condition
	if (object_reached_term_cond->is_condition_met()) {
		sr_ecp_msg.message("object_reached_term_cond is met");
		ecp_t.ecp_reply.variant = 1;

		if (param == BOARD_COLOR) {

			//getting position in ANGLE_AXIS coordinates
			gp->Move();
			position_vector = gp->get_position_vector();

			/*if (!position.size()) {
				sr_ecp_msg.message("get_position_vector is empty");
			}*/

			//printing position
			std::cout << "POSITION" << std::endl;
			for (size_t i = 0; i < position_vector.size(); ++i) {
				std::cout << position_vector[i] << std::endl;
			}
			std::cout << std::endl;
		}
	} else {
		sr_ecp_msg.message("object_reached_term_cond IS NOT MET");
	}

	//obsługa warunku zakończenia pracy - timeout
	if (timeout_term_cond->is_condition_met()) {
		sr_ecp_msg.message("timeout_term_cond is met");
		ecp_t.ecp_reply.variant = 0;
	} else {
		sr_ecp_msg.message("timeout_term_cond IS NOT MET");
	}

	sr_ecp_msg.message("servovision end");
}

}
}
}
}
