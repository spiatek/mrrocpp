/*
 * ecp_g_position_board.cc
 *
 *  Created on: 10-02-2012
 *      Author: spiatek
 */

#include <vector>
#include <iostream>

#include "ecp_g_position_board.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

position_board::position_board(task::task & _ecp_t) :
		common::generator::newsmooth(_ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6)
{
	generator_name = ecp_mp::generator::ECP_GEN_POSITION_BOARD;
}

void position_board::conditional_execution() {

	sr_ecp_msg.message("configurate Smooth Generator...");

	//get position compute parameters
	ecp_bm_config_section_name = "[ecp_block_move]";
	offset = ecp_t.config.value <6, 1>("offset", ecp_bm_config_section_name);
	block_size = ecp_t.config.value <6, 1>("block_size", ecp_bm_config_section_name);
	correction = ecp_t.config.value <6, 1>("correction", ecp_bm_config_section_name);
	position = ecp_t.config.value <6, 1>("position", ecp_bm_config_section_name);

	int param = (int) ecp_t.mp_command.ecp_next_state.variant;

	sr_ecp_msg.message("after loading param from variant");

	int change_pos[6];

	change_pos[2] = param % 10;
	change_pos[1] = (param % 100 - change_pos[2]) / 10;
	change_pos[0] = (param % 1000 - change_pos[1]) / 100;
	change_pos[3] = 0;
	change_pos[4] = 0;
	change_pos[5] = 0;

	std::cout << "CHANGE_POSITION" << std::endl;
	for (size_t i = 0; i < 6; ++i) {
		std::cout << change_pos[i] << std::endl;
	}
	std::cout << std::endl;

	position_on_board(0, 0) = change_pos[0] - 1.0; //TODO: odwrócić planszę
	position_on_board(1, 0) = change_pos[1] - 1.0;
	position_on_board(2, 0) = change_pos[2] - 2.0;
	position_on_board(3, 0) = 0;
	position_on_board(4, 0) = 0;
	position_on_board(5, 0) = 0;

	correction_weights(0, 0) = (position_on_board(0, 0) == 3.0) ? 0 : 1;
	correction_weights(1, 0) = (position_on_board(1, 0) == 3.0) ? 0 : 1;
	correction_weights(2, 0) = 0;
	correction_weights(3, 0) = 0;
	correction_weights(4, 0) = 0;
	correction_weights(5, 0) = 0;

	sr_ecp_msg.message("after load coordinates");

	int do_move = 0; //czy zmieniac pozycje
	for (int i = 0; i < 6; ++i) {
		if (abs(position_on_board(i, 0)) < 4 && position_on_board(i, 0) >= 0) {
			do_move = 1;
		}
	}

	if (do_move == 1) {
		reset();
		set_absolute();

		std::vector <double> coordinates_vector(6);

		sr_ecp_msg.message("after reset and set_absoulute");

		std::cout << "POSITION ON BOARD" << std::endl;
		for (size_t i = 0; i < 6; ++i) {
			std::cout << position_on_board(i, 0) << std::endl;
		}
		std::cout << std::endl;

		for (size_t i = 0; i < 6; ++i) {
			coordinates_vector[i] = position(i) + offset(i, 0) + position_on_board(i, 0) * block_size(i, 0)
					+ correction(i, 0) * correction_weights(i, 0);
		}

		sr_ecp_msg.message("coordinates ready");

		load_absolute_angle_axis_trajectory_pose(coordinates_vector);

		sr_ecp_msg.message("pose loaded");

		if (calculate_interpolate()) {
			Move();
		}

		sr_ecp_msg.message("smooth generator configuration end");

	}

}

}
}
}
}
