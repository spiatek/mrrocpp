/*
 * ecp_g_position_board.h
 *
 *  Created on: 10-02-2012
 *      Author: spiatek
 */

#ifndef ECP_G_POSITION_BOARD_H_
#define ECP_G_POSITION_BOARD_H_

#include "ecp_mp_g_position_board.h"

#include "base/ecp/ecp_generator.h"
#include "generator/ecp/ecp_g_newsmooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class position_board : public common::generator::newsmooth
{
private:

	std::string ecp_bm_config_section_name;

	Eigen::Matrix <double, 6, 1> offset;
	Eigen::Matrix <double, 6, 1> correction;
	Eigen::Matrix <double, 6, 1> block_size;
	Eigen::Matrix <double, 6, 1> position;

	Eigen::Matrix <double, 6, 1> coordinates;
	Eigen::Matrix <double, 6, 1> position_on_board;
	Eigen::Matrix <double, 6, 1> correction_weights;

public:

	position_board(task::task & _ecp_t);
	void conditional_execution();
};

}
}
}
}

#endif /* ECP_G_POSITION_BOARD_H_ */
