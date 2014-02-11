/*
 * mp_t_block_planner.cc
 *
 *  Created on: 29-06-2013
 *      Author: spiatek
 */

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "mp_t_block_planner.h"
#include "BlockPosition.h"

#include "base/lib/mrmath/mrmath.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/configurator.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"

#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"

#include "../visual_servoing/visual_servoing.h"
#include "../visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"

#define WIDTH 4
#define BOARD_COLOR 5
#define BLOCK_SIZE 4
#define COORD_N 3
#define BLOCK_REACHING 0
#define BUILDING 1
#define COUNT_SERVO_TRY_1 5
#define COUNT_SERVO_TRY_2 5

typedef list <BlockPosition> block_position_list;

using namespace std;

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new block_planner(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void block_planner::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
}

block_planner::block_planner(lib::configurator &_config) :
		task(_config)
{
}

void block_planner::main_task_algorithm(void)
{
	sr_ecp_msg->message("Block Move MP Start");

	int board_localization = config.value <int>("board_localization", "[mp_block_planner]");

	if(board_localization == 1) {
		localize_board();
	}

	ClgPlanner *clg_p = new ClgPlanner();

	clg_p->initializeConnection(config.value <int>("port_nr", "[mp_block_planner]"));
	clg_p->checkConnection();

	int do_finish = 0;
	do {
		do_finish = clg_p->clgMrrocppCommunication(this);
		executeAction(clg_p->getCurrentAction());		//tu trzeba dodać obsługę parametrów akcji
		clg_p->sendResponse();							//to trzeba przemyśleć dokładniej
	}
	while(do_finish == 0);

	clg_p->closeConnection();
}

void block_planner::observ_action(void)
{

}

void block_planner::observe_color_action(void)
{
	int count_servo_try_2 = config.value <int>("count_servo_try_2", "[mp_block_planner]");

	for(int h = 0; h < count_servo_try_2; ++h) {

		wait_ms(1000);

		sr_ecp_msg->message("Start position");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_search_area_start.trj", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_ms(1000);

		sr_ecp_msg->message("Block localization - servovision");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, present_color, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		if (robot_m[lib::irp6p_m::ROBOT_NAME]->ecp_reply_package.variant == 1) {
			sr_ecp_msg->message("Block localized");
			break;
		}

		sr_ecp_msg->message("Block not localized!!!");
	}

	wait_ms(1000);
}

void block_planner::move_action(void)
{
	sr_ecp_msg->message("Reaching building place...");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_build_start.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	wait_ms(1000);

	//TODO: zmienić wyliczanie present position
	int param = 100 * present_position[0] + 10 * present_position[1] + present_position[2];

	sr_ecp_msg->message("Reaching position...");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_POSITION_BOARD, param, "", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	wait_ms(1000);
}

void block_planner::putdown_action(void)
{
	sr_ecp_msg->message("Force approach");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 600, 2), lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	wait_ms(4000);

	sr_ecp_msg->message("Raising up...");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/build.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Force approach");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, BUILDING, "", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Raising up...");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/up_after_pushing.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	wait_ms(1000);
}

void block_planner::pickup_action(void)
{
	/* Tu trzeba jeszcze wrócić do tej pozycji */

	sr_ecp_msg->message("Force approach");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.03, 800, 3), lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	wait_ms(1000);

	sr_ecp_msg->message("Go up");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/up_to_p0.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	wait_ms(1000);
}


void block_planner::localize_board(void)
{
	int count_servo_try_1 = config.value <int>("count_servo_try_1", "[mp_block_planner]");

	for (int h = 0; h < count_servo_try_1; ++h) {

		sr_ecp_msg->message("Reaching building place...");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_build_start.trj", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_ms(1000);

		sr_ecp_msg->message("Board localization - servovision");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, BOARD_COLOR, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		if (robot_m[lib::irp6p_m::ROBOT_NAME]->ecp_reply_package.variant == 1) {
			sr_ecp_msg->message("Board localized");
			break;
		}

		sr_ecp_msg->message("Board not localized!!!");
	}
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
