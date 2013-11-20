#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <list>
#include <map>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/ecp_mp/ecp_mp_sensor.h"
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"
#include "mp_rubic_cube_observer.h"
#include "cube_face.h"
#include "ecp_mp_tr_rc_windows.h"
#include "robot/festival/ecp_g_festival.h"
#include "robot/festival/ecp_mp_t_festival.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/constant_velocity/ecp_mp_g_constant_velocity.h"
#include "generator/ecp/weight_measure/ecp_mp_g_weight_measure.h"
#include "robot/festival/const_festival.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/force_tool_change/ecp_mp_g_force_tool_change.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
#include "generator/ecp/tff_rubik_face_rotate/ecp_mp_g_tff_rubik_face_rotate.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;
using mrrocpp::ecp_mp::sensor::discode::ds_exception;

namespace mrrocpp {
namespace mp {
namespace task {

void rubik_cube_observer::initiate(common::CUBE_COLOR up_is, common::CUBE_COLOR down_is, common::CUBE_COLOR front_is, common::CUBE_COLOR rear_is, common::CUBE_COLOR left_is, common::CUBE_COLOR right_is)
{
	cube_state = new common::CubeState(up_is, down_is, front_is, rear_is, left_is, right_is);
	manipulation_sequence_computed = false;
}

rubik_cube_observer::rubik_cube_observer(lib::configurator &_config) :
		task(_config), cube_state(NULL)
{
	// Powolanie czujnikow

	//odczyt z configa
	vis_servoing = config.value <int>("vis_servoing");

	if (vis_servoing)
	{
		BOOST_FOREACH (ecp_mp::sensor_item_t & s, sensor_m)
				{
					s.second->configure_sensor();
				}
	}

	if (vis_servoing) {
		// dodanie transmitter'a
		transmitter_m[ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS] =
				new ecp_mp::transmitter::rc_windows(ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);

	}
	configure_discode();
}

rubik_cube_observer::~rubik_cube_observer()
{
	if (cube_state)
		delete cube_state;
}

void rubik_cube_observer::configure_discode()
{
	sr_ecp_msg->message("configure_discode");
	char config_section_name[]= {"[Discode_TEST]"};
	discode = boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(config, config_section_name));

	try {
		discode->configure_sensor();
		sr_ecp_msg->message("sensor_ready");

		discode_sensor::discode_sensor_state st;
		st = discode->get_state();

		if(st == 0) sr_ecp_msg->message("sensor state 0");
		if(st == 1) sr_ecp_msg->message("sensor state 1");
		if(st == 2) sr_ecp_msg->message("sensor state 2");
		if(st == 3) sr_ecp_msg->message("sensor state 3");
		if(st == 4) sr_ecp_msg->message("sensor state 4");


	}catch(mrrocpp::ecp_mp::sensor::discode::ds_exception e){
		sr_ecp_msg->message("configure_discode error");
		sr_ecp_msg->message(e.what());
	}
}

Types::Mrrocpp_Proxy::CubeReading rubik_cube_observer::read_from_discode()
{
	sr_ecp_msg->message("read_from_discode");

	Types::Mrrocpp_Proxy::CubeReading reading;

	try {
		discode->get_reading();
		wait_ms(500);
		reading = discode->retreive_reading <Types::Mrrocpp_Proxy::CubeReading>();
		sr_ecp_msg->message("reading recived");

	} catch (ds_exception e){
		sr_ecp_msg->message("read_from_discode error");
		sr_ecp_msg->message(e.what());
	}

	return reading;
}

void rubik_cube_observer::identify_colors() //DO WIZJI (przekladanie i ogladanie scian)
{
	//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
	//cube_initial_state=BGROWY

	// manianka
	cube_state->set_state(common::BLUE, common::GREEN, common::RED, common::ORANGE, common::WHITE, common::YELLOW);

	const common::CUBE_TURN_ANGLE changing_order[] =
			{ common::CL_0, common::CL_0, common::CL_180, common::CL_0, common::CL_180, common::CL_0 };

	for (int k = 0; k < 6; k++) {
		face_turn_op(common::CL_0);

		//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "oglo~dam kolory na s~ciance", 0, lib::festival::ROBOT_NAME);

		// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
		// wait_for_task_termination(false, 1, lib::festival::ROBOT_NAME.c_str());

		if (vis_servoing) {
			ecp_mp::sensor::sensor <mrrocpp::mp::task::cube_face_t> * cube_recognition =
					dynamic_cast <ecp_mp::sensor::sensor <mrrocpp::mp::task::cube_face_t> *>(sensor_m[mrrocpp::ecp_mp::sensor::SENSOR_CAMERA_ON_TRACK]);

			wait_ms(5000);
			cube_recognition->initiate_reading();
			wait_ms(1000);
			cube_recognition->get_reading();

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					cube_state->cube_tab[k][3 * i + j] = cube_recognition->image.colors[3 * i + j];
		}

		printf("\nFACE FACE %d:\n", k);
		flushall();

		for (int i = 0; i < 9; i++) {
			switch (cube_state->cube_tab[k][i])
			{
				case 1:
					cube_state->cube_tab[k][i] = 'r';
					printf("R");
					break;
				case 2:
					cube_state->cube_tab[k][i] = 'o';
					printf("O");
					break;
				case 3:
					cube_state->cube_tab[k][i] = 'y';
					printf("Y");
					break;
				case 4:
					cube_state->cube_tab[k][i] = 'g';
					printf("G");
					break;
				case 5:
					cube_state->cube_tab[k][i] = 'b';
					printf("B");
					break;
				case 6:
					cube_state->cube_tab[k][i] = 'w';
					printf("W");
					break;
				default:
					cube_state->cube_tab[k][i] = 'o';
					printf("?");
					break;
			}
			if (cube_state->cube_tab[k][i] != 'o') {
				putchar(std::toupper(cube_state->cube_tab[k][i]));
			} else {
				putchar('?');
			}
		}
		printf("\n");

		wait_ms(1000);
		face_change_op(changing_order[k]);

	} //for
}

bool rubik_cube_observer::communicate_with_windows_observer()
{
	char cube_tab_send[55];
	char manipulation_sequence[200];

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[2 * 9 + 3 * i + j] = cube_state->cube_tab[0][3 * i + j]; //rot cl 0

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[1 * 9 + 3 * j + 2 - i] = cube_state->cube_tab[1][3 * i + j]; //rot cl 90

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[3 * 9 + 3 * (2 - j) + i] = cube_state->cube_tab[2][3 * i + j]; //rot ccl 90

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[5 * 9 + 3 * i + j] = cube_state->cube_tab[3][3 * i + j]; //rot cl 0

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[4 * 9 + 3 * j + 2 - i] = cube_state->cube_tab[4][3 * i + j]; //rot cl 90

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[0 * 9 + 3 * j + 2 - i] = cube_state->cube_tab[5][3 * i + j]; //rot cl 90

	printf("SEQ IN COLOR : %s\n", cube_tab_send);

	char c_up = cube_tab_send[4];
	char c_right = cube_tab_send[13];
	char c_front = cube_tab_send[22];
	char c_down = cube_tab_send[31];
	char c_left = cube_tab_send[40];
	char c_back = cube_tab_send[49];

	printf("%c %c %c %c %c %c\n", c_up, c_right, c_front, c_down, c_left, c_back);

	for (int i = 0; i < 54; i++) {
		if (cube_tab_send[i] == c_up)
			cube_tab_send[i] = 'u';
		else if (cube_tab_send[i] == c_down)
			cube_tab_send[i] = 'd';
		else if (cube_tab_send[i] == c_front)
			cube_tab_send[i] = 'f';
		else if (cube_tab_send[i] == c_back)
			cube_tab_send[i] = 'b';
		else if (cube_tab_send[i] == c_right)
			cube_tab_send[i] = 'r';
		else if (cube_tab_send[i] == c_left)
			cube_tab_send[i] = 'l';
	}

	/*
	 for(int i=0; i<54; i++)
	 {
	 switch (cube_tab_send[i])
	 {
	 case 'b': cube_tab_send[i]='u'; break;
	 case 'g': cube_tab_send[i]='d'; break;
	 case 'o': cube_tab_send[i]='f'; break;
	 case 'r': cube_tab_send[i]='b'; break;
	 case 'y': cube_tab_send[i]='r'; break;
	 case 'w': cube_tab_send[i]='l'; break;
	 }
	 }
	 */

	cube_tab_send[54] = '\0';

	printf("SEQ FROM VIS : %s\n", cube_tab_send);

	//reszta

	// czyszczenie listy
	manipulation_list.clear();

	ecp_mp::transmitter::transmitter_base * transmitter_ptr = transmitter_m[ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS];
	assert(transmitter_ptr);

	ecp_mp::transmitter::rc_windows * rc_observer_ptr = dynamic_cast <ecp_mp::transmitter::rc_windows *>(transmitter_ptr);
	assert(rc_observer_ptr);

	ecp_mp::transmitter::rc_windows & rc_observer = *rc_observer_ptr;

	for (int i = 0; i < 54; i++) {
		rc_observer.to_va.rc_state[i] = cube_tab_send[i];
	}
	rc_observer.to_va.rc_state[54] = '\0';

	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "mys~le~", 0, lib::festival::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	// wait_for_task_termination(false, 1, lib::festival::ROBOT_NAME.c_str());

	rc_observer.t_write();

	rc_observer.t_read(true);

	printf("OPS: %s", rc_observer.from_va.sequence);

	strcpy(manipulation_sequence, rc_observer.from_va.sequence);

	if ((manipulation_sequence[0] == 'C') && (manipulation_sequence[1] == 'u') && (manipulation_sequence[2] == 'b')
			&& (manipulation_sequence[3] == 'e')) {
		printf("Jam jest daltonista. ktory Ci nie uloz*y kostki\n");
		manipulation_sequence_computed = false;
		return false;
	}

	//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
	//cube_initial_state=BGROWY

	int s = 0;
	int str_size = 0;
	for (unsigned int char_i = 0; char_i < strlen(rc_observer.from_va.sequence) - 1; char_i++) {
		if (s == 0) {
			switch (rc_observer.from_va.sequence[char_i])
			{
				case 'U':
					manipulation_sequence[str_size] = 'B';
					break;
				case 'D':
					manipulation_sequence[str_size] = 'G';
					break;
				case 'F':
					manipulation_sequence[str_size] = 'O';
					break;
				case 'B':
					manipulation_sequence[str_size] = 'R';
					break;
				case 'L':
					manipulation_sequence[str_size] = 'W';
					break;
				case 'R':
					manipulation_sequence[str_size] = 'Y';
					break;
			}
			s = 1;
			str_size++;
		} else if (s == 1) {
			switch (rc_observer.from_va.sequence[char_i])
			{
				case ' ':
					manipulation_sequence[str_size] = '1';
					s = 0;
					break;
				case '2':
					manipulation_sequence[str_size] = '2';
					s = 2;
					break;
				case '\'':
					manipulation_sequence[str_size] = '3';
					s = 2;
					break;
			}
			str_size++;
		} else if (s == 2) {
			s = 0;
		}

	}

	if (s == 1) {
		str_size--;
		manipulation_sequence[str_size] = '1';
		str_size++;
	}
	manipulation_sequence[str_size] = '\0';

	printf("\n%d %zd\n", str_size, strlen(manipulation_sequence));
	printf("SEQ from win %s\n", rc_observer.from_va.sequence);
	printf("\nSEQ2 %s\n", manipulation_sequence);

	//pocztaek ukladania
	// dodawanie manipulacji do listy
	for (unsigned int char_i = 0; char_i < strlen(manipulation_sequence) - 1; char_i += 2) {
		manipulation_list.push_back(common::SingleManipulation(common::read_cube_color(manipulation_sequence[char_i]), common::read_cube_turn_angle(manipulation_sequence[char_i+ 1])));
	}

	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "juZ ukl/adam", 0, lib::festival::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	// wait_for_task_termination(false, 1, lib::festival::ROBOT_NAME.c_str());

	manipulation_sequence_computed = true;

	return false;
}

void rubik_cube_observer::execute_manipulation_sequence()
{
	for (std::list <common::SingleManipulation>::iterator manipulation_list_iterator = manipulation_list.begin();manipulation_list_iterator != manipulation_list.end(); manipulation_list_iterator++)
	{
		manipulate(manipulation_list_iterator->face_to_turn, manipulation_list_iterator->turn_angle);
	}
}

void rubik_cube_observer::manipulate(common::CUBE_COLOR face_to_turn, common::CUBE_TURN_ANGLE turn_angle)
{

	if (face_to_turn == cube_state->up)
	{
		// printf("cube_state->up\n");
		face_change_op(common::CL_90);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->down)
	{
		// printf("cube_state->down\n");
		face_change_op(common::CCL_90);
		face_turn_op(turn_angle);

	}
	else if (face_to_turn == cube_state->front)
	{
		// printf("cube_state->front\n");
		face_change_op(common::CL_0);
		face_turn_op(common::CL_0);
		face_change_op(common::CL_90);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->rear)
	{
		// printf("cube_state->rear\n");
		face_change_op(common::CL_0);
		face_turn_op(common::CL_0);
		face_change_op(common::CCL_90);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->left)
	{
		// printf("cube_state->left\n");
		face_change_op(common::CL_0);
		face_turn_op(turn_angle);
	}
	else if (face_to_turn == cube_state->right)
	{
		// printf("cube_state->right\n");
		face_change_op(common::CL_180);
		face_turn_op(turn_angle);
	}
}
//obliczanie układu pól
void rubik_cube_observer::calculate_color_combination()
{
	//Robimy listę wszystkich
	std::vector<FieldOfCube*> colorsOfCube;
	colorsOfCube.clear();

	//Tworzenie listy wskaźników do każdego pola
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int l=0;l<3;l++)
			{
				wallsOfCube[i].field[j][l].numberOfColor=-1;
				FieldOfCube *color= &wallsOfCube[i].field[j][l];
				colorsOfCube.push_back(color);
			}
		}
	}

	for(int i=0;i<6;i++)
	{
		FieldOfCube *colorSuitTo=colorsOfCube[0];
		int maxR=-1;
		int noOfMaxR=-1;
		for(int l=0;l<colorsOfCube.size();l++)
		{
			if(colorsOfCube[l]->numberOfColor==-1)
			{
				int r=pow(colorsOfCube[l]->color[0],3)+pow(colorsOfCube[l]->color[1],3)+pow(colorsOfCube[l]->color[2],3);
				if(r>maxR)
				{
					maxR=r;
					noOfMaxR=l;
				}
			}
		}
		colorSuitTo=colorsOfCube[noOfMaxR];
		FieldOfCube *suitedColors[9];
		double distancesSuitedColors[9];

		for (int j=0;j<9;j++) distancesSuitedColors[j]=9999;

		for(int l=0;l<colorsOfCube.size();l++)
		{
				if(colorsOfCube[l]->numberOfColor!=-1) continue;

				double r=pow(abs((double)colorsOfCube[l]->color[0]-colorSuitTo->color[0]),3)+pow(abs((double)colorsOfCube[l]->color[1]-colorSuitTo->color[1]),3)+pow(abs((double)colorsOfCube[l]->color[2]-colorSuitTo->color[2]),3);
				r=pow(r,1.0/3);

				double biggestR=-1;
				int noOfBiggest=0;

				for(int j=0;j<9;j++)
				{
					if(biggestR<distancesSuitedColors[j])
					{
						noOfBiggest=j;
						biggestR=distancesSuitedColors[j];
					}
				}

				if(r<distancesSuitedColors[noOfBiggest])
				{
					suitedColors[noOfBiggest]=colorsOfCube[l];
					distancesSuitedColors[noOfBiggest]=r;
				}
		}

		for(int j=0;j<9; j++)
		{
			if(suitedColors[j]!=NULL) suitedColors[j]->numberOfColor=i;
			else {sr_ecp_msg->message("Something wierd happened in marking colors");wait_ms(200);}
		}

	}
}

//pokazanie sciany
void rubik_cube_observer::look_at_wall(common::CUBE_WALL wall,bool isAfterChanging)
{
	switch(wall)
	{
		case common::FRONT_WALL:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_front.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_front.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
			break;
		case common::UP_SIDE_WALL:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_up_side.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_side.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
			break;
		case common::DOWN_SIDE_WALL:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_down_side.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_side.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
			break;
		case common::LEFT_SIDE_WALL:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_left_side.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_side.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
			break;
		case common::RIGHT_SIDE_WALL:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_right_side.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_side.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
			break;
		default:
			break;
	}
	Types::Mrrocpp_Proxy::CubeReading pbr;
	while(1)
		{
			pbr=read_from_discode();
			if(pbr.exist==true) break;
		}
	switch(wall)
	{
		case common::FRONT_WALL:
			for(int i=0; i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					if(isAfterChanging)
					{
						wallsOfCube[5].field[i][j].color[0]=pbr.color[i][j][0];
						wallsOfCube[5].field[i][j].color[1]=pbr.color[i][j][1];
						wallsOfCube[5].field[i][j].color[2]=pbr.color[i][j][2];
					}
					else
					{
						wallsOfCube[0].field[i][j].color[0]=pbr.color[i][j][0];
						wallsOfCube[0].field[i][j].color[1]=pbr.color[i][j][1];
						wallsOfCube[0].field[i][j].color[2]=pbr.color[i][j][2];
					}
				}
			}
			break;
		case common::UP_SIDE_WALL:
			if(!isAfterChanging)
			{
				for(int i=0; i<3;i++)
				{
					for(int j=0;j<3;j++)
					{
						 if(!(i==0 && (j==1 || j==2)))
						{
							wallsOfCube[1].field[i][j].color[0]=pbr.color[i][j][0];
							wallsOfCube[1].field[i][j].color[1]=pbr.color[i][j][1];
							wallsOfCube[1].field[i][j].color[2]=pbr.color[i][j][2];
						}
					}
				}
			}
			else
			{
				wallsOfCube[1].field[0][2].color[0]=pbr.color[2][0][0];
				wallsOfCube[1].field[0][2].color[1]=pbr.color[2][0][1];
				wallsOfCube[1].field[0][2].color[2]=pbr.color[2][0][2];

				wallsOfCube[1].field[0][1].color[0]=pbr.color[2][1][0];
				wallsOfCube[1].field[0][1].color[1]=pbr.color[2][1][1];
				wallsOfCube[1].field[0][1].color[2]=pbr.color[2][1][2];
			}
			break;
		case common::DOWN_SIDE_WALL:
			if(!isAfterChanging)
			{
				for(int i=0; i<3;i++)
				{
					for(int j=0;j<3;j++)
					{
						if(!(i==0 && (j==1 || j==2)))
						{
							wallsOfCube[3].field[i][j].color[0]=pbr.color[i][j][0];
							wallsOfCube[3].field[i][j].color[1]=pbr.color[i][j][1];
							wallsOfCube[3].field[i][j].color[2]=pbr.color[i][j][2];
						}
					}
				}
			}
			else
			{
				wallsOfCube[3].field[0][2].color[0]=pbr.color[2][0][0];
				wallsOfCube[3].field[0][2].color[1]=pbr.color[2][0][1];
				wallsOfCube[3].field[0][2].color[2]=pbr.color[2][0][2];

				wallsOfCube[3].field[0][1].color[0]=pbr.color[2][1][0];
				wallsOfCube[3].field[0][1].color[1]=pbr.color[2][1][1];
				wallsOfCube[3].field[0][1].color[2]=pbr.color[2][1][2];
			}
			break;
		case common::LEFT_SIDE_WALL:
			if(!isAfterChanging)
			{
				for(int i=0; i<3;i++)
				{
					for(int j=0;j<3;j++)
					{
						if(!(i==2 && (j==1 || j==2)))
						{
							wallsOfCube[2].field[i][j].color[0]=pbr.color[i][j][0];
							wallsOfCube[2].field[i][j].color[1]=pbr.color[i][j][1];
							wallsOfCube[2].field[i][j].color[2]=pbr.color[i][j][2];
						}
					}
				}
			}
			else
			{
				wallsOfCube[4].field[2][1].color[0]=pbr.color[0][1][0];
				wallsOfCube[4].field[2][1].color[1]=pbr.color[0][1][1];
				wallsOfCube[4].field[2][1].color[2]=pbr.color[0][1][2];

				wallsOfCube[4].field[2][2].color[0]=pbr.color[0][0][0];
				wallsOfCube[4].field[2][2].color[1]=pbr.color[0][0][1];
				wallsOfCube[4].field[2][2].color[2]=pbr.color[0][0][2];
			}
			break;
		case common::RIGHT_SIDE_WALL:
			if(!isAfterChanging)
			{
				for(int i=0; i<3;i++)
				{
					for(int j=0;j<3;j++)
					{
						if(!(i==2 && (j==1 || j==2)))
						{
							wallsOfCube[4].field[i][j].color[0]=pbr.color[i][j][0];
							wallsOfCube[4].field[i][j].color[1]=pbr.color[i][j][1];
							wallsOfCube[4].field[i][j].color[2]=pbr.color[i][j][2];
						}
					}
				}
			}
			else
			{
				wallsOfCube[2].field[2][1].color[0]=pbr.color[0][1][0];
				wallsOfCube[2].field[2][1].color[1]=pbr.color[0][1][1];
				wallsOfCube[2].field[2][1].color[2]=pbr.color[0][1][2];

				wallsOfCube[2].field[2][2].color[0]=pbr.color[0][0][0];
				wallsOfCube[2].field[2][2].color[1]=pbr.color[0][0][1];
				wallsOfCube[2].field[2][2].color[2]=pbr.color[0][0][2];
			}
			break;
		default:
			break;
	}
	/*sr_ecp_msg->message("JEST KOSTKAJESTKOSTKA");
	std::string s;
	for (int i =0;i<3;i++)
	{
		for (int j =0;j<3;j++)
		{
			for(int l=0;l<3;l++)
			{
				s = boost::lexical_cast<std::string>( pbr.color[i][j][l] );//;
				sr_ecp_msg->message(s);
				wait_ms(500);
			}
		}
	}*/
	//std::cout << pbr.color[i][j][0] << " "<< pbr.color[i][j][1] << " " << pbr.color[i][j][2] << "\n" ;

	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_begin_end.trj", lib::irp6ot_m::ROBOT_NAME);
	//wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_begin_end.trj", lib::irp6p_m::ROBOT_NAME);
	//wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
}
// obrot sciany
void rubik_cube_observer::face_turn_op(common::CUBE_TURN_ANGLE turn_angle)
{

	// ustawienie chwytakow we wlasciwej wzajemnej orientacji

	//	printf("face_turn_op_CL: %d\n", turn_angle);

	// wlaczenie generatora uczacego w obu robotach
	switch (turn_angle)
	{
		case common::CL_90:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_cl_90_phase_1.trj", lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fturn_ap_cl_90_phase_1.trj", lib::irp6p_m::ROBOT_NAME);
			break;
		case common::CL_0:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_cl_0_phase_1.trj", lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fturn_ap_cl_0_phase_1.trj", lib::irp6p_m::ROBOT_NAME);
			break;
		case common::CCL_90:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_ccl_90_phase_1.trj", lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fturn_ap_ccl_90_phase_1.trj", lib::irp6p_m::ROBOT_NAME);
			break;
		case common::CL_180:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_cl_180_phase_1.trj", lib::irp6ot_m::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fturn_ap_cl_180_phase_1.trj", lib::irp6p_m::ROBOT_NAME);
			break;
		default:
			break;
	}
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.089, lib::irp6p_tfg::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME, lib::irp6p_tfg::ROBOT_NAME);

	// zblizenie chwytaka tracka do nieruchomego chwytaka postumenta

	switch (turn_angle)
	{
		case common::CL_90:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_cl_90_phase_2.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		case common::CL_0:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_cl_0_phase_2.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		case common::CCL_90:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_ccl_90_phase_2.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		case common::CL_180:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fturn_ap_cl_180_phase_2.trj", lib::irp6ot_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		default:
			break;
	}

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// zacisniecie postumenta na kostce

	// wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, true);

	// uruchomienie tff_nose run dla traka z podatnoscia w jednej osi
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(false, true, false, false, false, false), lib::irp6p_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej zacisnieciu na kostce z pol centymentrowym luzem
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.068, lib::irp6p_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6p_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6p_m::ROBOT_NAME);

	// uruchomienie tff_nose run dla traka z podatnoscia w dwoch osiach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(false, true, true, false, false, false), lib::irp6p_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej calkowitemu zacisnieciu na kostce bez luzu
    set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.058, lib::irp6p_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6p_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6p_m::ROBOT_NAME);

	// obrot kostki
	switch (turn_angle)
	{
		case common::CL_90:
			//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "obracam kostke~", 0, lib::festival::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) ecp_mp::generator::RCSC_CL_90, "", lib::irp6ot_m::ROBOT_NAME);
			// uruchomienie generatora empty_gen
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			// wait_for_task_termination(false, 2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::festival::ROBOT_NAME.c_str());
			break;
		case common::CL_0:
			break;
		case common::CCL_90:
			//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "obracam kostke~", 0, lib::festival::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) ecp_mp::generator::RCSC_CCL_90, "", lib::irp6ot_m::ROBOT_NAME);
			// uruchomienie generatora empty_gen
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			//wait_for_task_termination(false, {lib::irp6ot_m::ROBOT_NAME, lib::festival::ROBOT_NAME});
			break;
		case common::CL_180:
			//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "obracam kostke~", 0, lib::festival::ROBOT_NAME);
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE, (int) ecp_mp::generator::RCSC_CL_180, "", lib::irp6ot_m::ROBOT_NAME);
			// uruchomienie generatora empty_gen
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
			//wait_for_task_termination(false, {lib::irp6ot_m::ROBOT_NAME, lib::festival::ROBOT_NAME});
			break;
		default:
			break;
	}

	// rozwarcie chwytaka tracka

	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::RELATIVE, 0.022, lib::irp6ot_tfg::ROBOT_NAME);

	// uruchomienie generatora empty_gen
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	// odejscie tracka od postumenta
        set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_departure.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
        //set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rcsc/trj/irp6ot_sm_fturn_de.trj", lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
}

// zmiana sciany (przelozenie kostki)
void rubik_cube_observer::face_change_op(common::CUBE_TURN_ANGLE turn_angle)
{

	// zblizenie chwytakow

	//	printf("face_turn_op_CL: %d\n", turn_angle);

	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "przekl/adam kostke~", 0, lib::festival::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP

	// wlaczenie generatora uczacego w obu robotach
	switch (turn_angle)
	{
		case common::CL_90:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_cl_90_phase_1.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fchange_ap_cl_90_phase_1.trj", lib::irp6p_m::ROBOT_NAME.c_str());
			break;
		case common::CL_0:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_cl_0_phase_1.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fchange_ap_cl_0_phase_1.trj", lib::irp6p_m::ROBOT_NAME.c_str());
			break;
		case common::CCL_90:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_ccl_90_phase_1.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fchange_ap_ccl_90_phase_1.trj", lib::irp6p_m::ROBOT_NAME.c_str());
			break;
		case common::CL_180:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_cl_180_phase_1.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_fchange_ap_cl_180_phase_1.trj", lib::irp6p_m::ROBOT_NAME.c_str());
			break;
		default:
			break;
	}

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.089, lib::irp6ot_tfg::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME, lib::irp6ot_tfg::ROBOT_NAME);

	switch (turn_angle)
	{
		case common::CL_90:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_cl_90_phase_2.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		case common::CL_0:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_cl_0_phase_2.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		case common::CCL_90:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_ccl_90_phase_2.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		case common::CL_180:
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_ap_cl_180_phase_2.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME.c_str());
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, (int) lib::RELATIVE, "../../src/application/rubic_cube_observer/trj/irp6ot_cube_approach.trj", lib::irp6ot_m::ROBOT_NAME.c_str());
			break;
		default:
			break;
	}
	// TODO: wstawic generator VS + zblizanie silowe

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// zacisniecie tracka na kostce

	// wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, true);

	// uruchomienie tff_nose run dla traka z podatnoscia w jednej osi
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(false, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej zacisnieciu na kostce z pol centymentrowym luzem
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.072, lib::irp6ot_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie tff_nose run dla traka z podatnoscia w dwoch osiach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej calkowitemu zacisnieciu na kostce bez luzu
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.065, lib::irp6ot_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);

	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::generator::RCSC_RG_FCHANGE_PHASE_1, "", lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	//wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::generator::RCSC_RG_FCHANGE_PHASE_2, "", lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	//wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// docisniecie chwytaka tracka do kostki

	// uruchomienie tff_nose run dla traka z podatnoscia w dwoch osiach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.01, 1000, 3), lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie generatora empty_gen
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// zacisniecie tracka na kostce

	// uruchomienie tff_nose run dla traka z podatnoscia w dwoch osiach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej calkowitemu zacisnieciu na kostce bez luzu
        set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.058, lib::irp6ot_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);

	// wstepne rozwarcie chwytaka postumenta
	//set_next_ecp_state ((int) ecp_mp::subtask::ECP_ST_GRIPPER_OPENING, (int) ecp_mp::generator::RCSC_GO_VAR_1, "",  lib::irp6p_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::RELATIVE, 0.004, lib::irp6p_tfg::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	wait_for_task_termination(false, lib::irp6p_tfg::ROBOT_NAME);

	// ostateczne zacisniecie tracka na kostce
	// wlaczenie generatora zacisku na kostce w robocie irp6ot
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_RUBIK_GRAB, (int) ecp_mp::generator::RCSC_RG_FCHANGE_PHASE_4, "", lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	//wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// dalsze rozwarcie chwytaka postumenta
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::RELATIVE, 0.023, lib::irp6p_tfg::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	wait_for_task_termination(false, lib::irp6p_tfg::ROBOT_NAME);

	// odejscie tracka od postumenta
	switch (turn_angle)
	{
		case common::CL_90:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_de_cl_90.trj", lib::irp6ot_m::ROBOT_NAME);
			break;
		case common::CL_0:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_de_cl_0.trj", lib::irp6ot_m::ROBOT_NAME);
			break;
		case common::CCL_90:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_de_ccl_90.trj", lib::irp6ot_m::ROBOT_NAME);
			break;
		case common::CL_180:

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_fchange_de_cl_180.trj", lib::irp6ot_m::ROBOT_NAME);
			break;
		default:
			break;
	}

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// zmiana stanu kostki

	common::CubeState tmp_cube_state;

	switch (turn_angle)
	{
		case common::CL_90:
			tmp_cube_state.set_state(cube_state->left, cube_state->right, cube_state->up, cube_state->down, cube_state->front, cube_state->rear);
			break;
		case common::CL_0:
			tmp_cube_state.set_state(cube_state->front, cube_state->rear, cube_state->left, cube_state->right, cube_state->up, cube_state->down);
			break;
		case common::CCL_90:
			tmp_cube_state.set_state(cube_state->right, cube_state->left, cube_state->down, cube_state->up, cube_state->front, cube_state->rear);
			break;
		case common::CL_180:
			tmp_cube_state.set_state(cube_state->front, cube_state->rear, cube_state->right, cube_state->left, cube_state->down, cube_state->up);
			break;
		default:
			break;
	}

	*cube_state = tmp_cube_state;

	//	cube_state->print_cube_colors();

}

void rubik_cube_observer::configure_edp_force_sensor(bool configure_track, bool configure_postument)
{
	if (configure_track) {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
	}

	if (configure_postument) {
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6p_m::ROBOT_NAME);
	}

	if ((configure_track) && (!configure_postument)) {
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	} else if ((!configure_track) && (configure_postument)) {
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
	} else if ((configure_track) && (configure_postument)) {
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);
	}
}

// dojscie
void rubik_cube_observer::approach_op(int mode)
{

	//pierwsza konfiguracja czujnikow
	//wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, true);

	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, ecp::festival::generator::generator::POLISH_VOICE, "jestem podatny", 0, lib::festival::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
	//wait_for_task_termination(false, 1, lib::festival::ROBOT_NAME.c_str());

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.070, lib::irp6ot_tfg::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.075, lib::irp6p_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME, lib::irp6p_tfg::ROBOT_NAME);

	if (config.exists_and_true("irp6p_compliant")) {
		// wlaczenie genrator tff_nose_run_generator w postumencie

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, 0, "", lib::irp6p_m::ROBOT_NAME);

		// uruchomienie generatora empty_gen
		wait_for_task_termination(true, lib::irp6p_m::ROBOT_NAME);

		// przerwanie pracy generatora w ECP
		send_end_motion_to_ecps(lib::irp6p_m::ROBOT_NAME);
	} else {
		sr_ecp_msg->message("trak podatny");
		// wlaczenie genrator tff_nose_run_generator w tracku

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, true, true, true, true), lib::irp6ot_m::ROBOT_NAME);

		// uruchomienie generatora empty_gen
		wait_for_task_termination(true, lib::irp6ot_m::ROBOT_NAME);

		// przerwanie pracy generatora w ECP
		send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);
	}

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_ap_1.trj", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_ap_1.trj", lib::irp6p_m::ROBOT_NAME);

	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej zacisnieciu na kostce z pol centymentrowym luzem
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.075, lib::irp6ot_tfg::ROBOT_NAME);

	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "jestem robotem usl/ugowym", 0, lib::festival::ROBOT_NAME);

	//zadanie chwytania kostki

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_ap_2.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6ot_tfg::ROBOT_NAME);

	//wait_for_task_termination(false, 2, lib::festival::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());
	/*
	 //generator sledzacy kostke
	 set_next_ecp_state(ecp_mp::generator::ECP_GEN_IB_EIH, (int) 1, "", lib::irp6ot_m::ROBOT_NAME);

	 //wait_for_task_termination(false, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	 */

	wait_ms(500);

	//wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, false);

	// docisniecie chwytaka tracka do kostki
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.01, 1000, 3), lib::irp6ot_m::ROBOT_NAME);
	//wait_for_task_termination(false, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	//podnoszenie o 2 milimetry nad kostke
	// uruchomienie tff_nose run dla traka z podatnoscia w jednej osi
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(false, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej zacisnieciu na kostce z pol centymentrowym luzem
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.068, lib::irp6ot_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.01, 1000, 3), lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie tff_nose run dla traka z podatnoscia w dwoch osiach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, false, false, false, false), lib::irp6ot_m::ROBOT_NAME);

	// uruchomienie zaciskania chwytaka traka do pozycji zadanej odpowiadajacej calkowitemu zacisnieciu na kostce bez luzu
        set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.058, lib::irp6ot_tfg::ROBOT_NAME);

	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//zakonczenie generatora traka
	send_end_motion_to_ecps(lib::irp6ot_m::ROBOT_NAME);

	//zadanie chwytania kostki (koniec)
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_ap_3.trj", lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP

	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	// poprawinie ulozenia kostki w pozycji chytakiem do gory
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::RELATIVE, 0.01, lib::irp6ot_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
	//zaciskanie na kostce

        set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.058, lib::irp6ot_tfg::ROBOT_NAME);
	// oczekiwania na zakonczenie ruchu chwytaka
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//zadanie chwytania kostki (koniec)
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_ap_4.trj", lib::irp6ot_m::ROBOT_NAME);
	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP

	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

}

// odejscie
void rubik_cube_observer::departure_op()
{
	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "skon~czyl/em", 0, lib::festival::ROBOT_NAME);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_de_1.trj", lib::irp6ot_m::ROBOT_NAME);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_de_1.trj", lib::irp6p_m::ROBOT_NAME);

	//wait_for_task_termination(false, 1, lib::festival::ROBOT_NAME.c_str());

	//set_next_ecp_state(ecp_mp::task::ECP_GEN_FESTIVAL, 0, "jade~ pracowac~ do anglii", 0, lib::festival::ROBOT_NAME);

	// uruchomienie generatora empty_gen i oczekiwanie na zakonczenie obydwu generatorow ECP
        wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);
}

void rubik_cube_observer::main_task_algorithm(void)
{
	std::string cube_initial_state_string = config.value <std::string>("cube_initial_state");

	const char * cube_initial_state = cube_initial_state_string.c_str();

	//	enum common::CUBE_COLOR {UKNOWN, common::RED, common::YELLOW, common::GREEN, common::BLUE, common::ORANGE, common::WHITE};
	//	 cube_state::set_state(common::CUBE_COLOR up_is, common::CUBE_COLOR down_is, common::CUBE_COLOR front_is,
	//		common::CUBE_COLOR rear_is, common::CUBE_COLOR left_is, common::CUBE_COLOR right_is)

	initiate(common::read_cube_color(cube_initial_state[0]), common::read_cube_color(cube_initial_state[1]), common::read_cube_color(cube_initial_state[2]), common::read_cube_color(cube_initial_state[3]), common::read_cube_color(cube_initial_state[4]), common::read_cube_color(cube_initial_state[5]));

	// ---- tester ----
	approach_op(vis_servoing);

	look_at_wall(common::FRONT_WALL,false);wait_ms(1000);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_begin_end.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
	look_at_wall(common::UP_SIDE_WALL,false);wait_ms(1000);
	look_at_wall(common::RIGHT_SIDE_WALL,false);wait_ms(1000);
	look_at_wall(common::DOWN_SIDE_WALL,false);wait_ms(1000);
	look_at_wall(common::LEFT_SIDE_WALL,false);wait_ms(1000);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_begin_end.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_begin_end.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);

	face_turn_op(common::CL_0);
	face_change_op(common::CL_90);
	face_turn_op(common::CL_0);
	face_change_op(common::CL_180);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_begin_end.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_begin_end.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
	look_at_wall(common::FRONT_WALL,true);wait_ms(1000);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_begin_end.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);
	look_at_wall(common::UP_SIDE_WALL,true);wait_ms(1000);
	look_at_wall(common::RIGHT_SIDE_WALL,true);wait_ms(1000);
	look_at_wall(common::DOWN_SIDE_WALL,true);wait_ms(1000);
	look_at_wall(common::LEFT_SIDE_WALL,true);wait_ms(1000);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6ot_sm_watch_wall_begin_end.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, (int) lib::ABSOLUTE, "../../src/application/rubic_cube_observer/trj/irp6p_sm_watch_wall_begin_end.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false,lib::irp6p_m::ROBOT_NAME);

	calculate_color_combination();

	wait_ms(500);
	for(int i=0; i<6;i++)
	{
		sr_ecp_msg->message("   ");
		for(int j=0; j<3;j++)
		{
			String c11 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[0][j].color[0]);
			String c12 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[0][j].color[1]);
			String c13 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[0][j].color[2]);
			String c14 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[0][j].numberOfColor);

			String c21 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[1][j].color[0]);
			String c22 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[1][j].color[1]);
			String c23 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[1][j].color[2]);
			String c24 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[1][j].numberOfColor);

			String c31 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[2][j].color[0]);
			String c32 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[2][j].color[1]);
			String c33 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[2][j].color[2]);
			String c34 = boost::lexical_cast<std::string>( (int)wallsOfCube[i].field[2][j].numberOfColor);


			String s=c11+" "+c12+" "+c13+ " "+ c14 +" | "+c21+" "+c22+" "+c23+ " "+ c24 +" | "+c31+" "+c32+" "+c33 +" "+c34;
			sr_ecp_msg->message(s);wait_ms(500);
		}
	}//*/

	wait_ms(500);
	for(int i=0; i<6;i++)
	{
		sr_ecp_msg->message("   ");
		for(int j=0; j<3;j++)
		{
			String c1;
			switch((int)wallsOfCube[i].field[0][j].numberOfColor)
			{
			case 0: c1="Bialy       ";break;
			case 1: c1="Zolty       ";break;
			case 2: c1="Pomaranczowy";break;
			case 3: c1="Niebieski   ";break;
			case 4: c1="Czerwony    ";break;
			case 5: c1="Zielony     ";break;
			}
			String c2;
			switch((int)wallsOfCube[i].field[1][j].numberOfColor)
			{
			case 0: c2="Bialy       ";break;
			case 1: c2="Zolty       ";break;
			case 2: c2="Pomaranczowy";break;
			case 3: c2="Niebieski   ";break;
			case 4: c2="Czerwony    ";break;
			case 5: c2="Zielony     ";break;
			}
			String c3;
			switch((int)wallsOfCube[i].field[2][j].numberOfColor)
			{
			case 0: c3="Bialy       ";break;
			case 1: c3="Zolty       ";break;
			case 2: c3="Pomaranczowy";break;
			case 3: c3="Niebieski   ";break;
			case 4: c3="Czerwony    ";break;
			case 5: c3="Zielony     ";break;
			}


			String s=c1+" | "+c2+" | "+c3;
			sr_ecp_msg->message(s);wait_ms(500);
		}
	}

	/*String c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12;
	String s;
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[0][0].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[1][0].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[2][0].numberOfColor);
	s=c1+c2+c3;
	sr_ecp_msg->message(s);wait_ms(500);
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[0][1].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[1][1].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[2][1].numberOfColor);
	s=c1+c2+c3;
	sr_ecp_msg->message(s);wait_ms(500);
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[0][2].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[1][2].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[0].field[2][2].numberOfColor);
	s=c1+c2+c3;
	sr_ecp_msg->message(s);wait_ms(500);


	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[0][0].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[1][0].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[2][0].numberOfColor);
	c4 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[0][0].numberOfColor);
	c5 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[1][0].numberOfColor);
	c6 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[2][0].numberOfColor);
	c7 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[0][0].numberOfColor);
	c8 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[1][0].numberOfColor);
	c9 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[2][0].numberOfColor);
	c10 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[0][0].numberOfColor);
	c11 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[1][0].numberOfColor);
	c12 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[2][0].numberOfColor);
	s=c1+c2+c3+"|"+c4+c5+c6+"|"+c7+c8+c9+"|"+c10+c11+c12;
	sr_ecp_msg->message(s);wait_ms(500);
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[0][1].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[1][1].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[2][1].numberOfColor);
	c4 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[0][1].numberOfColor);
	c5 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[1][1].numberOfColor);
	c6 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[2][1].numberOfColor);
	c7 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[0][1].numberOfColor);
	c8 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[1][1].numberOfColor);
	c9 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[2][1].numberOfColor);
	c10 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[0][1].numberOfColor);
	c11 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[1][1].numberOfColor);
	c12 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[2][1].numberOfColor);
	s=c1+c2+c3+"|"+c4+c5+c6+"|"+c7+c8+c9+"|"+c10+c11+c12;
	sr_ecp_msg->message(s);wait_ms(500);
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[0][2].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[1][2].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[4].field[2][2].numberOfColor);
	c4 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[0][2].numberOfColor);
	c5 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[1][2].numberOfColor);
	c6 = boost::lexical_cast<std::string>( (int)wallsOfCube[3].field[2][2].numberOfColor);
	c7 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[0][2].numberOfColor);
	c8 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[1][2].numberOfColor);
	c9 = boost::lexical_cast<std::string>( (int)wallsOfCube[2].field[2][2].numberOfColor);
	c10 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[0][2].numberOfColor);
	c11 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[1][2].numberOfColor);
	c12 = boost::lexical_cast<std::string>( (int)wallsOfCube[1].field[2][2].numberOfColor);
	s=c1+c2+c3+"|"+c4+c5+c6+"|"+c7+c8+c9+"|"+c10+c11+c12;
	sr_ecp_msg->message(s);wait_ms(500);


	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[0][0].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[1][0].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[2][0].numberOfColor);
	s=c1+c2+c3;
	sr_ecp_msg->message(s);wait_ms(500);
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[0][1].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[1][1].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[2][1].numberOfColor);
	s=c1+c2+c3;
	sr_ecp_msg->message(s);wait_ms(500);
	c1 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[0][2].numberOfColor);
	c2 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[1][2].numberOfColor);
	c3 = boost::lexical_cast<std::string>( (int)wallsOfCube[5].field[2][2].numberOfColor);
	s=c1+c2+c3;
	sr_ecp_msg->message(s);wait_ms(500);*/

	departure_op();
	//Puszczanie kostki
    set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.065, lib::irp6ot_tfg::ROBOT_NAME);
    // oczekiwania na zakonczenie ruchu chwytaka
    wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void rubik_cube_observer::create_robots()
{

	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);

	//ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	//ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

task* return_created_mp_task(lib::configurator &_config)
{
	return new rubik_cube_observer(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp

