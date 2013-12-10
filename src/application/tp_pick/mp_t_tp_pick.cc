#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <iostream>

#include "mp_t_tp_pick.h"

// ecp generators to be commanded
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/constant_velocity/ecp_mp_g_constant_velocity.h"


#include "application/visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"

// mp_robots headers
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "application/irp6_tfg/ecp_mp_g_tfg.h"
#include "robot/irp6_tfg/dp_tfg.h"

#include "sensor/discode/discode_sensor.h"
#include "DiceReading.h"
#include "Algorythm.cc"

#define GRIP_MIN 0.064
#define GRIP_MAX 0.090

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;
using mrrocpp::ecp_mp::sensor::discode::ds_exception;

namespace mrrocpp {
namespace mp {
namespace task {


// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void tp_pick::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
}

void tp_pick::configure_discode()
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


	}catch(ds_exception e){
		sr_ecp_msg->message("configure_discode error");
		sr_ecp_msg->message(e.what());
	}
}

tp_pick::tp_pick(lib::configurator &config) :
		task(config), config_section_name("[visualservo_tester]")
{
	run_vs = config.value <bool>("run_vs", config_section_name);
	vs_settle_time = config.value <int>("vs_settle_time", config_section_name);
	robot_name = config.value <std::string>("robot_name", config_section_name);

	configure_discode();
	dots = 0;
	angle = 0.0;
	for(int i=0;i<5;i++){
		dices.push_back(0);
	}

}

Types::Mrrocpp_Proxy::DiceReading tp_pick::read_from_discode()
{
	sr_ecp_msg->message("read_from_discode");

	Types::Mrrocpp_Proxy::DiceReading reading;

	try {
		discode->get_reading();
		wait_ms(500);
		reading = discode->retreive_reading <Types::Mrrocpp_Proxy::DiceReading>();
		sr_ecp_msg->message("reading recived");

	} catch (ds_exception e){
		sr_ecp_msg->message("read_from_discode error");
		sr_ecp_msg->message(e.what());
	}

	return reading;
}

double tp_pick::recive_angle(void)
{
	double angle = 0.0;
	while (1)
	{
		angle = read_from_discode().angle;
		if (angle < 0)
			return angle + 3.1415/2.0;
		else if (angle > 0)
			return angle - 3.1415/2.0;
		else {
			wait_ms(500);
		}
	}
}

int tp_pick::recive_dots(void)
{
	return read_from_discode().dots;
}

void tp_pick::recive_data(int i)
{
	angle = 0.0;
	int tr = 10;
	Types::Mrrocpp_Proxy::DiceReading reading;
	while (1)
	{
		reading = read_from_discode();
		angle = reading.angle;
		if (angle < 0)
		{
			angle = angle + 3.1415/2.0;
			break;
		} else if (angle > 0)
		{
			angle = angle - 3.1415/2.0;
			break;
		} else {
			wait_ms(500);
			tr--;
			if (tr<0) break;
		}
	}
	dots = reading.dots;
	dices[i-1] = dots;
}

void tp_pick::start_position(void)
{
	sr_ecp_msg->message("start_position");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/start_position.trj", robot_name);
	wait_for_task_termination(false, robot_name);
}

void tp_pick::servovision(void)
{
	sr_ecp_msg->message("Servovision started");

	set_next_ecp_state(mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, 0, "", robot_name);
	wait_for_task_termination(false, robot_name);

	sr_ecp_msg->message("Object located");

}

void tp_pick::double_servovision(void)
{
	down(0.01);
	servovision();
	down(0.07);
	servovision();
	wait_ms(1000);
}

void tp_pick::servo_correction(void)
{
	sr_ecp_msg->message("servo_correction");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/servo_correction.trj",robot_name);
	wait_for_task_termination(false, robot_name);
}

void tp_pick::calibration(void)
{
	start_position();

	set_next_ecp_state("DICE", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);
	save_position(0);

	double_servovision();
	servo_correction();
	save_position(1);
	gripper(GRIP_MAX);
	down(0.055);
	gripper(GRIP_MIN);
	up();

	set_next_ecp_state("CUP", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	save_position(6);
	gripper(GRIP_MAX);
	wait_ms(500);

	for(int i=1; i<5; i++)
	{
		load_position(0);
		double_servovision();
		servo_correction();
		save_position(i+1);
		gripper(GRIP_MAX);
		down(0.055);
		gripper(GRIP_MIN);
		up();
		wait_ms(500);
		load_position(6);
		gripper(GRIP_MAX);
		wait_ms(500);

	}
}

void tp_pick::save_position(int i)
{
	switch (i)
	{
		case 0 :
			set_next_ecp_state("SAVE 0", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 1 :
			set_next_ecp_state("SAVE 1", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 2 :
			set_next_ecp_state("SAVE 2", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 3 :
			set_next_ecp_state("SAVE 3", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 4 :
			set_next_ecp_state("SAVE 4", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 5 :
			set_next_ecp_state("SAVE 5", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 6 :
			set_next_ecp_state("SAVE 6", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;
	}
}

void tp_pick::load_position(int i)
{
	switch (i)
	{
		case 0 :
			set_next_ecp_state("LOAD 0", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 1 :
			set_next_ecp_state("LOAD 1", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 2 :
			set_next_ecp_state("LOAD 2", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 3 :
			set_next_ecp_state("LOAD 3", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 4 :
			set_next_ecp_state("LOAD 4", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 5 :
			set_next_ecp_state("LOAD 5", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;

		case 6 :
			set_next_ecp_state("LOAD 6", 0, "",robot_name);
			wait_for_task_termination(false, robot_name);
		break;
	}
}

void tp_pick::gripper(double width)
{
	sr_ecp_msg->message("gripper");
	
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, width, lib::irp6ot_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);
}

void tp_pick::turn_gripper(double angle)
{
	sr_ecp_msg->message("turn_gripper");

	std::ostringstream strs;
	strs << angle;
	std::string str = strs.str();

	std::string order = "TURN " + str;

	set_next_ecp_state(order, 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

}
 
void tp_pick::down(double distance)
{
	sr_ecp_msg->message("down");

	std::ostringstream strs;
	strs << distance;
	std::string str = strs.str();

	std::string order = "DOWN " + str;

	set_next_ecp_state(order, 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/down.trj", lib::irp6ot_m::ROBOT_NAME);
	//wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
}

void tp_pick::up(void)
{
	sr_ecp_msg->message("up");
	
	set_next_ecp_state("UP", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);
}

void tp_pick::throw_dices(void)
{
	sr_ecp_msg->message("throw_dices");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_1_to_cup.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	gripper(GRIP_MAX);

	set_next_ecp_state("THROW 1", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	gripper(0.071);

	set_next_ecp_state("THROW 2", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	turn_gripper(2.0);

	turn_gripper(0.0);

	set_next_ecp_state("THROW 1", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	gripper(GRIP_MAX);

	set_next_ecp_state("THROW 4", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	start_position();

	/*
	//nad kubek
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_1_to_cup.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//w dół do krawędzi
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_2_go_down.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//otwórz szczęki
	gripper(GRIP_MAX);
	//czekaj na reakcje
	wait_ms(3000);
	//w dół do kubka
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_3_go_down.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//zaciśnij szczęki
	gripper(0.071);
	//w górę
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_2_go_down.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//do rzutu
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_4_ready.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//przechyl
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_5_throw.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//do odłożenia
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_2_go_down.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//odłóż
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_3_go_down.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	//rozluźnij szczęki	
	gripper(0.085);
	//nad kubek
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/throw_dices_1_to_cup.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	*/

}

void tp_pick::pick(void)
{
	gripper(GRIP_MAX);
	down(0.086);
	gripper(GRIP_MIN);
	up();
}

void tp_pick::pick_from_slot(int slot)
{
	load_position(slot);
	gripper(GRIP_MAX);
	down(0.055);
	gripper(GRIP_MIN);
	up();
	wait_ms(500);
	load_position(6);
	gripper(GRIP_MAX);

	/*
	gripper(GRIP_MAX);
	under_slot(slot);
	wait_ms(1500);
	down(0.012);
	gripper(GRIP_MIN);
	wait_ms(500);
	up();
	put_to_cup();
	*/
}


void tp_pick::under_slot(int slot)
{
	sr_ecp_msg->message("under_slot");
	
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/slot0.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	switch (slot)
	{
		case 0 :
		break;

		case 1 :
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/slot1.trj", lib::irp6ot_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
		break;

		case 2 :
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/slot2.trj", lib::irp6ot_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
		break;

		case 3 :
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/slot3.trj", lib::irp6ot_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
		break;

		case 4 :
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/slot4.trj", lib::irp6ot_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
		break;

		case 5 :
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/slot5.trj", lib::irp6ot_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
		break;

		
	}
}

void tp_pick::put_away(int slot)
{
	sr_ecp_msg->message("put_away");

	load_position(slot);
	down(0.045);
	gripper(GRIP_MAX);
	up();
	start_position();
}

void tp_pick::put_to_cup(void)
{
	sr_ecp_msg->message("put_to_cup");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tp_pick/trj/under_cup.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);
	gripper(0.09);
	
}

void tp_pick::pick_all(void)
{
	sr_ecp_msg->message("pick_all");

	for(int i=1;i<6;i++){
		gripper(GRIP_MAX);
		under_slot(i);
		wait_ms(1500);
		down(0.012);
		gripper(GRIP_MIN);
		wait_ms(500);		
		up();
		put_to_cup();	
	}	

}

bool tp_pick::throw_again(int i)
{
	if (decision.dice[i-1] == ROLL)
		return true;
	else
		return false;

}

bool tp_pick::make_decision(void)
{

	Algorythm a;
	a.load(dices);
	decision = a.calculate();
	return (throw_again(1) || throw_again(2) || throw_again(3) || throw_again(4) || throw_again(5));

}

void tp_pick::main_task_algorithm(void)
{
	sr_ecp_msg->message("(!) TP_PICK (MP) START v 1.28");

	//throw test
	start_position();

	set_next_ecp_state("DICE", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);
	save_position(0);

	double_servovision();
	servo_correction();
	save_position(1);
	gripper(GRIP_MAX);
	down(0.055);
	gripper(GRIP_MIN);
	up();

	set_next_ecp_state("CUP", 0, "",robot_name);
	wait_for_task_termination(false, robot_name);

	save_position(6);
	gripper(GRIP_MAX);
	wait_ms(500);

	start_position();

	throw_dices();

	/*
	calibration();

	throw_dices();

	for (int i=1; i<6; i++)
	{
		start_position();
		double_servovision();
		recive_data(i);
		servo_correction();
		turn_gripper(angle);
		pick();
		start_position();
		put_away(i);
	}

	if (make_decision()){

		for (int i=1; i<6; i++)
		{
			if (throw_again(i)){
				pick_from_slot(i);
			}
		}

		throw_dices();

		for (int i=1; i<6; i++)
		{
			if (throw_again(i)){
				start_position();
				double_servovision();
				recive_data(i);
				servo_correction();
				turn_gripper(angle);
				pick();
				start_position();
				put_away(i);
			}

		}

	}
*/

	sr_ecp_msg->message("(!) TP_PICK (MP) END");
}

task* return_created_mp_task(lib::configurator &_config)
{
	return new tp_pick(_config);
}



} // namespace task
} // namespace mp
} // namespace mrrocpp
