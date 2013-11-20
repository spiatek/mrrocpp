// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <cfloat>
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_common_012.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ui {
namespace common_012 {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		common::EcpRobot(_ui_robot)
{

}
// ---------------------------------------------------------------

EcpRobot::~EcpRobot()
{
	delete ecp;
}

// ---------------------------------------------------------------
void EcpRobot::init()
{
	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.robot_model.kinematic_model.kinematic_model_no = 0;
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
//	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = 0;
	ecp->ecp_command.value_in_step_no = 0;

	ecp->synchronised = false;

}

// ---------------------------------------------------------------
void EcpRobot::move_currents(const short des_cur[])
{
//	printf("move_currents: \n\n\n");
	int nr_of_steps = 10;

	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;

	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
//	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;
	for (int j = 0; j < ecp->number_of_servos; j++) {
		ecp->ecp_command.arm.pf_def.desired_torque_or_current[j] = double(des_cur[j]);
		//	printf("ggg: %f\n\n\n", ecp->ecp_command.arm.pf_def.desired_torque_or_current[j]);
	}

	// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp_command.motion_steps, ecp_command.value_in_step_no, ecp_command.arm.pf_def.arm_coordinates[1]);

	execute_motion();

}

// ---------------------------------------------------------------
void EcpRobot::move_motors(const double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze

	/*
	 if (is_synchronised())
	 printf("zsynchronizowany move motors\n");
	 else
	 printf("niezsynchronizowany move motors\n");
	 */
	if (ecp->is_synchronised()) { // Robot zsynchronizowany
		// Odczyt aktualnego polozenia
		//   	printf("is synchronised przed read motors\n");
		read_motors(current_position);

		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j] - current_position[j]);
			nr_tmp = (int) ceil(temp / MOTOR_STEP[j]);
			nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
		}

		//  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
		// Parametry zlecenia ruchu i odczytu polozenia
		ecp->ecp_command.instruction_type = lib::SET_GET;
		ecp->ecp_command.motion_type = lib::ABSOLUTE;
		ecp->ecp_command.interpolation_type = lib::MIM;
	} else {
		// printf("!is_synchronised: %f \n",MOTOR_STEP);
		// Robot niezsynchroniozowany
		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j]);
			nr_tmp = (int) ceil(temp / MOTOR_STEP[j]);
			nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
		}

		ecp->ecp_command.instruction_type = lib::SET;
		ecp->ecp_command.motion_type = lib::RELATIVE;
		ecp->ecp_command.interpolation_type = lib::MIM;
	}
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
//	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;
	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.arm.pf_def.arm_coordinates[j] = final_position[j];

	// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp_command.motion_steps, ecp_command.value_in_step_no, ecp_command.arm.pf_def.arm_coordinates[1]);

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
				{
			current_position[j] = ecp->reply_package.arm.pf_def.motor_coordinates[j];
		}
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::move_joints(const double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze

	// Odczyt aktualnego polozenia
	read_joints(current_position);

	for (int j = 0; j < ecp->number_of_servos; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		nr_tmp = (int) ceil(temp / JOINT_STEP[j]);
		nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
	}

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
//	ecp->ecp_command.get_arm_type = lib::JOINT;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::JOINT;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	// cprintf("NOS=%u\n",ecp_command.motion_steps);

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.arm.pf_def.arm_coordinates[j] = final_position[j];

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.joint_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::set_desired_position(const double d_position[])
{
	// Przepisanie polozen zadanych do tablicy desired_position[]
	for (int j = 0; j < ecp->number_of_servos; j++)
		desired_position[j] = d_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::get_current_position(double c_position[])
{
	// Pobranie aktualnych polozen
	for (int j = 0; j < ecp->number_of_servos; j++)
		c_position[j] = current_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// zlecenie odczytu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow

void EcpRobot::get_kinematic(uint8_t* kinematic_model_no)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.get_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL
	execute_motion();

	*kinematic_model_no = ecp->reply_package.robot_model.kinematic_model.kinematic_model_no;
}

void EcpRobot::get_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie odczytu numerow algorytmow i zestawow parametrow
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.get_robot_model_type = lib::SERVO_ALGORITHM; //
	execute_motion();

	// Przepisanie aktualnych numerow algorytmow i zestawow parametrow
	memcpy(algorithm_no, ecp->reply_package.robot_model.servo_algorithm.servo_algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(parameters_no, ecp->reply_package.robot_model.servo_algorithm.servo_parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
}

// ---------------------------------------------------------------
void EcpRobot::set_kinematic(uint8_t kinematic_model_no)
{
	// zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
	// algorytmow serwo i numerow zestawow parametrow algorytmow

	// Zlecenie zapisu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL
	ecp->ecp_command.get_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL

	ecp->ecp_command.robot_model.kinematic_model.kinematic_model_no = kinematic_model_no;

	execute_motion();
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::set_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{
	// Zlecenie zapisu numerow algorytmow i zestawow parametrow
	// Przepisanie zadanych numerow algorytmow i zestawow parametrow
	memcpy(ecp->ecp_command.robot_model.servo_algorithm.servo_algorithm_no, algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(ecp->ecp_command.robot_model.servo_algorithm.servo_parameters_no, parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::SERVO_ALGORITHM; //
	ecp->ecp_command.get_robot_model_type = lib::SERVO_ALGORITHM; //
	execute_motion();
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::read_currents(short current_current[])
{
	// Zlecenie odczytu polozenia

	// printf("poczatek read motors\n");
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction_type = lib::GET;
//	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();
	// printf("dalej za query read motors\n");
	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		// { // printf("current position: %f\n",ecp->reply_package.arm.pf_def.arm_coordinates[i]);
		current_current[i] = ecp->reply_package.arm.measured_current.average_value[i];
	// 			    }
	// printf("koniec read motors\n");
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::read_motors(double current_position[])
{
	// Zlecenie odczytu polozenia

	// printf("poczatek read motors\n");
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction_type = lib::GET;
//	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();
	// printf("dalej za query read motors\n");
	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		// { // printf("current position: %f\n",ecp->reply_package.arm.pf_def.arm_coordinates[i]);
		current_position[i] = ecp->reply_package.arm.pf_def.motor_coordinates[i];
	// 			    }
	// printf("koniec read motors\n");
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::read_joints(double current_position[])
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ARM_DEFINITION;
	//ecp->ecp_command.get_arm_type = lib::JOINT;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();

	for (int i = 0; i < ecp->number_of_servos; ++i) // Przepisanie aktualnych polozen
		current_position[i] = ecp->reply_package.arm.pf_def.joint_coordinates[i];
}
// ---------------------------------------------------------------

// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::set_tool_xyz_angle_axis(const lib::Xyz_Angle_Axis_vector &tool_vector)
{
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	ecp->ecp_command.robot_model.tool_frame_def.tool_frame.set_from_xyz_angle_axis(tool_vector);

	execute_motion();
}
// ---------------------------------------------------------------

// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::set_tool_xyz_euler_zyz(const lib::Xyz_Euler_Zyz_vector &tool_vector)
{
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	ecp->ecp_command.robot_model.tool_frame_def.tool_frame.set_from_xyz_euler_zyz(tool_vector);

	execute_motion();
}
// ---------------------------------------------------------------

// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::read_tool_xyz_angle_axis(lib::Xyz_Angle_Axis_vector & tool_vector)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	execute_motion();

	ecp->reply_package.robot_model.tool_frame_def.tool_frame.get_xyz_angle_axis(tool_vector);
}
// ---------------------------------------------------------------

// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::read_tool_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector &tool_vector)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	execute_motion();

	ecp->reply_package.robot_model.tool_frame_def.tool_frame.get_xyz_euler_zyz(tool_vector);
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::move_xyz_euler_zyz(const double final_position[7])
{
	// Zlecenie wykonania makrokroku ruchu zadanego we wspolrzednych
	// zewnetrznych: xyz i katy Euler'a Z-Y-Z

	lib::Homog_matrix current_htm;
	lib::Homog_matrix desired_htm;
	desired_htm.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(final_position));

	move_htm_absolute(desired_htm, current_htm);

	lib::Xyz_Euler_Zyz_vector tmp_vector;
	current_htm.get_xyz_euler_zyz(tmp_vector);
	tmp_vector.to_table(current_position);

}
// ---------------------------------------------------------------

void EcpRobot::move_xyz_angle_axis(const double final_position[7])
{

	lib::Homog_matrix current_htm;
	lib::Homog_matrix desired_htm;
	desired_htm.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(final_position));

	move_htm_absolute(desired_htm, current_htm);

	lib::Xyz_Angle_Axis_vector tmp_vector;
	ecp->reply_package.arm.pf_def.arm_frame.get_xyz_angle_axis(tmp_vector);
	tmp_vector.to_table(current_position);

}

void EcpRobot::move_xyz_angle_axis_relative(const double position_increment[7])
{

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position_increment));

	move_htm_relative(tmp);

}

// ---------------------------------------------------------------
void EcpRobot::read_xyz_euler_zyz(double current_position[])
{

	lib::Homog_matrix tmp;

	read_htm(tmp);

	lib::Xyz_Euler_Zyz_vector tmp_vector;
	tmp.get_xyz_euler_zyz(tmp_vector);
	tmp_vector.to_table(current_position);

}
// ---------------------------------------------------------------

void EcpRobot::move_htm_absolute(lib::Homog_matrix & desired_htm, lib::Homog_matrix & current_htm)
{
	int nr_of_steps = 0; // Liczba krokow

	read_htm(current_htm);

	lib::Homog_matrix increment_htm;
	increment_htm = (!current_htm) * desired_htm;

	nr_of_steps = count_nr_of_steps(increment_htm);

	// cprintf("eNOS=%u\n",ecp->ecp_command.motion_steps);
	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	ecp->ecp_command.instruction_type = lib::SET_GET;
//	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::FRAME;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	ecp->ecp_command.arm.pf_def.arm_frame = desired_htm;

	execute_motion();

	current_htm = ecp->reply_package.arm.pf_def.arm_frame;

}

void EcpRobot::move_htm_relative(lib::Homog_matrix & desired_htm)
{
	int nr_of_steps = 0; // Liczba krokow

	nr_of_steps = count_nr_of_steps(desired_htm);

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction_type = lib::SET_GET;
//	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::FRAME;
	ecp->ecp_command.motion_type = lib::RELATIVE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	ecp->ecp_command.arm.pf_def.arm_frame = desired_htm;

	execute_motion();

}

int EcpRobot::count_nr_of_steps(lib::Homog_matrix & increment_htm)
{
	int nr_of_steps = 0, nr_position = 0, nr_angle; // Liczba krokow

	lib::Xyz_Angle_Axis_Gamma_vector increment_vector;
	increment_htm.get_xyz_angle_axis_gamma(increment_vector);

//	std::cout << "increment_vector :" << increment_vector << "\n\n";

//	double increment_table[6]; // polozenie aktualne
//	increment_vector.(increment_table);

	double position_increment = sqrt(increment_vector[0] * increment_vector[0]
			+ increment_vector[1] * increment_vector[1] + increment_vector[2] * increment_vector[2]);
	double angle_increment = fabs(increment_vector[6]);

	nr_position = (int) ceil(position_increment / END_EFFECTOR_LINEAR_STEP);
	nr_angle = (int) ceil(angle_increment / END_EFFECTOR_ANGULAR_STEP);
	nr_of_steps = (nr_position > nr_angle) ? nr_position : nr_angle;

	return nr_of_steps;
}

// ---------------------------------------------------------------
void EcpRobot::read_htm(lib::Homog_matrix & htm)
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction_type = lib::GET;
//	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();

	htm = ecp->reply_package.arm.pf_def.arm_frame;

}
// ---------------------------------------------------------------

void EcpRobot::read_xyz_angle_axis(double current_position[])
{
	lib::Homog_matrix tmp;
	read_htm(tmp);

	lib::Xyz_Angle_Axis_vector tmp_vector;
	tmp.get_xyz_angle_axis(tmp_vector);
	tmp_vector.to_table(current_position);

}

}
} //namespace ui
} //namespace mrrocpp

