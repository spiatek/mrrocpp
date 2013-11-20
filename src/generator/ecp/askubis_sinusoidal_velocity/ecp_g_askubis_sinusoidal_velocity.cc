/*!
 * @file
 * @brief File contains  tff nose run generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_askubis_sinusoidal_velocity.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

askubis_sinusoidal_velocity::askubis_sinusoidal_velocity(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), pulse_check_activated(false), force_meassure(false), step_no(step)
{
	generator_name = ecp_mp::generator::ECP_GEN_ASKUBIS_SINUSOIDAL_VELOCITY;
	step=0;
	nmc=5;//nmc = 5 => 10ms loop
	axes_num=6;

	// domyslnie wszytkie osie podatne a pulse_check nieaktywne
//	configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT);
	configure_pulse_check(false);
//	configure_velocity(0.0,0.0,0.0,0.0,0.0,0.0);
//	configure_force(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	//askubis start change
//	configure_reciprocal_damping(lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING);
	//configure_inertia(lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA);


//	configure_reciprocal_damping(lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, 0.0025, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING);
//	configure_inertia(lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA);
//	configure_reciprocal_damping(lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, 0.005, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING);
//	configure_inertia(lib::FORCE_INERTIA, lib::FORCE_INERTIA, 0.25, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA);

	std::cout<<"KOPYTKO 123 test"<<std::endl;

}

void askubis_sinusoidal_velocity::conditional_execution()
{

	switch ((ecp_mp::generator::askubis_sinusoidal_velocity::communication_type) ecp_t.mp_command.ecp_next_state.variant)
	{

		case ecp_mp::generator::askubis_sinusoidal_velocity::behaviour_specification: {
			//ecp_mp::generator::askubis_sinusoidal_velocity::behaviour_specification_data_type beh;
			//ecp_t.mp_command.ecp_next_state.sg_buf.get(beh);
//			configure_behaviour(beh.behaviour[0], beh.behaviour[1], beh.behaviour[2], beh.behaviour[3], beh.behaviour[4], beh.behaviour[5]);
			break;
		}
		case ecp_mp::generator::askubis_sinusoidal_velocity::no_data:
			break;
		default:
			break;
	}

	Move();
}

void askubis_sinusoidal_velocity::set_force_meassure(bool fm)
{
	force_meassure = fm;
}

void askubis_sinusoidal_velocity::configure_pulse_check(bool pulse_check_activated_l)
{
	pulse_check_activated = pulse_check_activated_l;
}
//
//void askubis_sinusoidal_velocity::configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z, lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az)
//{
//	generator_edp_data.next_behaviour[0] = x;
//	generator_edp_data.next_behaviour[1] = y;
//	generator_edp_data.next_behaviour[2] = z;
//	generator_edp_data.next_behaviour[3] = ax;
//	generator_edp_data.next_behaviour[4] = ay;
//	generator_edp_data.next_behaviour[5] = az;
//}
//
//void askubis_sinusoidal_velocity::configure_velocity(double x, double y, double z, double ax, double ay, double az)
//{
//	generator_edp_data.next_velocity[0] = x;
//	generator_edp_data.next_velocity[1] = y;
//	generator_edp_data.next_velocity[2] = z;
//	generator_edp_data.next_velocity[3] = ax;
//	generator_edp_data.next_velocity[4] = ay;
//	generator_edp_data.next_velocity[5] = az;
//}
//
//void askubis_sinusoidal_velocity::configure_force(double x, double y, double z, double ax, double ay, double az)
//{
//	generator_edp_data.next_force_xyz_torque_xyz[0] = x;
//	generator_edp_data.next_force_xyz_torque_xyz[1] = y;
//	generator_edp_data.next_force_xyz_torque_xyz[2] = z;
//	generator_edp_data.next_force_xyz_torque_xyz[3] = ax;
//	generator_edp_data.next_force_xyz_torque_xyz[4] = ay;
//	generator_edp_data.next_force_xyz_torque_xyz[5] = az;
//}
//
//void askubis_sinusoidal_velocity::configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az)
//{
//	generator_edp_data.next_reciprocal_damping[0] = x;
//	generator_edp_data.next_reciprocal_damping[1] = y;
//	generator_edp_data.next_reciprocal_damping[2] = z;
//	generator_edp_data.next_reciprocal_damping[3] = ax;
//	generator_edp_data.next_reciprocal_damping[4] = ay;
//	generator_edp_data.next_reciprocal_damping[5] = az;
//}
//
//void askubis_sinusoidal_velocity::configure_inertia(double x, double y, double z, double ax, double ay, double az)
//{
//	generator_edp_data.next_inertia[0] = x;
//	generator_edp_data.next_inertia[1] = y;
//	generator_edp_data.next_inertia[2] = z;
//	generator_edp_data.next_inertia[3] = ax;
//	generator_edp_data.next_inertia[4] = ay;
//	generator_edp_data.next_inertia[5] = az;
//}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool askubis_sinusoidal_velocity::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	//	std::cout << "askubis_sinusoidal_velocity" << node_counter << std::endl;
//
//	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
//	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;
//
//	the_robot->ecp_command.instruction_type = lib::GET;
//	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
//	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
//	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
//	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
//	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
//	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
////	the_robot->ecp_command.get_arm_type = lib::FRAME;
//	the_robot->ecp_command.motion_type = lib::RELATIVE;
//	the_robot->ecp_command.interpolation_type = lib::TCIM;
//	the_robot->ecp_command.motion_steps = step_no;
//	the_robot->ecp_command.value_in_step_no = step_no - 2;
//
//	for (int i = 0; i < 6; i++) {
//		the_robot->ecp_command.arm.pf_def.behaviour[i] = generator_edp_data.next_behaviour[i];
//		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = generator_edp_data.next_velocity[i];
//		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = generator_edp_data.next_force_xyz_torque_xyz[i];
//		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = generator_edp_data.next_reciprocal_damping[i];
//		the_robot->ecp_command.arm.pf_def.inertia[i] = generator_edp_data.next_inertia[i];
//	}

	lib::Homog_matrix actual_position_matrix;
	actual_position_matrix = the_robot->reply_package.arm.pf_def.arm_frame;
	lib::Xyz_Euler_Zyz_vector euler_vector;
	actual_position_matrix.get_xyz_euler_zyz(euler_vector);
	first_pos = euler_vector;
	current_pos = euler_vector[0];

	timeval start;
	gettimeofday(&start, NULL);
	last_timestamp = start;

	step = 0;
			the_robot->ecp_command.set_type = ARM_DEFINITION;
			the_robot->ecp_command.motion_steps = nmc;
			the_robot->ecp_command.value_in_step_no = nmc - 2;
			the_robot->communicate_with_edp = false;

			the_robot->ecp_command.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.set_arm_type = lib::FRAME;

			the_robot->ecp_command.interpolation_type = lib::TCIM;
			for (int i = 0; i < axes_num; i++) {
				the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
			}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool askubis_sinusoidal_velocity::next_step()
{

	//	std::cout << "askubis_sinusoidal_velocity" << node_counter << std::endl;
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//std::cout << "next_step" << std::endl;

	//the_robot->ecp_command.arm.pf_def.

//	if (pulse_check_activated && check_and_null_trigger()) { // Koniec odcinka
//		//	ecp_t.set_ecp_reply (lib::TASK_TERMINATED);
//
//		return false;
//	}
//
//	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
//	the_robot->ecp_command.instruction_type = lib::SET_GET;
//
//	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
//
//	// wyrzucanie odczytu sil
//
	lib::Homog_matrix actual_position_matrix;
	actual_position_matrix = the_robot->reply_package.arm.pf_def.arm_frame;
	lib::Xyz_Euler_Zyz_vector euler_vector;
	actual_position_matrix.get_xyz_euler_zyz(euler_vector);

	step++;
	double time = step*0.01;
	double A= 0.001;
	double B = 0.5;
	double gain = A*std::sin(2*3.14*B*time*time);
	//double gain = A*(0.5 - (1/(3.14*(2*sqrt(B)*time)))*std::cos(1/2 * 3.14 *(2*sqrt(B)*time)*(2*sqrt(B)*time)))/(2*sqrt(B));
	//double gain = A/(2*sqrt(B))*(0.5 - ((1/(2*3.14*sqrt(B)*time)) * std::cos(2*3.14*B*time*time)));
	current_pos+=gain;
	mrrocpp::lib::Xyz_Euler_Zyz_vector tmp = first_pos;
	tmp[0]=current_pos;

	timeval next;
	gettimeofday(&next, NULL);
	int usecdiff = next.tv_usec-last_timestamp.tv_usec;

	std::cout<<"step no "<<step<<" gain: "<<gain<< " TIME: "<<usecdiff<<" ZADANE: "<<current_pos<</*" pozycja osiagnieta "<<euler_vector[0]<<*/std::endl;//wypisac time
	//std::cout<<tmp[0]<<" "<<tmp[1]<<" "<<tmp[2]<<" "<<tmp[3]<<" "<<tmp[4]<<" "<<tmp[5]<<" "<<std::endl;
	last_timestamp = next;

	the_robot->communicate_with_edp = true; //turn on the communication with EDP
	the_robot->ecp_command.instruction_type = lib::SET;
	the_robot->ecp_command.arm.pf_def.arm_frame.set_from_xyz_euler_zyz(tmp);


	if (force_meassure) {
			lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
			current_frame_wo_offset.remove_translation();

			lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);


			std::cout << "force measured: " << force_torque << std::endl;


		}
	return true;

}

// metoda przeciazona bo nie chcemy rzucac wyjatku wyjscia poza zakres ruchu - UWAGA napisany szkielet skorygowac cialo funkcji

void askubis_sinusoidal_velocity::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	// komunikacja wlasciwa
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();
		BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));

	}
	the_robot->query();

	if (the_robot->reply_package.reply_type == lib::ERROR) {
		switch (the_robot->reply_package.error_no.error0)
		{
			case BEYOND_UPPER_D0_LIMIT:
			case BEYOND_UPPER_THETA1_LIMIT:
			case BEYOND_UPPER_THETA2_LIMIT:
			case BEYOND_UPPER_THETA3_LIMIT:
			case BEYOND_UPPER_THETA4_LIMIT:
			case BEYOND_UPPER_THETA5_LIMIT:
			case BEYOND_UPPER_THETA6_LIMIT:
			case BEYOND_UPPER_THETA7_LIMIT:
			case BEYOND_LOWER_D0_LIMIT:
			case BEYOND_LOWER_THETA1_LIMIT:
			case BEYOND_LOWER_THETA2_LIMIT:
			case BEYOND_LOWER_THETA3_LIMIT:
			case BEYOND_LOWER_THETA4_LIMIT:
			case BEYOND_LOWER_THETA5_LIMIT:
			case BEYOND_LOWER_THETA6_LIMIT:
			case BEYOND_LOWER_THETA7_LIMIT:
				break;
			default:
				BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));
				break;

		} /* end: switch */
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
