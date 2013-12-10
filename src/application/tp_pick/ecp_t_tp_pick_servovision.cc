/*
 * ecp_t_tp_pick_servovision.cc
 *
 *      Author: tpokorsk
 */

#include <stdexcept>

#include "ecp_t_tp_pick_servovision.h"

#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"
//#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"

//#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
//#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
//#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_multiple_position.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

#include "../visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"

using namespace std;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_t_tp_pick_servovision::ecp_t_tp_pick_servovision(mrrocpp::lib::configurator& config) :
	common::task::task(config)
{
	try{

		std::string robot_name = config.value<std::string>("robot_name", "[visualservo_tester]");
		if(robot_name == lib::irp6p_m::ROBOT_NAME){
			ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
		} else if(robot_name == lib::irp6ot_m::ROBOT_NAME){
			ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
		} else {
			throw std::runtime_error("ecp_t_tp_pick_servovision option robot_name in config file has unknown value: " + robot_name);
		}

		// utworzenie generatorow
		sg = new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS ,6);
		gp = new common::generator::get_position(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);

		// utworzenie generatorow do uruchamiania dispatcherem
		register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_JOINT, ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, true));
		register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, true));

		char config_section_name[] = { "[object_follower_pb]" };

		log_enabled = true;
		log_dbg_enabled = true;

		boost::shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));

		if(config.exists_and_true("use_pid_regulator", config_section_name)){
			sr_ecp_msg->message("Using PID regulator");
			reg = boost::shared_ptr <visual_servo_regulator> (new regulator_pid(config, config_section_name));
		} else {
			sr_ecp_msg->message("Using P regulator");
			reg = boost::shared_ptr <visual_servo_regulator> (new regulator_p(config, config_section_name));
		}

		sr_ecp_msg->message("Creating DisCODe sensor");
		boost::shared_ptr <discode_sensor> ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));
		vs = boost::shared_ptr <visual_servo> (new pb_eih_visual_servo(reg, ds, config_section_name, config));

		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 3\n");
		sm = boost::shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));
		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 4\n");

		object_reached_term_cond =
					(boost::shared_ptr <termination_condition>) new object_reached_termination_condition(config, config_section_name);

		sm->add_position_constraint(cube);
		log_dbg("ecp_t_objectfollower_pb: configuring visual_servo_manager\n");
		sm->configure();
	}catch(std::exception& ex){
		sr_ecp_msg->message(lib::FATAL_ERROR, string("ERROR in ecp_t_tp_pick_servovision: ") + ex.what());
		throw ex;
	}
	log_dbg("ecp_t_objectfollower_pb: initialization completed.\n");
}

//void ecp_t_tp_pick_servovision::main_task_algorithm(void)
//{

//}

void ecp_t_tp_pick_servovision::mp_2_ecp_next_state_string_handler(void)
{

		//get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {

			sr_ecp_msg->message("Starting visual servo...");
			if(sm->log_client.get() != NULL){
				sm->log_client->set_filename_prefix("pb-eih_vs_manager");
				sm->log_client->set_connect();
			}
			if(vs->log_client.get() != NULL){
				vs->log_client->set_filename_prefix("pb-eih_vs");
				vs->log_client->set_connect();
			}
			sm->add_termination_condition(object_reached_term_cond);
			sm->Move();
			sr_ecp_msg->message("Visual servo ended.");

		} else if(mp_2_ecp_next_state_string.substr(0,4) == "TURN") {

			sr_ecp_msg->message("Turn...");

			std::string s_angle = mp_2_ecp_next_state_string.substr(5,10);
			double d_angle = atof(s_angle.c_str());
			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.0;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = 0.0;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = d_angle;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

		} else if(mp_2_ecp_next_state_string.substr(0,4) == "DOWN") {

			sr_ecp_msg->message("DOWN...");

			std::string s_distance = mp_2_ecp_next_state_string.substr(5,10);
			double d_distance = atof(s_distance.c_str());
			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.0;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = d_distance;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

		} else if(mp_2_ecp_next_state_string.substr(0,2) == "UP") {

			sr_ecp_msg->message("UP...");

			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.0;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = -0.15;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

		} else if(mp_2_ecp_next_state_string.substr(0,4) == "DICE") {

			sr_ecp_msg->message("DICE...");

			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = -0.1;
			coordinates_vector[1] = -0.2;
			coordinates_vector[2] = 0.0;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

		} else if(mp_2_ecp_next_state_string.substr(0,7) == "THROW 1") {

			sr_ecp_msg->message("THROW...");

			sg->reset();
			sg->set_absolute();
			std::vector <double> new_position_vector = position_vector[6];
			new_position_vector[5] = new_position_vector[4];
			new_position_vector[4] = 0.0;
			sg->load_absolute_angle_axis_trajectory_pose(new_position_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.0;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = -0.005;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

			sg->reset();
			sg->set_absolute();

			coordinates_vector[0] = -0.155;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = 0.0;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

		} else if(mp_2_ecp_next_state_string.substr(0,7) == "THROW 2") {

			sr_ecp_msg->message("THROW...");


			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.15;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = 0.0;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

			sg->reset();
			sg->set_absolute();


			coordinates_vector[0] = -0.1;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = 0.2;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}


		} else if(mp_2_ecp_next_state_string.substr(0,7) == "THROW 3") {

			sr_ecp_msg->message("THROW...");


			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.1;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = -0.2;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

			sg->reset();
			sg->set_absolute();


			coordinates_vector[0] = -0.15;
			coordinates_vector[1] = 0.0;
			coordinates_vector[2] = 0.0;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}


		} else if(mp_2_ecp_next_state_string.substr(0,7) == "THROW 4") {

			sr_ecp_msg->message("THROW 4...");

			sg->reset();
			sg->set_absolute();
			std::vector <double> new_position_vector = position_vector[6];
			new_position_vector[5] = new_position_vector[4];
			new_position_vector[4] = 0.0;
			sg->load_absolute_angle_axis_trajectory_pose(new_position_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

			sg->reset();
			sg->set_absolute();


		} else if(mp_2_ecp_next_state_string.substr(0,3) == "CUP") {

			sr_ecp_msg->message("CUP...");

			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			coordinates_vector[0] = 0.285;
			coordinates_vector[1] = 0,0;
			coordinates_vector[2] = 0,0;
			coordinates_vector[3] = 0.0;
			coordinates_vector[4] = 0.0;
			coordinates_vector[5] = 0.0;

			sg->load_relative_angle_axis_trajectory_pose(coordinates_vector);

			if (sg->calculate_interpolate()) {

				sg->Move();
			}

		} else if(mp_2_ecp_next_state_string.substr(0,4) == "SAVE") {

			sr_ecp_msg->message("SAVE...");

			std::string s_slot = mp_2_ecp_next_state_string.substr(5,6);

			int i_slot = atoi(s_slot.c_str());
			if (i_slot>= 0 && i_slot<7)
			{
				gp->Move();
				position_vector[i_slot] = gp->get_position_vector();
			}



		} else if (mp_2_ecp_next_state_string.substr(0,4) == "LOAD") {

			sr_ecp_msg->message("LOAD...");

			std::string s_slot = mp_2_ecp_next_state_string.substr(5,6);
			int i_slot = atoi(s_slot.c_str());

			if (i_slot>= 0 && i_slot<7)
			{
				sg->reset();
				sg->set_absolute();
				sg->load_absolute_angle_axis_trajectory_pose(position_vector[i_slot]);

				if (sg->calculate_interpolate()) {

					sg->Move();
				}
			}



		} else if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP){
			sr_ecp_msg->message("Starting smooth");
			sg->reset();
			sg->set_absolute();

			sr_ecp_msg->message("coordinates ready");

			//sg->load_absolute_angle_axis_trajectory_pose(coordinates_vector);

			sr_ecp_msg->message("pose loaded");

			if (sg->calculate_interpolate()) {
				sg->Move();
			}

			sr_ecp_msg->message("smooth generator configuration end");
		} else	{
			sr_ecp_msg->message("bad next state string!");
			log("ecp_t_objectfollower_pb::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
		//termination_notice();

}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_tp_pick_servovision(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
