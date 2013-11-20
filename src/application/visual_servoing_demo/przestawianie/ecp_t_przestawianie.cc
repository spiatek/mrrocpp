/*
 * ecp_t_przestawianie.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <sstream>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>

#include "ecp_t_przestawianie.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

#include "../ecp_mp_g_visual_servo_tester.h"

using namespace std;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

char ecp_t_przestawianie::config_section_name[128] = { "[przestawianie]" };

ecp_t_przestawianie::ecp_t_przestawianie(mrrocpp::lib::configurator& config) :
	common::task::task(config)
{
	try{
		std::string robot_name = config.value<std::string>("robot_name", "[visualservo_tester]");
		if(robot_name == lib::irp6p_m::ROBOT_NAME){
			ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
		} else if(robot_name == lib::irp6ot_m::ROBOT_NAME){
			ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
		} else {
			throw std::runtime_error("ecp_t_przestawianie option robot_name in config file has unknown value: " + robot_name);
		}

		newsmooth_gen = boost::shared_ptr<generator::newsmooth>(new generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6));

		step_distance = config.value<double>("step_distance", config_section_name);

		number_of_probes = config.value<int>("number_of_probes", config_section_name);

		load_regulator_settings_from_config();

		msr_dir = config.value<string>("msr_dir", config_section_name);
		sr_ecp_msg->message("MSR dir: " + msr_dir);

		destination_dir = config.value<string>("destination_dir", config_section_name);
		sr_ecp_msg->message("Destination dir: " + destination_dir);

		log_enabled = true;
		log_dbg_enabled = true;

		boost::shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));

		sr_ecp_msg->message("Creating regulator...");

		reg = boost::shared_ptr <regulator_pid> (new regulator_pid(config, config_section_name));

		sr_ecp_msg->message("Creating DisCODe sensor");
		boost::shared_ptr <discode_sensor> ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));
		vs = boost::shared_ptr <visual_servo> (new pb_eih_visual_servo(reg, ds, config_section_name, config));

		obj_reached_term_cond = boost::shared_ptr <object_reached_termination_condition> (new object_reached_termination_condition(config, config_section_name));
		timeout_term_cond = boost::shared_ptr <timeout_termination_condition> (new timeout_termination_condition(config.value<double>("vs_timeout", config_section_name)));

		log_dbg("ecp_t_przestawianie::ecp_t_przestawianie(): 3\n");
		sm = boost::shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));
		log_dbg("ecp_t_przestawianie::ecp_t_przestawianie(): 4\n");
		sm->add_position_constraint(cube);

		sm->add_termination_condition(obj_reached_term_cond);
		sm->add_termination_condition(timeout_term_cond);
		log_dbg("ecp_t_przestawianie: configuring visual_servo_manager\n");
		sm->configure();
	}catch(std::exception& ex){
		sr_ecp_msg->message(lib::FATAL_ERROR, string("ERROR in ecp_t_przestawianie: ") + ex.what());
		throw ex;
	}
	log_dbg("ecp_t_przestawianie: initialization completed.\n");
}

void ecp_t_przestawianie::load_regulator_settings_from_config()
{
	for(int i=0;;++i){
		char property_key[256];
		sprintf(property_key, "regulator_settings_%d", i);
		if(!config.exists(property_key, config_section_name)){
			break;
		}
		Eigen::Matrix <double, 4, 6> k = config.value<4,6>(property_key, config_section_name);
		regulator_settings.push_back(k);
	}
	log_dbg("%lu regulator settings loaded.", regulator_settings.size());
}

void ecp_t_przestawianie::apply_regulator_settings(unsigned int idx)
{
	Eigen::Matrix <double, 6, 6> Kp;
	Eigen::Matrix <double, 6, 6> Ki;
	Eigen::Matrix <double, 6, 6> Kd;
	Eigen::Matrix <double, 6, 1> error_integral_limit;

//	cout<<"Kp.diagonal().rows() = " << Kp.diagonal().rows()<<endl;
//	cout<<"Kp.diagonal().cols() = " << Kp.diagonal().cols()<<endl;
//	cout<<"regulator_settings[idx].block(0, 0, 1, 6).rows() = "<<regulator_settings[idx].block(0, 0, 1, 6).rows()<<endl;
//	cout<<"regulator_settings[idx].block(0, 0, 1, 6).cols() = "<<regulator_settings[idx].block(0, 0, 1, 6).cols()<<endl;

	Kp.setZero();
	Kp.diagonal() = regulator_settings[idx].block(0, 0, 1, 6).transpose();

	Ki.setZero();
	Ki.diagonal() = regulator_settings[idx].block(1, 0, 1, 6).transpose();

	Kd.setZero();
	Kd.diagonal() = regulator_settings[idx].block(2, 0, 1, 6).transpose();

	error_integral_limit.setZero();
	error_integral_limit = regulator_settings[idx].block(3, 0, 1, 6).transpose();

	reg->set_config(Kp, Ki, Kd);
	reg->set_error_integral_limit(error_integral_limit);
	reg->reset();
}

void ecp_t_przestawianie::main_task_algorithm(void)
{
//	char txt[256];
//	newsmooth_gen->set_debug(true);
	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {
			// dojechac do przedmiotu
			sr_ecp_msg->message("Approaching the goal...");

			sm->Move();

			if(timeout_term_cond->is_condition_met()){
				sr_ecp_msg->message("Timeout. Object not found.");
				break;
			}

			if(!obj_reached_term_cond->is_condition_met()){
				sr_ecp_msg->message("Error: Object not reached.");
				break;
			}

			sm->remove_termination_condition(obj_reached_term_cond);

			// zapamietac pozycje KR jako object_reached_position
			sr_ecp_msg->message("Saving object_reached_position");
			object_reached_position_hm = sm->get_current_position();
			lib::Xyz_Angle_Axis_vector xyz_aa;
			object_reached_position_hm.get_xyz_angle_axis(xyz_aa);
			xyz_aa.to_vector(object_reached_position);

			for(int step_axis=0; step_axis<3; ++step_axis){
				for(int setting_idx=0; setting_idx<regulator_settings.size(); ++setting_idx){
					apply_regulator_settings(setting_idx);
					for(int probe_n=0; probe_n < number_of_probes; ++probe_n){
						iteration(setting_idx, step_axis, 1, probe_n);
					}
					for(int probe_n=0; probe_n < number_of_probes; ++probe_n){
						iteration(setting_idx, step_axis, -1, probe_n);
					}
				}
			}

			sr_ecp_msg->message("Finished");
		} else {
			log("ecp_t_przestawianie::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
	}

	termination_notice();
}

void ecp_t_przestawianie::iteration(int setting_idx, int axis_idx, int direction, int probe_idx)
{
	char txt[256];
	sprintf(txt, "Running VS with axis %d, distance %lf... (probe %d)", axis_idx, direction * step_distance, probe_idx);
	sr_ecp_msg->message(txt);
	log("%s\n", txt);

	std::vector<double> starting_position;

	lib::Homog_matrix starting_position_hm = object_reached_position_hm;
	starting_position_hm(axis_idx, 3) += direction * step_distance;

	lib::Xyz_Angle_Axis_vector xyz_aa;
	starting_position_hm.get_xyz_angle_axis(xyz_aa);
	xyz_aa.to_vector(starting_position);

	// przesun KR do object_reached_position
	newsmooth_gen->reset();
	newsmooth_gen->load_absolute_angle_axis_trajectory_pose(object_reached_position);
	newsmooth_gen->calculate_interpolate();
	newsmooth_gen->Move();

	// przesun KR wzdluz osi current_axis o odleglosc step_distance
	newsmooth_gen->reset();
	newsmooth_gen->load_absolute_angle_axis_trajectory_pose(starting_position);
	newsmooth_gen->calculate_interpolate();
	newsmooth_gen->Move();

	run_vs();
	move_files(setting_idx, axis_idx, direction, probe_idx);
}

void ecp_t_przestawianie::run_vs()
{
	// uruchom VS z timeout_termination_condition i object_reached_termination_condition
	sr_ecp_msg->message("Staring logger.");

	sm->log_client->set_filename_prefix("pb-eih_vs_manager");
	vs->log_client->set_filename_prefix("pb-eih_vs");

	sm->log_client->set_connect();
	vs->log_client->set_connect();

	sr_ecp_msg->message("Staring visual servo.");

	sm->Move();

	sr_ecp_msg->message("Visual servo finished.");

	sm->log_client->set_disconnect();
	vs->log_client->set_disconnect();

	sr_ecp_msg->message("Logger stopped.");
}

void ecp_t_przestawianie::move_files(int setting_idx, int axis_idx, int direction, int probe_idx)
{
	char destDir[256];
	sprintf(destDir, "%s/Regulator%02d_Axis%d_%+d_Probe%03d/", destination_dir.c_str(), setting_idx, axis_idx, direction, probe_idx);

	// create destination_dir
	if(mkdir(destDir, 0755) != 0){
		perror("mkdir(destDir)");
		sr_ecp_msg->message(lib::FATAL_ERROR, string("Error creating dir: ") + destDir);
		return;
	}

	// create file with setting description
	string description_file = string(destDir) + "description.txt";
	ofstream ofDescription;
	ofDescription.open(description_file.c_str());
	ofDescription<<"Kp_x Kp_y Kp_z Kp_alpha Kp_betha Kp_gamma\n";
	ofDescription<<"Ki_x Ki_y Ki_z Ki_alpha Ki_betha Ki_gamma\n";
	ofDescription<<"Kd_x Kd_y Kd_z Kd_alpha Kd_betha Kd_gamma\n";
	ofDescription<<"IntegralLimit_x IntegralLimit_y IntegralLimit_z IntegralLimit_alpha IntegralLimit_betha IntegralLimit_gamma\n";
	ofDescription<<regulator_settings[setting_idx];
	ofDescription.close();

	string regulator_settings_file = string(destDir) + "regulator_settings.txt";
	ofstream ofSettings;
	ofSettings.open(regulator_settings_file.c_str());
	ofSettings<<regulator_settings[setting_idx];
	ofSettings.close();

	// move files from msr_dir to destination_dir
	DIR *dir;
	struct dirent *ent;
	dir = opendir (msr_dir.c_str());
	if (dir == NULL) {
		perror("opendir (msr_dir.c_str())");
		sr_ecp_msg->message(lib::FATAL_ERROR, string("Error opening MSR dir: ") + destDir);
		return;
	}

	while ((ent = readdir (dir)) != NULL) {
		if(strcmp(".", ent->d_name) && strcmp("..", ent->d_name)){
			printf ("found file: %s\n", ent->d_name);
			string from = msr_dir + "/" + ent->d_name;
			string to = string(destDir) + ent->d_name;
			if(rename(from.c_str(), to.c_str()) != 0){
				perror("rename(from.c_str(), to.c_str())");
				sr_ecp_msg->message(lib::FATAL_ERROR, string("Error moving file from ") + from + " to " + to);
				closedir (dir);
				return;
			}
		}
	}
	closedir (dir);
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_przestawianie(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
