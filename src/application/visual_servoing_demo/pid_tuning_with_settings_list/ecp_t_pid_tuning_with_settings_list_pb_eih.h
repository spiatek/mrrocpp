/*
 * ecp_t_pid_tuning_with_settings_list_pb_eih.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef ecp_t_pid_tuning_with_settings_list_pb_eih_H_
#define ecp_t_pid_tuning_with_settings_list_pb_eih_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "base/lib/logger.h"
#include "application/visual_servoing/visual_servoing.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/** @addtogroup servovision
 *  @{
 *
 *  Settings in the config file:
 *  [visualservo_tester]
 *  	robot_name
 *  [pid_tuning_with_settings_list_pb_eih]
 *  	step_distance (double, !=0, [m])
 *  	step_axis (int, in {0,1,2}) - Along which axis to move end effector before running VS
 *  	vs_timeout (double, >0) - how long to wait before terminating VS
 *  	number_of_probes (int, >=1)- how many times to run visual servo with single settings
 *  	msr_dir (string) - path from where data will be moved
 *  	destination_dir (string) - path where to data will be moved
 *		regulator_settings_0, regulator_settings_1, ..., regulator_settings_N - regulator settings matrices.
 *			N is a number of settings to probe. Every setting will be tested number_of_probes times.
 *			Every matrix has form of:
 *				[
 *					Kp_x Kp_y Kp_z Kp_alpha Kp_betha Kp_gamma;
 *					Ki_x Ki_y Ki_z Ki_alpha Ki_betha Ki_gamma;
 *					Kd_x Kd_y Kd_z Kd_alpha Kd_betha Kd_gamma;
 *					IntegralLimit_x IntegralLimit_y IntegralLimit_z IntegralLimit_alpha IntegralLimit_betha IntegralLimit_gamma
 *				]
 *
 *
 */
class ecp_t_pid_tuning_with_settings_list_pb_eih : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_pid_tuning_with_settings_list_pb_eih(mrrocpp::lib::configurator& config);

	void main_task_algorithm(void);

protected:
	boost::shared_ptr<regulator_pid> reg;
	boost::shared_ptr<single_visual_servo_manager> sm;
	boost::shared_ptr<visual_servo> vs;
	boost::shared_ptr<timeout_termination_condition> timeout_term_cond;
	boost::shared_ptr<object_reached_termination_condition> obj_reached_term_cond;

private:
	// distance from target position
	double step_distance;

	/** Along which axis to move end effector before running VS */
	int step_axis;

	int number_of_probes;

	std::vector<Eigen::Matrix <double, 4, 6> > regulator_settings;

	std::string msr_dir;
	std::string destination_dir;

	std::vector<double> object_reached_position, starting_position;

	boost::shared_ptr<generator::newsmooth> newsmooth_gen;


	void load_regulator_settings_from_config();

	void apply_regulator_settings(unsigned int idx);

	void run_vs();

	void move_files(int setting_idx, int probe_idx);

	static char config_section_name[128];
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_PB_H_ */
