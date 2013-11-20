/*
 * ecp_st_mkula_gen.cc
 *
 *  Created on: 05-02-2013
 *      Author: mkula
 */



#include <vector>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "application/mkula/ecp_st_mkula_gen.h"
#include "generator/ecp/limit_force/ecp_g_limit_force.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/conveyor/const_conveyor.h"

#include "ecp_st_mkula_gen.h"

#include "base/ecp/ecp_task.h"



namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

mkula_gen::mkula_gen(task::task & _ecp_t) : common::generator::generator(_ecp_t)
{
	generator_name = mrrocpp::ecp_mp::generator::ECP_MP_MKULA;

	if (_ecp_t.ecp_m_robot->robot_name == lib::irp6p_m::ROBOT_NAME) {
		lfgenjoint = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_JOINT, 6);

		lfgenmotor = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_MOTOR, 6);

		track = false;
		postument = true;
		conv = false;

		lfgeneuler = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6);

		lfgenangle = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);

	} else if (_ecp_t.ecp_m_robot->robot_name == lib::irp6ot_m::ROBOT_NAME) {
		lfgenjoint = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_JOINT, 7);

		lfgenmotor = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_MOTOR, 7);

		track = true;
		postument = false;
		conv = false;

		lfgeneuler = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6);

		lfgenangle = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);

	} else if (_ecp_t.ecp_m_robot->robot_name == lib::conveyor::ROBOT_NAME) {
		lfgenjoint = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_JOINT, 1);

		lfgenmotor = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_MOTOR, 1);

		track = false;
		postument = false;
		conv = true;

		lfgeneuler = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_XYZ_EULER_ZYZ, 1);

		lfgenangle = (boost::shared_ptr <limit_force>) new limit_force(_ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 1);

	}

	network_path = _ecp_t.config.value <std::string>("trajectory_file", lib::MP_SECTION);
}


void mkula_gen::conditional_execution()
{

	std::vector <double> coordinates1(6); //postument
		std::vector <double> coordinates2(7); //track
		std::vector <double> coordinates4(1); //conveyor


		//network_path = "../../src/application/generator_tester/optimizedTraj.trj";
		//cvgenjoint->load_coordinates_from_file(network_path.c_str());
		//cvgenjoint->Move();

		//network_path = "../../src/application/generator_tester/trajectory.trj";
		lfgenjoint->load_trajectory_from_file(network_path.c_str());
		//network_path = std::string(ecp_t.mrrocpp_network_path);

		if (lfgenjoint->calculate_interpolate()
		//	 && cvgenjoint->detect_jerks(1) == 0
		) {
			lfgenjoint->Move();
		}


		//cvgenjoint->load_trajectory_from_file(network_path1.c_str());
		//network_path = std::string(ecp_t.mrrocpp_network_path);

		if (lfgenjoint->calculate_interpolate()
		//	 && cvgenjoint->detect_jerks(1) == 0
		) {
			lfgenjoint->Move();
		}

}

mkula_gen::~mkula_gen()
{

}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


