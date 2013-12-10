/*
* ecp_st_limit_force_gen_test.cc
*
* Created on: 05-02-2013
* Author: mkula
*/



#include <vector>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "application/generator_tester/ecp_st_limit_force_gen_test.h"
#include "generator/ecp/limit_force/ecp_g_limit_force.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "robot/conveyor/const_conveyor.h"
//#include "ecp_mp_st_limit_force_gen_test.h"
#include "ecp_st_limit_force_gen_test.h"

#include "base/ecp/ecp_task.h"



namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

limit_force_gen_test::limit_force_gen_test(task::task & _ecp_t) : common::generator::generator(_ecp_t)
{
generator_name = mrrocpp::ecp_mp::generator::ECP_MP_LIMIT_FORCE_GEN_TEST;

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
//network_path1 = _ecp_t.config.value <std::string>("trajectory_file1", lib::MP_SECTION);
}


void limit_force_gen_test::conditional_execution()
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
// && cvgenjoint->detect_jerks(1) == 0
) {
lfgenjoint->Move();
}


//cvgenjoint->load_trajectory_from_file(network_path1.c_str());
//network_path = std::string(ecp_t.mrrocpp_network_path);

if (lfgenjoint->calculate_interpolate()
// && cvgenjoint->detect_jerks(1) == 0
) {
lfgenjoint->Move();
}

/*// JOINT ABSOLUTE

/* JOINT ABSOLUTE
sr_ecp_msg.message("Joint absolute");
lfgenjoint->reset();
lfgenjoint->set_absolute();

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = -0.104;
coordinates2[2] = -1.542;
coordinates2[3] = 0.020;
coordinates2[4] = 1.404;
coordinates2[5] = 3.358;
coordinates2[6] = -2.538;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = -0.104;
coordinates1[1] = -1.542;
coordinates1[2] = 0.020;
coordinates1[3] = 1.404;
coordinates1[4] = 3.358;
coordinates1[5] = -2.538;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
} else if (conv) {
coordinates4[0] = 1.500;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates4);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = -0.804;
coordinates2[2] = -1.342;
coordinates2[3] = 0.020;
coordinates2[4] = 1.034;
coordinates2[5] = 3.858;
coordinates2[6] = -2.738;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = -0.804;
coordinates1[1] = -1.342;
coordinates1[2] = 0.020;
coordinates1[3] = 1.034;
coordinates1[4] = 3.858;
coordinates1[5] = -2.738;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
} else if (conv) {
coordinates4[0] = 1.000;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates4);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = -0.104;
coordinates2[2] = -1.542;
coordinates2[3] = 0.020;
coordinates2[4] = 1.134;
coordinates2[5] = 3.658;
coordinates2[6] = -2.738;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = -0.104;
coordinates1[1] = -1.542;
coordinates1[2] = 0.020;
coordinates1[3] = 1.134;
coordinates1[4] = 3.658;
coordinates1[5] = -2.738;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
} else if (conv) {
coordinates4[0] = 0.500;
lfgenjoint->load_absolute_joint_trajectory_pose(coordinates4);
}

if (lfgenjoint->calculate_interpolate()) {
lfgenjoint->Move();
}*/
// JOINT ABSOLUTE END


/* //JOINT RELATIVE
sr_ecp_msg.message("Joint relative");
lfgenjoint->reset();
lfgenjoint->set_relative();

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = 0.1;
coordinates2[2] = 0.0;
coordinates2[3] = -0.2;
coordinates2[4] = 0.0;
coordinates2[5] = 0.0;
coordinates2[6] = 0.0;
lfgenjoint->load_relative_joint_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.1;
coordinates1[1] = 0.0;
coordinates1[2] = 0.0;
coordinates1[3] = 0.0;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
lfgenjoint->load_relative_joint_trajectory_pose(coordinates1);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = -0.2;
coordinates2[2] = 0.0;
coordinates2[3] = 0.1;
coordinates2[4] = 0.0;
coordinates2[5] = 0.1;
coordinates2[6] = 0.0;
lfgenjoint->load_relative_joint_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.2;
coordinates1[1] = 0.1;
coordinates1[2] = -0.1;
coordinates1[3] = 0.0;
coordinates1[4] = 0.6;
coordinates1[5] = 0.0;
lfgenjoint->load_relative_joint_trajectory_pose(coordinates1);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = 0.1;
coordinates2[2] = 0.0;
coordinates2[3] = 0.1;
coordinates2[4] = 0.0;
coordinates2[5] = -0.1;
coordinates2[6] = 0.0;
lfgenjoint->load_relative_joint_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = -0.3;
coordinates1[1] = -0.1;
coordinates1[2] = 0.1;
coordinates1[3] = 0.0;
coordinates1[4] = -0.6;
coordinates1[5] = 0.0;
lfgenjoint->load_relative_joint_trajectory_pose(coordinates1);
}

if (lfgenjoint->calculate_interpolate()) {
lfgenjoint->Move();
}
// JOINT RELATIVE END


// MOTOR ABSOLUTE
sr_ecp_msg.message("Motor absolute");
lfgenmotor->reset();
lfgenmotor->set_absolute();

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = 20.0;
coordinates2[2] = 5.0;
coordinates2[3] = 0.0;
coordinates2[4] = 20.0;
coordinates2[5] = 0.0;
coordinates2[6] = 0.0;
lfgenmotor->load_absolute_motor_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 20.0;
coordinates1[2] = 0.0;
coordinates1[3] = 0.0;
coordinates1[4] = 20.0;
coordinates1[5] = 0.0;
lfgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = 10.0;
coordinates2[2] = 10.0;
coordinates2[3] = 0.0;
coordinates2[4] = 0.0;
coordinates2[5] = 20.0;
coordinates2[6] = 0.0;
lfgenmotor->load_absolute_motor_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.0;
coordinates1[2] = 0.0;
coordinates1[3] = 50.0;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
lfgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = 0.0;
coordinates2[2] = 0.0;
coordinates2[3] = 0.0;
coordinates2[4] = 0.0;
coordinates2[5] = 0.0;
coordinates2[6] = 0.0;
lfgenmotor->load_absolute_motor_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = -10.0;
coordinates1[2] = 0.0;
coordinates1[3] = 0.0;
coordinates1[4] = 15.0;
coordinates1[5] = 0.0;
lfgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
}

if (lfgenmotor->calculate_interpolate()) {
lfgenmotor->Move();
}
// MOTOR ABSOLUTE END


// MOTOR RELATIVE
sr_ecp_msg.message("Motor relative");
lfgenmotor->reset();
lfgenmotor->set_relative();

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = 20.0;
coordinates2[2] = 0.0;
coordinates2[3] = 0.0;
coordinates2[4] = 20.0;
coordinates2[5] = 0.0;
coordinates2[6] = 0.0;
lfgenmotor->load_relative_motor_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 20.0;
coordinates1[2] = 0.0;
coordinates1[3] = 0.0;
coordinates1[4] = 20.0;
coordinates1[5] = 0.0;
lfgenmotor->load_relative_motor_trajectory_pose(coordinates1);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = -10.0;
coordinates2[2] = 0.0;
coordinates2[3] = 50.0;
coordinates2[4] = -20.0;
coordinates2[5] = 0.0;
coordinates2[6] = 0.0;
lfgenjoint->load_relative_motor_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = -10.0;
coordinates1[2] = 0.0;
coordinates1[3] = 50.0;
coordinates1[4] = -20.0;
coordinates1[5] = 0.0;
lfgenmotor->load_relative_motor_trajectory_pose(coordinates1);
}

if (track) {
coordinates2[0] = 0.0;
coordinates2[1] = -10.0;
coordinates2[2] = 0.0;
coordinates2[3] = -50.0;
coordinates2[4] = -0.0;
coordinates2[5] = 0.0;
coordinates2[6] = 0.0;
lfgenmotor->load_relative_motor_trajectory_pose(coordinates2);
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = -10.0;
coordinates1[2] = 0.0;
coordinates1[3] = -50.0;
coordinates1[4] = -0.0;
coordinates1[5] = 0.0;
lfgenmotor->load_relative_motor_trajectory_pose(coordinates1);
}

if (lfgenmotor->calculate_interpolate()) {
lfgenmotor->Move();
}
// MOTOR RELATIVE END


// EULER ABSOLUTE
sr_ecp_msg.message("Euler absolute");
lfgeneuler->reset();
lfgeneuler->set_absolute();

if (track) {
coordinates1[0] = 0.534991;
coordinates1[1] = -0.136314;
coordinates1[2] = 0.186314;
coordinates1[3] = -2.602063;
coordinates1[4] = 1.871544;
coordinates1[5] = 2.204729;
} else if (postument) {
coordinates1[0] = 0.529991;
coordinates1[1] = 1.706314;
coordinates1[2] = 0.178314;
coordinates1[3] = -2.185063;
coordinates1[4] = 1.666544;
coordinates1[5] = 2.328729;
}
lfgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.534991;
coordinates1[1] = -0.156314;
coordinates1[2] = 0.186314;
coordinates1[3] = -2.662063;
coordinates1[4] = 1.871544;
coordinates1[5] = 2.234729;
} else if (postument) {
coordinates1[0] = 0.529991;
coordinates1[1] = 1.7506314;
coordinates1[2] = 0.178314;
coordinates1[3] = -2.185063;
coordinates1[4] = 1.566544;
coordinates1[5] = 2.328729;
}
lfgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.534991;
coordinates1[1] = -0.176314;
coordinates1[2] = 0.186314;
coordinates1[3] = -2.662063;
coordinates1[4] = 1.871544;
coordinates1[5] = 2.234729;
} else if (postument) {
coordinates1[0] = 0.529991;
coordinates1[1] = 1.806314;
coordinates1[2] = 0.178314;
coordinates1[3] = -2.185063;
coordinates1[4] = 1.766544;
coordinates1[5] = 2.328729;
}
lfgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

if (lfgeneuler->calculate_interpolate()) {
lfgeneuler->Move();
}
// EULER ABSOLUTE END


// EULER RELATIVE
sr_ecp_msg.message("Euler relative");
lfgeneuler->reset();
lfgeneuler->set_relative();

if (track) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.1;
coordinates1[2] = 0.0;
coordinates1[3] = 0.15;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.1;
coordinates1[2] = 0.0;
coordinates1[3] = 0.15;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
}
lfgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.0;
coordinates1[1] = -0.1;
coordinates1[2] = 0.1;
coordinates1[3] = -0.15;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = -0.1;
coordinates1[2] = 0.1;
coordinates1[3] = -0.15;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
}
lfgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.0;
coordinates1[2] = -0.1;
coordinates1[3] = 0.0;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.0;
coordinates1[2] = -0.1;
coordinates1[3] = 0.0;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
}
lfgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

if (lfgeneuler->calculate_interpolate()) {
lfgeneuler->Move();
}
// EULER RELATIVE END


// ANGLE AXIS ABSOLUTE
sr_ecp_msg.message("Angle axis absolute");
lfgenangle->reset();
lfgenangle->set_absolute();

if (track) {
coordinates1[0] = 0.534987;
coordinates1[1] = -0.176317;
coordinates1[2] = 0.180306;
coordinates1[3] = 1.000713;
coordinates1[4] = -1.447889;
coordinates1[5] = -0.264191;
} else if (postument) {
coordinates1[0] = 0.529987;
coordinates1[1] = 1.606317;
coordinates1[2] = 0.178306;
coordinates1[3] = 1.267713;
coordinates1[4] = -1.219889;
coordinates1[5] = 0.114191;
}
lfgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.504987;
coordinates1[1] = -0.146317;
coordinates1[2] = 0.106306;
coordinates1[3] = 1.103713;
coordinates1[4] = -1.407889;
coordinates1[5] = -0.294191;
} else if (postument) {
coordinates1[0] = 0.509987;
coordinates1[1] = 1.806317;
coordinates1[2] = 0.148306;
coordinates1[3] = 1.367713;
coordinates1[4] = -1.119889;
coordinates1[5] = 0.114191;
}
lfgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.534987;
coordinates1[1] = -0.176317;
coordinates1[2] = 0.186306;
coordinates1[3] = 1.203713;
coordinates1[4] = -1.447889;
coordinates1[5] = -0.294191;
} else if (postument) {
coordinates1[0] = 0.529987;
coordinates1[1] = 1.806317;
coordinates1[2] = 0.178306;
coordinates1[3] = 1.367713;
coordinates1[4] = -1.119889;
coordinates1[5] = 0.104191;
}
lfgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

if (lfgenangle->calculate_interpolate()) {
lfgenangle->Move();
}
// ANGLE AXIS ABSOLUTE END


// ANGLE AXIS RELATIVE
sr_ecp_msg.message("Angle axis relative");
lfgenangle->reset();
lfgenangle->set_relative();

if (track) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.1;
coordinates1[2] = 0.0;
coordinates1[3] = 0.05;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.1;
coordinates1[2] = 0.0;
coordinates1[3] = 0.05;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
}
lfgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.0;
coordinates1[1] = -0.1;
coordinates1[2] = 0.1;
coordinates1[3] = -0.05;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = -0.1;
coordinates1[2] = 0.1;
coordinates1[3] = -0.05;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
}
lfgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

if (track) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.0;
coordinates1[2] = -0.1;
coordinates1[3] = 0.0;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
} else if (postument) {
coordinates1[0] = 0.0;
coordinates1[1] = 0.0;
coordinates1[2] = -0.1;
coordinates1[3] = 0.0;
coordinates1[4] = 0.0;
coordinates1[5] = 0.0;
}
lfgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

if (lfgenangle->calculate_interpolate()) {
lfgenangle->Move();
}
// ANGLE AXIS RELATIVE END*/
}

limit_force_gen_test::~limit_force_gen_test()
{

}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
