#if !defined(_ECP_GEN_LIMIT_FORCE_H)
#define _ECP_GEN_LIMIT_FORCE_H

/*!
 * @file
 * @brief File contains limit_force generator declaration
 * @author mkula, Warsaw University of Technology
 * @ingroup generators
 */

#include "ecp_mp_g_limit_force.h"
#include "base/ecp/ecp_generator.h"
#include "generator/ecp/constant_velocity/ecp_g_constant_velocity.h"
#include "base/lib/trajectory_pose/constant_velocity_trajectory_pose.h"
#include "generator/ecp/ecp_g_multiple_position.h"




namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief generator limiting on the maximum force
 *
 * @author mkula, Warsaw University of Technology
 * @ingroup generators
 */
class limit_force : public ecp::common::generator::constant_velocity
{
private:
	float max_force_p;
	float max_force_ot;
	float max_torque_p;
	float max_torque_ot;
	lib::K_vector torque_v;
	lib::K_vector force_v;
	float force;
	float torque;
public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	limit_force(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);

	virtual ~limit_force() {};
	/**
	 * @brief generates first step of transition function
	 * @return terminal condition value
	 */
	bool next_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
