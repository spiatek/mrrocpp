#ifndef __FORCETRANS_H
#define __FORCETRANS_H

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

class ForceTrans
{

protected:

	const short force_sensor_name;

	bool initialized;
	double tool_weight;

	lib::Ft_vector gravity_force_torque_in_base;
	lib::Ft_vector reaction_force_torque_in_wrist;
	lib::K_vector gravity_arm_in_wrist;

	lib::Homog_matrix sensor_frame;

	lib::Xi_f ft_tool_mass_center_translation;

	lib::Xi_f ft_tr_sensor_in_wrist;

	bool is_right_turn_frame;

public:
	ForceTrans(const short l_force_sensor_name, const lib::Homog_matrix & init_frame, const lib::Homog_matrix & s_frame, const double weight, const lib::K_vector & point_of_gravity, bool _is_right_turn_frame);
	void defineTool(const lib::Homog_matrix & init_frame, const double weight, const lib::K_vector & point_of_gravity);
	void synchro(const lib::Homog_matrix & init_frame);
	lib::Ft_vector getForce(const lib::Ft_vector _inputForceTorque, const lib::Homog_matrix curr_frame);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace lib
} // namespace mrrocpp

#endif
