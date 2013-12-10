/*!
 * @file
 * @brief File containing definitions of kinematic_model class methods.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

std::string kinematic_model::get_kinematic_model_label(void)
{
	return label;
}

void kinematic_model::set_kinematic_model_label(const std::string & _label)
{
	label = _label;
}

void kinematic_model::i2e_transform(const lib::JointArray& local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{
	// Direct kinematics solution.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);
}

void kinematic_model::e2i_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Inverse kinematics solution.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_frame);

}

void kinematic_model::direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{

}

void kinematic_model::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{

}

void kinematic_model::check_direct_inverse_kinematic_match(const lib::JointArray & local_desired_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
#define epsikinem 0.0001

	lib::Homog_matrix check_matrix;

	direct_kinematics_transform(local_desired_joints, check_matrix);
	lib::Homog_matrix error_matrix = local_desired_end_effector_frame * (!check_matrix);
//	std::cout << error_matrix << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				if (fabs(error_matrix(i, j) - 1.0) > epsikinem) {
					BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(DIRECT_INVERSE_KINEMATIC_MISMATCH));
				}
			} else {
				if (fabs(error_matrix(i, j)) > epsikinem) {
					std::cout << i << " j: " << j << std::endl;
					BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(DIRECT_INVERSE_KINEMATIC_MISMATCH));
				}
			}
		}
	}
}

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

