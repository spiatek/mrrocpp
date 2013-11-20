#if !defined(_MP_T_IMU_H)
#define _MP_T_IMU_H

/*!
 * @file
 * @brief File contains edge_follow_mr mp_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

namespace mrrocpp {
namespace mp {
namespace task {

/*!
 * @brief Task that executes motion of manipulator and its gripper to follow an unknown contour
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup edge_follow
 */
class imu : public task
{
protected:

public:

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	imu(lib::configurator &_config);

	void create_robots(void);

	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
