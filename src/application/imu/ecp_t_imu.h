#if !defined(_ECP_T_IMU_H)
#define _ECP_T_IMU_H

/*!
 * @file
 * @brief File contains ecp_imu ecp_task class declaration of imu verification
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_imu
 */

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/*!
 * @brief task to verify imu
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup edge_follow
 */
class ecp_imu : public common::task::task
{
protected:

public:

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	ecp_imu(lib::configurator &_config);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
