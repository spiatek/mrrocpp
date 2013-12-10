/*!
 * @file
 * @brief File containing the declaration of the ADIS16362 IMU sensor class.
 *
 * @author yoyek
 *
 */

#if !defined(_EDP_S_ADIS16362_IMU_H)
#define _EDP_S_ADIS16362_IMU_H

#include "base/edp/edp_imu_sensor.h"
#include "imu.hpp"

namespace mrrocpp {
namespace edp {
namespace sensor {

class adis16362_imu : public imu
{
public:

	adis16362_imu(common::manip_effector &_master);
	virtual ~adis16362_imu();

	void connect_to_hardware(void);
	void disconnect_from_hardware(void);
	void configure_particular_sensor(void);
	void wait_for_particular_event(void);
	void get_particular_reading(void);

private:
	IMU* ai;
};

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif

