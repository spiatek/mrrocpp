/*
 * visual_servo_regulator_pid.h
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_PID_H_
#define VISUAL_SERVO_REGULATOR_PID_H_

#include <Eigen/Core>

#include "visual_servo_regulator.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class regulator_pid : public visual_servo_regulator
{
public:
	regulator_pid(const lib::configurator & config, const std::string& config_section_name);
	virtual ~regulator_pid();

	virtual const Eigen::Matrix <double, 6, 1>
			& compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt);

	virtual void reset();

	void set_config(Eigen::Matrix <double, 6, 6> Kp, Eigen::Matrix <double, 6, 6> Ki, Eigen::Matrix <double, 6, 6> Kd);
	void set_error_integral_limit(Eigen::Matrix <double, 6, 1> error_integral_limit);
protected:
	Eigen::Matrix <double, 6, 6> Kp, Ki, Kd;
	Eigen::Matrix <double, 6, 1> error_t_1;
	Eigen::Matrix <double, 6, 1> error_integral;
	Eigen::Matrix <double, 6, 1> max_error_integral;
	Eigen::Matrix <double, 6, 1> min_error_integral;
};

/** @} */

} // namespace
}
}

#endif /* VISUAL_SERVO_REGULATOR_PID_H_ */
