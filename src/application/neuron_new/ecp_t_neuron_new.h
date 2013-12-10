/**
 * @file mp_t_neuron.h
 * @brief Header file for Neuron_new task class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup neuron
 * @date 13.05.2010
 */

#ifndef ECP_T_NEURON_NEW_H_
#define ECP_T_NEURON_NEW_H_

#include "base/ecp/ecp_task.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "generator/ecp/constant_velocity/ecp_g_constant_velocity.h"
#include "ecp_g_neuron_generator_new.h"
#include "generator/ecp/sleep/ecp_g_sleep.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/**
 * @brief Representation of Neuron_new task in MRROC++.
 * @details Neuron_new task is responsible for communication and proper information
 * flow on the MRROC++ side. It engages different generators to accomplish
 * the task.
 */
class Neuron_new : public task
{

private:
	/**
	 * @brief Smooth trajectory generator.
	 * @detials Smooth generator used for moving a manipulator to proper
	 * position which is the first position from a trajectory read from
	 * VSP. The trajectory itself is also stored in VSP.
	 */
	common::generator::newsmooth* smoothGenerator;

	/**
	 * @brief Trajectory generator for position from neuron VSP.
	 * @details The generator is started just after smooth generator
	 * reaches first position of a trajectory, and works until appropriate
	 * signal from VSP is sent START_BREAKING which indicate the moment
	 * when braking occurs and stops execution of generator.
	 */
	common::generator::Neuron_generator_new* neuronGenerator;

	/**
	 * @brief Communication manager between VSP and MRROC++.
	 * @details Neuron_new sensor from the point of view of this class is used
	 * to control MRROC++ form VSP side. Mainly to start entire system and
	 * sent information when the system stops working.
	 */
	ecp_mp::sensor::Neuron_sensor_new* neuronSensor;

public:
	Neuron_new(lib::configurator &_config);
	~Neuron_new();
	void mp_2_ecp_next_state_string_handler(void);
	void ecp_stop_accepted_handler();
};

} // namespace task
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_T_NEURON_H_ */
