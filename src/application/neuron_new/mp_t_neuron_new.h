/**
 * @file mp_t_Neuron_new.h
 * @brief Header file for Neuron_new class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup Neuron_new
 * @date 25.06.2010
 */

#ifndef MP_T_NEURON_NEW_H_
#define MP_T_NEURON_NEW_H_

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @brief Neuron_new task class declaration.
 * @details Taks for optimization trajectory with usage of neural networks.
 */
class Neuron_new : public task
{
public:
	Neuron_new(lib::configurator &_config);
	void main_task_algorithm(void);
	/// utworzenie robotow
	void create_robots(void);
	virtual ~Neuron_new();
};

} //task
} //mp
} //mrrocpp

#endif /* MP_T_NEURON_H_ */
