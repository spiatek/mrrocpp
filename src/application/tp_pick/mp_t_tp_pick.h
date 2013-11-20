#if !defined(__MP_T_TP_PICK_H)
#define __MP_T_TP_PICK_IRP6_H

#include "base/mp/mp_task.h"

#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>
#include <vector>

#include "DiceReading.h"
#include "DecisionVector.h"

namespace mrrocpp {
namespace mp {
namespace task {

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

/**
 * @defgroup tp_pick tp_pick
 * @ingroup application
 * A tp_pick application
 */

class tp_pick : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	tp_pick(lib::configurator &config);
	/// utworzenie robotow
	void create_robots(void);
	void main_task_algorithm(void);

	void start_position(void);
	void servovision(void);
	void double_servovision(void);
	void servo_correction(void);
	void calibration(void);
	void save_position(int i);
	void load_position(int i);
	void gripper(double);
	void turn_gripper(double);
	void set_gripper(void);
	void down(double);
	void up(void);
	void throw_dices(void);
	void pick(void);
	void pick_from_slot(int slot);
	void under_slot(int slot);
	void put_away(int slot);
	void put_to_cup(void);
	void pick_all(void);
	void recive_data(int i);
	bool make_decision(void);
	bool throw_again(int i);


	void configure_discode();
	Types::Mrrocpp_Proxy::DiceReading read_from_discode(void);
	double recive_angle();
	int recive_dots();

	DecisionVector decision;
	std::vector<int> dices;
	double angle;
	int dots;

	const std::string config_section_name;
	bool run_vs;
	int vs_settle_time;
	std::string robot_name;

private:
	boost::shared_ptr<discode_sensor> discode;

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
