// -------------------------------------------------------------------------
//                            servo_gr.h
// Definicje struktur danych i metod dla procesu SERVO_GROUP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __REGULATOR_IRP6P_TFG_H
#define __REGULATOR_IRP6P_TFG_H

//#include "base/edp/servo_gr.h"
#include "base/edp/regulator.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_tfg {

// stale dla automatu w regulatorze chwytka
const int GRIPPER_BLOCKED_TIME_PERIOD = 200;
const int MAX_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS = 1000;

const int IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS = 100;
const int IRP6_POSTUMENT_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE = 1000;
// const int IRP6_POSTUMENT_GRIPPER_SUM_PWM_MIN = 80;

const double AXIS_7_SYNCHRO_STEP_COARSE = -0.5;
const double AXIS_7_SYNCHRO_STEP_FINE = -0.2;

/*-----------------------------------------------------------------------*/
class NL_regulator_8_irp6p : public common::NL_regulator
{
	/* Klasa regulatorow konkretnych */
	// Obiekt z algorytmem regulacji
	long gripper_blocked_start_time;
	int sum_of_currents, current_index;
	int currents[MAX_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS];

public:
	NL_regulator_8_irp6p(uint8_t _axis_number, uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master, common::REG_OUTPUT _reg_output); // konstruktor

	virtual uint8_t compute_set_value(void);
	// obliczenie nastepnej wartosci zadanej dla napedu - metoda konkretna

};
// end: class NL_regulator_8
// ----------------------------------------------------------------------

}// namespace common
} // namespace edp
} // namespace mrrocpp

#endif
