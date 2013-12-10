// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP postument
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#ifndef __SG_IRP6P_TFG_H
#define __SG_IRP6P_TFG_H

#include "base/edp/edp_typedefs.h"
#include "base/edp/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_tfg {

class effector;

class servo_buffer : public common::servo_buffer
{
	// Bufor polecen przysylanych z EDP_MASTER dla SERVO
	// Obiekt z algorytmem regulacji

public:
	// output_buffer
	void get_all_positions(void);
	effector &master;
	void load_hardware_interface(void);
	servo_buffer(effector &_master); // konstruktor

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
