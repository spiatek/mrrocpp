// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP on_track
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#ifndef __SG_SARKOFAG_H
#define __SG_SARKOFAG_H

#include "base/edp/edp_typedefs.h"
#include "base/edp/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace sarkofag {
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

} // namespace sarkofag
} // namespace edp
} // namespace mrrocpp

#endif
