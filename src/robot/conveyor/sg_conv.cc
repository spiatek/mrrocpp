/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

// Klasa edp_conveyor_effector.
#include "robot/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "robot/hi_moxa/hi_moxa.h"
// Klasa servo_buffer.
#include "robot/conveyor/sg_conv.h"
#include "robot/conveyor/regulator_conv.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

servo_buffer::servo_buffer(effector &_master) :
		common::servo_buffer(_master), master(_master)
{
	for (int j = 0; j < lib::conveyor::NUM_OF_SERVOS; j++) {
		axe_inc_per_revolution[j] = INC_PER_REVOLUTION;
	}

	thread_id = boost::thread(boost::bind(&servo_buffer::operator(), this));
}

void servo_buffer::load_hardware_interface(void)
{
	// tablica pradow maksymalnych dla poszczegolnych osi
	//int max_current[lib::conveyor::NUM_OF_SERVOS] = { AXIS_1_MAX_CURRENT };

	const std::vector <std::string> ports_vector(mrrocpp::lib::conveyor::ports_strings, mrrocpp::lib::conveyor::ports_strings
			+ mrrocpp::lib::conveyor::LAST_MOXA_PORT_NUM + 1);
	hi =
			new hi_moxa::HI_moxa(master, mrrocpp::lib::conveyor::LAST_MOXA_PORT_NUM, ports_vector, mrrocpp::lib::conveyor::CARD_ADDRESSES, mrrocpp::lib::conveyor::MAX_INCREMENT, mrrocpp::lib::conveyor::TX_PREFIX_LEN);

	hi->init();

	hi->set_parameter_now(0, NF_COMMAND_SetDrivesMaxCurrent, mrrocpp::lib::conveyor::MAX_CURRENT_0);
	/*
	 NF_STRUCT_Regulator tmpReg = { 0x1010, 0x2020, 0x3030, 0x4040 };

	 hi->set_parameter_now(0, NF_COMMAND_SetCurrentRegulator, tmpReg);
	 */
	// conveyor uruchamia sie jako zsynchronizowany - ustawic parametr na karcie sterownika
	hi->set_parameter_now(0, NF_COMMAND_SetDrivesMisc, NF_DrivesMisc_SetSynchronized);
	//hi->set_parameter_now(0, NF_COMMAND_SetDrivesMaxCurrent, mrrocpp::lib::conveyor::MAX_CURRENT_0);

	// utworzenie tablicy regulatorow

	// Serwomechanizm 1
	regulator_ptr[0] =
			new NL_regulator_1_conv(0, 0, 0, 0.333, 6.2, 5.933, 0.35, master, common::REG_OUTPUT::CURRENT_OUTPUT); // tasmociag dla irp6 postument

	common::servo_buffer::load_hardware_interface();
}

void servo_buffer::synchronise(void)
{
	common::regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

	double synchro_step = 0.0; // zadany przyrost polozenia

	if (master.robot_test_mode) {
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();

		reply_to_EDP_MASTER();
		return;
	}

	// zerowanie regulatorow
	for (int j = 0; j < lib::conveyor::NUM_OF_SERVOS; ++j) {
		crp = regulator_ptr[j];
		crp->clear_regulator();
		hi->reset_position(j);
	}

	// zatrzymanie na chwile robota
	for (int j = 0; j < lib::conveyor::NUM_OF_SERVOS; ++j) {
		synchro_step = 0.0;
		crp = regulator_ptr[j];
		crp->insert_new_step(synchro_step);
	}

	for (int j = 0; j < 25; ++j)
		Move_1_step();

	//	kk = 0;
	clear_reply_status();
	clear_reply_status_tmp();

	reply_to_EDP_MASTER();
	return;
}

} // namespace conveyor
} // namespace edp
} // namespace mrrocpp
