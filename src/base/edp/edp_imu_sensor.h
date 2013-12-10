// -------------------------------------------------------------------------
//
// Definicje klasy edp_imu_sensor
//
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_IMU_SENSOR_H)
#define _EDP_IMU_SENSOR_H

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

#include "base/lib/mrmath/ForceTrans.h"
#include "base/lib/sensor_interface.h"				// klasa bazowa sensor
#include "base/edp/edp_typedefs.h"				// klasa bazowa sensor
#include "base/lib/condition_synchroniser.h"
#include "base/lib/sr/sr_vsp.h"
#include "base/lib/imudata.hpp"

namespace mrrocpp {
namespace edp {
namespace common {
class manip_effector;

enum IMU_ORDER
{
	IMU_CONFIGURE
};

}

namespace sensor {

/********** klasa czujnikow po stronie EDP **************/
class imu : public lib::sensor::sensor_interface
{
protected:
	ImuData ldata;
//	static const int IMU_BUFFER_LENGHT = 3;
//	ImuData previous_ldata;
	/*!
	 * \brief Info if the imu sensor test mode is active.
	 *
	 * It is taken from configuration data.
	 */
	bool imu_sensor_test_mode;

	int imu_buffer_length;

	common::manip_effector &master;

	virtual void connect_to_hardware(void) = 0;
	virtual void disconnect_from_hardware(void) = 0;

	// particular imu sensor configuration
	virtual void configure_particular_sensor(void) = 0;

	// particular imu sensor get reading
	virtual void get_particular_reading(void) = 0;

	virtual void wait_for_particular_event(void) = 0; // oczekiwanie na zdarzenie

	void get_reading(void);
	void wait_for_event(void); // oczekiwanie na zdarzenie
	void configure_sensor(void);

public:
	boost::mutex mtx;
	lib::condition_synchroniser new_command_synchroniser;
	bool new_edp_command;
	common::IMU_ORDER command;

	boost::circular_buffer <lib::Xyz_Angle_Axis_vector> *cb;

	lib::condition_synchroniser first_measure_synchroniser;
	lib::condition_synchroniser thread_started;
	//! komunikacja z SR
	boost::shared_ptr <lib::sr_vsp> sr_msg;

	void operator()();

	imu(common::manip_effector &_master);

	virtual ~imu();
	void set_command_execution_finish();

};
// end: class edp_imu_sensor

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
imu* return_created_edp_imu_sensor(common::manip_effector &_master);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
