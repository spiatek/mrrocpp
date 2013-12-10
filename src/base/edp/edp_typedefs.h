// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_TYPEDEFS_H
#define __EDP_TYPEDEFS_H

#include <string>

namespace mrrocpp {
namespace edp {
namespace common {

enum MT_ORDER
{
	MT_GET_CONTROLLER_STATE,
	MT_SET_ROBOT_MODEL,
	MT_GET_ARM_POSITION,
	MT_GET_ALGORITHMS,
	MT_MOVE_ARM,
	MT_SYNCHRONISE,
	MT_UNSYNCHRONISE
};

enum ERROR_TYPE
{
	NO_ERROR, Fatal_erroR, NonFatal_erroR_2, NonFatal_erroR_3, NonFatal_erroR_4, System_erroR
};


const std::string FORCE_SENSOR_TEST_MODE = "force_sensor_test_mode";
const std::string IMU_SENSOR_TEST_MODE = "imu_sensor_test_mode";
const std::string IMU_GRAVITY_COMPENSATION = "imu_gravity_compensation";
const std::string IMU_BUFFER_LENGTH = "imu_buffer_length";


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
