/*
 * hi_moxa.h
 *
 *  Created on: Nov 14, 2011
 *      Author: mwalecki
 */

#ifndef __HI_MOXA_H
#define __HI_MOXA_H

#include <termios.h>
#include <string>
#include <vector>
#include <stdint.h>

// // // // // // // //
// TERMINAL INFO
//#define T_INFO_FUNC
//#define T_INFO_CALC
//#define NFV2_TX_DEBUG
//#define NFV2_RX_DEBUG

#define STATUS_DISP_T 100

#include "base/lib/periodic_timer.h"
#include "base/edp/HardwareInterface.h"
#include "robot/hi_moxa/hi_moxa_combuf.h"

#include "serialcomm/serialcomm.hpp"
#include "nf/nfv2.h"

namespace mrrocpp {
namespace edp {
namespace common {
class motor_driven_effector;
}
namespace hi_moxa {

const std::size_t WRITE_BYTES = 10;
const std::size_t READ_BYTES = 8;
const std::size_t MOXA_SERVOS_NR = 8;
const int MAX_PARAM_SET_ATTEMPTS = 3;
const int MAX_COMM_TIMEOUTS = 3;
const int FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT = 4;

const int VOLTAGE = 48.0;

const unsigned long COMMCYCLE_TIME_NS = 2000000;

/*!
 * @brief hardware interface class
 *
 * @author mwalecki
 * @ingroup edp
 */
class HI_moxa : public common::HardwareInterface
{

public:
	/**
	 * @brief constructor
	 * @param &_master			master effector
	 * @param last_drive_n		number of drives
	 * @param ports				vector of serial port names
	 * @param *max_increments	tab of max allowed motor increments
	 */
	HI_moxa(common::motor_driven_effector &_master, int last_drive_n, std::vector <std::string> ports, const unsigned int* card_addresses, const double* max_increments, int tx_prefix_len); // Konstruktor

	/**
	 * @brief destructor
	 */
	~HI_moxa();

	/**
	 * @brief initialization of hardware interface
	 * opens serial ports, writes init values to data structures
	 */
	virtual void init();

	/**
	 * @brief write pwm to communication buffer
	 * @param drive_number		number of drive
	 * @param set_value			pwm value
	 * writes desired pwm value to drive's communication buffer
	 */
	virtual void set_pwm(int drive_number, double set_value);

	/**
	 * @brief write current to communication buffer
	 * @param drive_number		number of drive
	 * @param set_value			current value [mA]
	 * writes desired current value to drive's communication buffer
	 */
	virtual void set_current(int drive_number, double set_value);

	virtual void set_pwm_mode(int drive_number);
	virtual void set_current_mode(int drive_number);

	/**
	 * @brief write parameter to communication buffer
	 * @param drive_number		number of drive
	 * @param parameter			parameter type
	 * @param new_value			parameter value
	 */
	virtual void set_parameter(int drive_number, const int parameter, ...);

	/**
	 * @brief read motor current from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual int get_current(int drive_number);

	/**
	 * @brief read voltage aplitude
	 * @param drive_number		number of drive
	 */
	virtual float get_voltage(int drive_number);

	/**
	 * @brief read motor increment from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual double get_increment(int drive_number);

	/**
	 * @brief read motor position from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual long int get_position(int drive_number);

	/**
	 * @brief do communication cycle
	 * sends data in communication buffer to motor controllers,
	 * waits 700us for answers from controllers,
	 * writes received data to communication buffer.
	 */
	virtual uint64_t read_write_hardware(void);

	/**
	 * @brief send parameter to motor driver
	 * @param drive_number		number of drive
	 * @param parameter			parameter type
	 * @param new_value			parameter value
	 */
	virtual int set_parameter_now(int drive_number, const int parameter, ...);

	/**
	 * @brief reset all motor positions and position increments in communication buffer
	 */
	virtual void reset_counters(void);

	/**
	 * @brief start synchronization procedure
	 * @param drive_number		number of drive
	 * command motor controller to look for 'synchro zero' signal
	 */
	virtual void start_synchro(int drive_number);

	/**
	 * @brief finish synchronization procedure
	 * @param drive_number		number of drive
	 */
	virtual void finish_synchro(int drive_number);

	/**
	 * @brief unsynchronise axis
	 * @param drive_number		number of drive
	 */
	virtual void unsynchro(int drive_number);

	/**
	 * @brief read 'in synchro area' flag from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual bool in_synchro_area(int drive_number);

	/**
	 * @brief read 'all robots synchronized' flag from communication buffer
	 */
	virtual bool robot_synchronized();

	/**
	 * @brief read 'is impulse zero' flag from communication buffer
	 * @param drive_number		number of drive
	 */
	virtual bool is_impulse_zero(int drive_offset);

	/**
	 * @brief reset motor position in communication buffer
	 * @param drive_number		number of drive
	 */
	virtual void reset_position(int drive_offset);

private:
	/// communication baud rate (bps)
#if defined(B921600)
			static const speed_t BAUD = B921600;
#else
			static const speed_t BAUD = 921600;
#endif
			const int howMuchItSucks;
			/// (number of drives)-1
			const std::size_t last_drive_number;
			/// vector of serial port names
			std::vector <std::string> port_names;
			/// tab of drives addresses
			const unsigned int* drives_addresses;
			/// tab of max allowed motor position increments
			const double* ridiculous_increment;
			/// tab of port designators
			int fd[MOXA_SERVOS_NR];
			int fd_max;
			/// tab of communication class instances
			SerialComm* SerialPort[MOXA_SERVOS_NR];
			/// tab of data buffers
			struct servo_St servo_data[MOXA_SERVOS_NR];
			struct termios oldtio[MOXA_SERVOS_NR];

			/// periodic timer used for generating read_write_hardware time base
			lib::periodic_timer ptimer;

			NF_STRUCT_ComBuf NFComBuf;
			uint8_t txBuf[256];
			uint8_t txCnt;
			uint8_t rxBuf[256];
			uint8_t rxCnt;
			uint8_t rxCommandArray[256];
			uint8_t rxCommandCnt;
		};
// endof: class hardware_interface

		}// namespace hi_moxa
		} // namespace edp
		} // namespace mrrocpp

#endif // __HI_MOXA_H
