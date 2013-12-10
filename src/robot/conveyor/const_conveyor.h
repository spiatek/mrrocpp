#if !defined(_CONVEYOR_CONST_H)
#define _CONVEYOR_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "base/lib/impconst.h"
#include <string>

namespace mrrocpp {
namespace lib {
namespace conveyor {

/*!
 * @brief Conveyor robot label
 * @ingroup conveyor
 */
const robot_name_t ROBOT_NAME = "conveyor";

/*!
 * @brief Conveyor total number of servos
 * @ingroup conveyor
 */
const int NUM_OF_SERVOS = 1;

/*!
 * @brief IRp6 conveyor last Moxa port number [0..7]
 * @ingroup conveyor
 */
const int LAST_MOXA_PORT_NUM = 0;

/*!
 * @brief conveyor array of communication port names
 * @ingroup conveyor
 */

const std::string ports_strings[] = //{ "/dev/ttyM7" };
		{ "/dev/ttyMI15" };

/*!
 * @brief Conveyor  overcurrent threshold [mA]
 * @ingroup irp6p_tfg
 */
const int16_t MAX_CURRENT_0 = 18000;

/*!
 * @brief Conveyor max encoder increment
 * @ingroup conveyor
 */
const double MAX_INCREMENT[] = { 0 };

/*!
 * @brief Conveyor motor driver cards addresses
 * @ingroup conveyor
 */
const unsigned int CARD_ADDRESSES[] = { 7 };

/*!
 * @brief Number of command prefix bytes
 * @ingroup conveyor
 */
const int TX_PREFIX_LEN = 0;

} // namespace conveyor
} // namespace lib
} // namespace mrrocpp

#endif /* _CONVEYOR_CONST_H */
