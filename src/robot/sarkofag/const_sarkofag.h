#if !defined(_SARKOFAG_CONST_H)
#define _SARKOFAG_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for Sarkofag
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sarkofag
 */

#include "base/lib/impconst.h"
#include <string>

namespace mrrocpp {
namespace lib {
namespace sarkofag {

/*!
 * @brief Sarkofag robot label
 * @ingroup sarkofag
 */
const robot_name_t ROBOT_NAME = "sarkofag";

/*!
 * @brief Sarkofag total number of servos
 * @ingroup sarkofag
 */
const int NUM_OF_SERVOS = 1;

/*!
 * @brief Sarkofag last Moxa port number [0..7]
 * @ingroup sarkofag
 */
const int LAST_MOXA_PORT_NUM = 0;

/*!
 * @brief IRp6 Sarkofag array of communication port names
 * @ingroup sarkofag
 */

const std::string ports_strings[] = { "/dev/ttyMI7" };
//const std::string ports_strings[] = { "/dev/ttyMI15" };

/*!
 * @brief Sarkofag overcurrent threshold [mA]
 * @ingroup sarkofag
 */
const int16_t MAX_CURRENT_0 = 25000;

/*!
 * @brief Sarkofag max increment
 * @ingroup sarkofag
 */
const double MAX_INCREMENT[] = { 0 };

/*!
 * @brief Sarkofag motor driver cards addresses
 * @ingroup sarkofag
 */
const unsigned int CARD_ADDRESSES[] = { 0 };

/*!
 * @brief Number of command prefix bytes
 * @ingroup sarkofag
 */
const int TX_PREFIX_LEN = 5;

}
} // namespace lib
} // namespace mrrocpp

#endif /* _SARKOFAG_CONST_H */
