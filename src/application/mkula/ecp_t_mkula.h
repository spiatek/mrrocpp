#if !defined(_ECP_T_MKULA_H)
#define _ECP_T_MKULA_H

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/conveyor/ecp_r_conv.h"


#include "base/ecp/ecp_task.h"
#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"
#include "application/generator_tester/ecp_mp_st_spline_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class mkula : public common::task::task
{
protected:

public:
	/**
	 * Constructor.
	 */
	mkula(lib::configurator &_config);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
