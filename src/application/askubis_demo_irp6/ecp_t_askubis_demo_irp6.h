#if !defined(_ECP_T_ASKUBIS_DEMO_IRP6_H)
#define _ECP_T_ASKUBIS_DEMO_IRP6_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class askubis_demo_irp6 : public common::task::task
{
public:
	askubis_demo_irp6(lib::configurator &_config);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
