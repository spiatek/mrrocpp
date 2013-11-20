#if !defined(__MP_T_askubis_DEMO_IRP6_H)
#define __MP_T_askubis_DEMO_IRP6_H

#include "base/mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @defgroup askubis_demo askubis_demo
 * @ingroup application
 * A askubis demo application
 */

class askubis_demo : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	askubis_demo(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
