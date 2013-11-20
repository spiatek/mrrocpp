#if !defined(__MP_T_MKULA_H)
#define __MP_T_MKULA_H

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @defgroup gen_test gen_test
 * @ingroup application
 * A gen_test (with active coordinator) QNX test application
 */

class mkula : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	mkula(lib::configurator &_config);
	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
