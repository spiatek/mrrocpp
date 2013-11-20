/*
 * ecp_st_mkula_gen.h
 *
 *  Created on: 05-02-2013
 *      Author: mkula
 */

#ifndef ECP_ST_MKULA_GEN_H_
#define ECP_ST_MKULA_GEN_H_

#include "application/mkula/ecp_mp_st_mkula_gen.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class limit_force;

class mkula_gen : public common::generator::generator
{

private:
        boost::shared_ptr <limit_force> lfgenjoint;
        boost::shared_ptr <limit_force> lfgenmotor;
        boost::shared_ptr <limit_force> lfgeneuler;
        boost::shared_ptr <limit_force> lfgenangle;

	bool track;
	bool postument;
	bool conv;
    std::string network_path;

public:
        mkula_gen(task::task & _ecp_t);
        ~mkula_gen();

	void conditional_execution();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
