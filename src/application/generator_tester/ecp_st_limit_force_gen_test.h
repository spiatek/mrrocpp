/*
* ecp_st_limit_force_gen_test.h
*
* Created on: 05-02-2013
* Author: mkula
*/

#ifndef ECP_ST_LIMIT_FORCE_GEN_TEST_H_
#define ECP_ST_LIMIT_FORCE_GEN_TEST_H_

#include "application/generator_tester/ecp_mp_st_limit_force_gen_test.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class limit_force;

class limit_force_gen_test : public common::generator::generator
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
        //std::string network_path1;

public:
        limit_force_gen_test(task::task & _ecp_t);
        ~limit_force_gen_test();

void conditional_execution();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
