add_library(ecp_g_limit_force
	ecp_g_limit_force.cc
)

target_link_libraries(ecp_g_limit_force ecp)

install(TARGETS ecp_g_limit_force DESTINATION lib)