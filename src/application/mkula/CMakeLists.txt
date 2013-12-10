add_executable(ecp_mkula
	ecp_t_mkula.cc
	ecp_st_mkula_gen.cc
        
)

target_link_libraries(ecp_mkula 
	ecp_g_limit_force ecp_robot
)

add_executable(mp_mkula
	mp_t_mkula.cc
)

target_link_libraries(mp_mkula mp)

install(TARGETS ecp_mkula mp_mkula DESTINATION bin)
