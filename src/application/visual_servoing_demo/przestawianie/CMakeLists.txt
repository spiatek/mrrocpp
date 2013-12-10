# object follower - PB - EIH
add_executable(ecp_vsdemo_przestawianie
    ecp_t_przestawianie.cc
)

target_link_libraries(ecp_vsdemo_przestawianie
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	visual_servoing
	ecp_robot
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_vsdemo_przestawianie DESTINATION bin)
