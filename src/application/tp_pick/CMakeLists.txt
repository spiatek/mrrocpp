add_executable(mp_tp_pick
	mp_t_tp_pick.cc
)

target_link_libraries(mp_tp_pick mp discode_sensor)

install(TARGETS mp_tp_pick DESTINATION bin)

add_executable(ecp_tp_pick_servovision
    ecp_t_tp_pick_servovision.cc
)

target_link_libraries(ecp_tp_pick_servovision
	ecp_robot
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	visual_servoing
	ecp ${COMMON_LIBRARIES}
)


install(TARGETS ecp_tp_pick_servovision DESTINATION bin)
