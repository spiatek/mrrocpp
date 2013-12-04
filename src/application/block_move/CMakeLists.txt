option (LIB_GECODE "Use Gecode library" ON)

add_executable(ecp_block_move
	ecp_t_block_move.cc
	ecp_g_position_board.cc
	ecp_g_block_reaching.cc
    ../visual_servoing/ib_eih_visual_servo.cc
    ../visual_servoing/visual_servo.cc
    ../visual_servoing/visual_servo_manager.cc
    ../visual_servoing/single_visual_servo_manager.cc
    ../visual_servoing/position_constraint.cc
    ../visual_servoing/cubic_constraint.cc
    ../visual_servoing/object_reached_termination_condition.cc
    ../visual_servoing/timeout_termination_condition.cc
    ../visual_servoing/termination_condition.cc
    ../../sensor/discode/discode_sensor.cc
)

target_link_libraries(ecp_block_move ecp_robot visual_servoing)

if (LIB_GECODE)
	include_directories(/usr/local/include/gecode)
	set(USE_GECODE 1)	
	add_executable(mp_block_move
		BlockPlanner.cc
		mp_t_block_move.cc
	)
	target_link_libraries(mp_block_move mp gecodesearch gecodeint gecodekernel gecodesupport)
else (LIB_GECODE)
	set(USE_GECODE 0)
	add_executable(mp_block_move
		mp_t_block_move.cc
	)
	target_link_libraries(mp_block_move mp)
endif (LIB_GECODE)

install(TARGETS ecp_block_move DESTINATION bin)
install(TARGETS mp_block_move DESTINATION bin)