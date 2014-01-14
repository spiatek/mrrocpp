add_executable(ecp_clg_planner
	ecp_t_clg_planner.cc
	generators/ecp_g_reach_already_localized_block.cc
	../block_move/ecp_g_block_reaching.cc
	../block_move/ecp_g_position_board.cc
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

target_link_libraries(ecp_clg_planner ecp_robot visual_servoing)

add_executable(mp_clg_planner
	mp_t_clg_planner.cc
)
target_link_libraries(mp_clg_planner mp)

install(TARGETS ecp_clg_planner DESTINATION bin)
install(TARGETS mp_clg_planner DESTINATION bin)