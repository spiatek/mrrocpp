add_executable(ecp_block_pickup
	ecp_t_block_pickup.cc
	../block_move/ecp_g_block_reaching.cc
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

target_link_libraries(ecp_block_pickup ecp_robot visual_servoing)

add_executable(mp_block_pickup
	mp_t_block_pickup.cc
)
target_link_libraries(mp_block_pickup mp)

install(TARGETS ecp_block_pickup DESTINATION bin)
install(TARGETS mp_block_pickup DESTINATION bin)
