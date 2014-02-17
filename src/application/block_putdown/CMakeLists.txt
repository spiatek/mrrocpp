add_executable(ecp_block_putdown
	ecp_t_block_putdown.cc
	../block_move/ecp_g_position_board.cc
)

target_link_libraries(ecp_block_putdown ecp_robot visual_servoing)

add_executable(mp_block_putdown
	mp_t_block_putdown.cc
)
target_link_libraries(mp_block_putdown mp)

install(TARGETS ecp_block_putdown DESTINATION bin)
install(TARGETS mp_block_putdown DESTINATION bin)