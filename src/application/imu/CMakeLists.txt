add_executable(ecp_imu
	ecp_t_imu.cc
)

target_link_libraries(ecp_imu ecp_robot)

add_executable(mp_imu
	mp_t_imu.cc
)

target_link_libraries(mp_imu mp)

install(TARGETS ecp_imu mp_imu DESTINATION bin)
