add_executable(ecp_neuron_new
	ecp_t_neuron_new.cc
	neuron_sensor_new.cc
	ecp_g_neuron_generator_new.cc
)

target_link_libraries(ecp_neuron_new ecp_robot)

add_executable(mp_neuron_new
	mp_t_neuron_new.cc
)

target_link_libraries(mp_neuron_new
	mp
)

install(TARGETS ecp_neuron_new mp_neuron_new DESTINATION bin)
