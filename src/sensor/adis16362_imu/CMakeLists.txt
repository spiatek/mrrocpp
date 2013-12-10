add_library(adis16362_imu
	edp_s.cc
	imu.cpp
)

install(TARGETS adis16362_imu DESTINATION lib)