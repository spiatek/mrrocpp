add_library(hi_moxa
	hi_moxa.cc
	serialcomm/serialcomm.cpp
	nf/nfv2.c
)

install(TARGETS hi_moxa DESTINATION lib)
