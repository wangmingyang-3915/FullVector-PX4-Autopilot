px4_add_module(
	MODULE modules__fullvector_control
	MAIN fullvector_control
	COMPILE_FLAGS
	SRCS
		fullvector_control.cpp
		fullvector_control.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		controllib
		geo
		SlewRate
		px4_work_queue
		mathlib
	)
