
px4_add_library(geo_math_lib
    GeoMath.cpp
    GeoMath.hpp
)

target_compile_options(geo_math_lib PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(geo_math_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 添加GeoMath单元测试 - 这和unit_test的方式一样
px4_add_unit_gtest(SRC ${CMAKE_CURRENT_SOURCE_DIR}/GeoMathTest.cpp LINKLIBS geo_math_lib)

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
		geo_math_lib
		SlewRate
		px4_work_queue
		mathlib
	)

px4_add_module(
	MODULE modules__euler_attitude_bridge
	MAIN euler_attitude_bridge
	COMPILE_FLAGS
	SRCS
		euler_attitude_bridge.cpp
		euler_attitude_bridge.h
	MODULE_CONFIG
		euler_attitude_bridge.module.yaml
	DEPENDS
		mathlib
)
