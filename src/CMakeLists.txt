#
#	pbrt-v3 CMake file
#
#	Designed for VS2015 and other generic
#
#	maintained by mwkm
#
#	This CMake script assumes CMAKE_SOURCE_DIR location = pbrt-v3/src
#


#	CMake 3.2.X required for VS2015 solution generation
#
cmake_minimum_required( VERSION 3.2 )

#	
#
project( pbrt-v3 CXX C )

#	To activate __MWKM__ patches
add_definitions( /D__MWKM__ )


#
#	Update PARSER during CMake Build file Generation
#
#	If YES, please provide FLEX/BISON binary locations below
#	Windows's flex and bison available here:
#	http://sourceforge.net/projects/winflexbison/
#
set( UPDATE_PARSER YES )
set( FLEX_BIN c:/bin/winFlexBison/win_flex.exe )
set( BISON_BIN c:/bin/winFlexBison/win_bison.exe )

if( ${UPDATE_PARSER} )

	#	Back up existing parsers 
	#
	if( EXISTS ${CMAKE_SOURCE_DIR}/core/pbrtparse.hh )
		MESSAGE( "Backing up current core/pbrtparse.hh to core/pbrtparse.hh.BAK" ) 	
		file( RENAME ${CMAKE_SOURCE_DIR}/core/pbrtparse.hh ${CMAKE_SOURCE_DIR}/core/pbrtparse.hh.BAK )
	endif()
	if( EXISTS ${CMAKE_SOURCE_DIR}/core/pbrtparse.cpp )
		MESSAGE( "Backing up current core/pbrtparse.cpp to core/pbrtparse.cpp.BAK" ) 
		file( RENAME ${CMAKE_SOURCE_DIR}/core/pbrtparse.cpp ${CMAKE_SOURCE_DIR}/core/pbrtparse.cpp.BAK )
	endif()
	if( EXISTS ${CMAKE_SOURCE_DIR}/core/pbrtlex.cpp )
		MESSAGE( "Backing up current core/pbrtlex.cpp to core/pbrtlex.cpp.BAK" ) 
		file( RENAME ${CMAKE_SOURCE_DIR}/core/pbrtlex.cpp ${CMAKE_SOURCE_DIR}/core/pbrtlex.cpp.BAK )
	endif()

	#	FLEX
	#
	set( FLEX_OUTPUT core/pbrtlex.cpp )
	MESSAGE( "Lex'ing pbrtlex.ll" ) 
	execute_process(
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND ${FLEX_BIN} -v -o ${FLEX_OUTPUT} core/pbrtlex.ll
	)
	
	#	BISON
	#
	set( BISON_OUTPUT core/pbrtparse.cpp )
	MESSAGE( "YACC'ing pbrtparse.yy" ) 
	execute_process(
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND ${BISON_BIN} -d -v -t -o ${BISON_OUTPUT} core/pbrtparse.yy
	)
	if( EXISTS ${CMAKE_SOURCE_DIR}/core/pbrtparse.hpp )
		MESSAGE( "Renaming core/pbrtparse.hpp to core/pbrtparse.hh" ) 
		file( RENAME ${CMAKE_SOURCE_DIR}/core/pbrtparse.hpp ${CMAKE_SOURCE_DIR}/core/pbrtparse.hh )
	endif()
	if( EXISTS ${CMAKE_SOURCE_DIR}/core/pbrtparse.cpp.h )
		MESSAGE( "Renaming core/pbrtparse.cpp.h to core/pbrtparse.hh" ) 
		file( RENAME ${CMAKE_SOURCE_DIR}/core/pbrtparse.cpp.h ${CMAKE_SOURCE_DIR}/core/pbrtparse.hh )
	endif()
	
endif()


#
#	PLATFORM-NEUTRAL settings
#

#	Header list
#
file( GLOB ACCELERATORS_H  accelerators/*.h )
file( GLOB CAMERAS_H       cameras/*.h ) 
file( GLOB CORE_H          core/*.h )
file( GLOB EXT_H           ext/*.h )
file( GLOB FILTERS_H       filters/*.h )
file( GLOB INTEGRATORS_H   integrators/*.h )
file( GLOB LIGHTS_H        lights/*.h )
file( GLOB MATERIALS_H     materials/*.h )
file( GLOB MEDIA_H         media/*.h )
file( GLOB SAMPLERS_H      samplers/*.h )
file( GLOB SHAPES_H        shapes/*.h )
file( GLOB TEXTURES_H      textures/*.h )
file( GLOB TOOLS_H         tools/*.h )


#	C++ source list
#
file( GLOB ACCELERATORS_CXX  accelerators/*.cpp )
file( GLOB CAMERAS_CXX       cameras/*.cpp ) 
file( GLOB CORE_CXX          core/*.cpp )
file( GLOB EXT_CXX           ext/*.cpp )
file( GLOB FILTERS_CXX       filters/*.cpp )
file( GLOB INTEGRATORS_CXX   integrators/*.cpp )
file( GLOB LIGHTS_CXX        lights/*.cpp )
file( GLOB MATERIALS_CXX     materials/*.cpp )
file( GLOB MEDIA_CXX         media/*.cpp )
file( GLOB SAMPLERS_CXX      samplers/*.cpp )
file( GLOB SHAPES_CXX        shapes/*.cpp )
file( GLOB TEXTURES_CXX      textures/*.cpp )
file( GLOB TOOLS_CXX         tools/*.cpp )
list( APPEND CORE_CXX		 core/pbrtlex.cpp core/pbrtparse.cpp )
list( REMOVE_DUPLICATES CORE_CXX )

#	Grouping for IDE
#
source_group( accelerators FILES ${ACCELERATORS_H} ${ACCELERATORS_CXX} )
source_group( cameras      FILES ${CAMERAS_H} ${CAMERAS_CXX} )
source_group( core         FILES ${CORE_H} ${CORE_CXX} )
source_group( ext          FILES ${EXT_H} ${EXT_CXX} )
source_group( filters      FILES ${FILTERS_H} ${FILTERS_CXX} )
source_group( integrators  FILES ${INTEGRATORS_H} ${INTEGRATORS_CXX} )
source_group( lights       FILES ${LIGHTS_H} ${LIGHTS_CXX} )
source_group( materials    FILES ${MATERIALS_H} ${MATERIALS_CXX} )
source_group( media        FILES ${MEDIA_H} ${MEDIA_CXX} )
source_group( samplers     FILES ${SAMPLERS_H} ${SAMPLERS_CXX} )
source_group( shapes       FILES ${SHAPES_H} ${SHAPES_CXX} )
source_group( textures     FILES ${TEXTURES_H} ${TEXTURES_CXX} )


set ( LIBPBRT_HEADERS
	${ACCELERATORS_H}
	${CAMERAS_H}
	${CORE_H}
	${EXT_H}
	${FILTERS_H}
	${INTEGRATORS_H}
	${LIGHTS_H}
	${MATERIALS_H}
	${MEDIA_H}
	${SAMPLERS_H}
	${SHAPES_H}
	${TEXTURES_H}
)


set ( LIBPBRT_CXXSRCS
	${ACCELERATORS_CXX}
	${CAMERAS_CXX}
	${CORE_CXX}
	${EXT_CXX}
	${FILTERS_CXX}
	${INTEGRATORS_CXX}
	${LIGHTS_CXX}
	${MATERIALS_CXX}
	${MEDIA_CXX}
	${SAMPLERS_CXX}
	${SHAPES_CXX}
	${TEXTURES_CXX}
)


#
#	libpbrt and other Build Targets
#

#	Main Library
add_library( libpbrt STATIC
	${LIBPBRT_HEADERS}
	${LIBPBRT_CXXSRCS}
)

#	pbrt executable
add_executable( pbrt 
	main/pbrt.cpp
)

target_link_libraries( pbrt
	libpbrt
)


#	Tools Build Targets
#
set( TOOLS_CXX
	tools/hdrtoldr.cpp
	tools/exravg.cpp
	tools/exrdiff.cpp
	tools/obj2pbrt.cpp
	tools/bsdftest.cpp
	tools/chi2test.cpp
)

foreach( toolSource ${TOOLS_CXX} )
	get_filename_component( toolName ${toolSource} NAME )
	string( REPLACE ".cpp" "" toolBinary ${toolName} )
	add_executable( ${toolBinary} ${toolSource} )
	target_link_libraries( ${toolBinary} libpbrt )
endforeach()



#
#	include directories
#
include_directories (
	"."
	"core"
)


#
# compiler flags 
#
if ( ${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC )

	# Force to always compile with W3
	if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
		string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	else()
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
	endif()
	# turn on parallel build
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	
	add_definitions(
		-D_CRT_SECURE_NO_WARNINGS
	)
	
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Werror=return-type -pedantic -std=c++11 -g")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Werror=return-type -pedantic -std=c++11 -g")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

endif()
