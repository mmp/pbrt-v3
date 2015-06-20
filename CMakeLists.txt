CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

PROJECT ( PBRT-V3 )

ENABLE_TESTING()

SET ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake )

IF (APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# src/core/*.cpp are handled differently due to
# the presence of yacc/lex generated files whic
# may be optionally generated, especially on
# Windows which is a pain
IF (WIN32)
  SET ( PBRT_YACC_LEX_SOURCE
    src/core/pbrtlex.cpp
    src/core/pbrtparse.cpp
    )
ELSE()
  FIND_PACKAGE ( BISON REQUIRED )
  
  IF(BISON_FOUND)
    SET(BisonOutput ${CMAKE_BINARY_DIR}/pbrtparser.cpp)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${BisonOutput}
      COMMAND ${BISON_EXECUTABLE} -d -v -t
              --output=${BisonOutput}
              ${CMAKE_SOURCE_DIR}/src/core/pbrtparse.yy
      COMMENT "Generating pbrtparser.cpp"
      )
  ENDIF()

  FIND_PACKAGE ( FLEX REQUIRED )
  IF(FLEX_FOUND)
    SET(FlexOutput ${CMAKE_BINARY_DIR}/pbrtlex.cpp)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${FlexOutput}
      COMMAND ${FLEX_EXECUTABLE}
              --outfile=${FlexOutput}
              ${CMAKE_SOURCE_DIR}/src/core/pbrtlex.ll
      COMMENT "Generating pbrtlex.cpp"
      )
  ENDIF()

  SET ( PBRT_YACC_LEX_SOURCE
    ${BisonOutput}
    ${FlexOutput}
    )

ENDIF ()

SET ( PBRT_CORE_SOURCE
  src/core/api.cpp
  src/core/camera.cpp
  src/core/efloat.cpp
  src/core/error.cpp
  src/core/fileutil.cpp
  src/core/film.cpp
  src/core/filter.cpp
  src/core/floatfile.cpp
  src/core/geometry.cpp
  src/core/imageio.cpp
  src/core/integrator.cpp
  src/core/interaction.cpp
  src/core/interpolation.cpp
  src/core/light.cpp
  src/core/lowdiscrepancy.cpp
  src/core/material.cpp
  src/core/medium.cpp
  src/core/memory.cpp
  src/core/microfacet.cpp
  src/core/parallel.cpp
  src/core/paramset.cpp
  src/core/parser.cpp
  src/core/primitive.cpp
  src/core/progressreporter.cpp
  src/core/quaternion.cpp
  src/core/reflection.cpp
  src/core/sampler.cpp
  src/core/sampling.cpp
  src/core/scene.cpp
  src/core/shape.cpp
  src/core/sobolmatrices.cpp
  src/core/spectrum.cpp
  src/core/stats.cpp
  src/core/texture.cpp
  src/core/transform.cpp
  )

FILE ( GLOB PBRT_SOURCE
  src/ext/*.cpp
  src/accelerators/*.cpp
  src/cameras/*.cpp
  src/filters/*.cpp
  src/integrators/*.cpp
  src/lights/*.cpp
  src/materials/*.cpp
  src/samplers/*.cpp
  src/shapes/*.cpp
  src/textures/*.cpp
  src/media/*.cpp
  )

INCLUDE_DIRECTORIES ( src )
INCLUDE_DIRECTORIES ( src/core )

ADD_LIBRARY ( pbrt STATIC
  ${PBRT_YACC_LEX_SOURCE}
  ${PBRT_CORE_SOURCE}
  ${PBRT_SOURCE}
  )

# Main renderer
ADD_EXECUTABLE ( pbrt_exe
  src/main/pbrt.cpp
  )

SET_TARGET_PROPERTIES ( pbrt_exe
  PROPERTIES
  OUTPUT_NAME pbrt
  )

TARGET_LINK_LIBRARIES ( pbrt_exe
  pbrt
  )

# Tools
ADD_EXECUTABLE ( bsdftest
  src/tools/bsdftest.cpp
  )

ADD_EXECUTABLE ( chi2test
  src/tools/chi2test.cpp
  )

ADD_EXECUTABLE ( exrdiff
  src/tools/exrdiff.cpp
  )

ADD_EXECUTABLE ( hdrtoldr
  src/tools/hdrtoldr.cpp
  )

ADD_EXECUTABLE ( obj2pbrt
  src/tools/obj2pbrt.cpp
  )

TARGET_LINK_LIBRARIES ( bsdftest
  pbrt
  )

TARGET_LINK_LIBRARIES ( chi2test
  pbrt
  )

TARGET_LINK_LIBRARIES ( exrdiff
  pbrt
  )

TARGET_LINK_LIBRARIES ( hdrtoldr
  pbrt
  )

TARGET_LINK_LIBRARIES ( obj2pbrt
  pbrt
  )

# Unit test

FILE ( GLOB PBRT_TEST_SOURCE
  src/tests/*.cpp
  src/tests/gtest/*.cc 
  )

ADD_EXECUTABLE ( pbrt_test
  ${PBRT_TEST_SOURCE}
  )

TARGET_LINK_LIBRARIES ( pbrt_test
  pbrt
  ${CMAKE_THREAD_LIBS_INIT}
  )

ADD_TEST ( pbrt_unit_test
  pbrt_test
  )

# Installation

INSTALL ( TARGETS
  pbrt_exe
  bsdftest
  chi2test
  exrdiff
  hdrtoldr
  obj2pbrt
  DESTINATION
  bin
  )

INSTALL ( TARGETS
  pbrt
  DESTINATION
  lib
  )
