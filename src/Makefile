###########################################################################
# user-configurable section
###########################################################################

# 64 bit
MARCH=-m64

OPT=-O3
# change this to -g3 for debug builds
#OPT=-g3
# on some Linux systems, '-g -gdwarf-2' may be necessary if g++ is generating
# DWARF 4 debugging info but the installed gdb doesn't support DWARF 4 yet.
#OPT=-g -gdwarf-2

# comment out this line to enable assertions at runtime
DEFS += -DNDEBUG

#########################################################################
# nothing below this line should need to be changed (usually)
#########################################################################

ARCH = $(shell uname)

LEX=flex
YACC=bison -d -v -t
ifeq ($(ARCH),OpenBSD)
    LEXLIB = -ll
endif

ifeq ($(ARCH),Linux)
  CC=g++
  CXX=g++ -std=c++11
#  CXXLIB=-lc++
else
  CC=/usr/bin/clang
  CXX=/usr/bin/clang++ -std=c++11 -stdlib=libc++
  CXXLIB=-lstdc++
endif

LD=$(CXX) $(OPT) $(MARCH)
INCLUDE=-I. -Icore
WARN=-Wall -Wno-deprecated-register
CWD=$(shell pwd)
CXXFLAGS=$(OPT) $(MARCH) $(INCLUDE) $(WARN) $(DEFS)
CCFLAGS=$(CXXFLAGS)
LIBS=$(LEXLIB) -lm $(CPPLIB) -lpthread

LIB_CXXSRCS  = $(wildcard core/*.cpp) core/pbrtlex.cpp core/pbrtparse.cpp
LIB_CXXSRCS += $(wildcard ext/*.cpp)
LIB_CXXSRCS += $(wildcard accelerators/*.cpp cameras/*.cpp filters/*.cpp )
LIB_CXXSRCS += $(wildcard integrators/*.cpp lights/*.cpp materials/*.cpp )
LIB_CXXSRCS += $(wildcard samplers/*.cpp shapes/*.cpp textures/*.cpp media/*.cpp)

LIBOBJS = $(addprefix objs/, $(subst /,_,$(LIB_CXXSRCS:.cpp=.o)))

HEADERS = $(wildcard */*.h)

TOOLS = bin/hdrtoldr bin/exravg bin/exrdiff bin/obj2pbrt bin/bsdftest bin/chi2test

default: dirs bin/pbrt $(TOOLS) bin/pbrt_tests

bin/%: dirs

pbrt: bin/pbrt

dirs:
	/bin/mkdir -p bin objs

$(LIBOBJS): $(HEADERS)

.PHONY: dirs tools
.SECONDARY:

objs/libpbrt.a: $(LIBOBJS)
	@echo "Building the core rendering library (libpbrt.a)"
	@ar rcs $@ $(LIBOBJS)

objs/accelerators_%.o: accelerators/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/cameras_%.o: cameras/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/core_%.o: core/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/ext_%.o: ext/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/filters_%.o: filters/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/integrators_%.o: integrators/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/lights_%.o: lights/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/main_%.o: main/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/materials_%.o: materials/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/samplers_%.o: samplers/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/shapes_%.o: shapes/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/textures_%.o: textures/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/media_%.o: media/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/pbrt.o: main/pbrt.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

objs/tools_%.o: tools/%.cpp
	@echo "Building object $@"
	@$(CXX) $(CXXFLAGS) -o $@ -c $<

bin/pbrt: objs/main_pbrt.o objs/libpbrt.a
	@echo "Linking $@"
	@$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

bin/%: objs/tools_%.o objs/libpbrt.a
	@echo "Linking $@"
	@$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

core/pbrtlex.cpp: core/pbrtlex.ll core/pbrtparse.cpp
	@echo "Lex'ing pbrtlex.ll"
	@$(LEX) -o$@ core/pbrtlex.ll

core/pbrtparse.cpp: core/pbrtparse.yy
	@echo "YACC'ing pbrtparse.yy"
	@$(YACC) -o $@ core/pbrtparse.yy
	@if [ -e core/pbrtparse.cpp.h ]; then /bin/mv core/pbrtparse.cpp.h core/pbrtparse.hh; fi
	@if [ -e core/pbrtparse.hpp ]; then /bin/mv core/pbrtparse.hpp core/pbrtparse.hh; fi

bin/pbrt_tests: $(wildcard tests/*cpp) objs/libpbrt.a
	@echo "Building $@"
	@$(CXX) $(CXXFLAGS) -o $@ $^ -Itests tests/gtest/gtest-all.cc tests/gtest/gtest_main.cc $(LIBS)

clean:
	rm -f objs/* bin/* core/pbrtlex.[ch]* core/pbrtparse.[ch]*
