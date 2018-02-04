cxx = g++
cxxflags=-Wall -pedantic

csrc = 
ccsrc = $(wildcard src/rcr/*.cc) \
        $(wildcard src/rcr/beaglebone/io/file/*.cc) \
        $(wildcard src/rcr/sensors/vn/*.cc) \
        $(wildcard src/rcr/util/*.cc) \
        $(wildcard src/rcr/vds/*.cc) \
        $(wildcard src/rcr/vds/blades/*.cc) \
        $(wildcard src/rcr/vds/io/*.cc)
obj = $(patsubst src%.cc,build%.o, $(SRCS))

all: $(dir $@)

build/%.o: src/%.cc
	@mkdir -p $(dir $@)
	$(CXX) -c -o $@ $^ $(CXXFLAGS)

out = vds
inc_dirs = lib/cpp-mediator/include/
inc_params = $(foreach d, $(inc_dirs), -I$d)

vds: $(obj)
	echo $(obj)
	$(cxx) $(cxxflags) -I/lib/cpp-mediator/include -o $(out) $^ $(objs)

c : clean
clean:
	rm -f *.o *~ $(out) build/
