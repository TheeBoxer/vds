default : build

b : build
c : clean
r : rebuild
rebuild : c b



CC=g++
CFLAGS=-Wall
OUT_EXE=vds
SOURCE_FILES=$(wildcard src/rcr/vds/*.hh)
OBJECTS=$(SOURCE_FILES:.cc=.o)

INCLUDE_DIRS=$(FRUIT_DIR)
INC_PARAMS=$(foreach d, $(INCLUDE_DIRS), -I$d)

build: $(SOURCE_FILES)
	$(CC) $(CFLAGS) -o $(OUT_EXE) $(SOURCE_FILES)

clean:
	rm -f $(OUT_EXE) $(OBJECTS)
