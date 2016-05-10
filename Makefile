CC      := gcc
CXX     := g++
LD      := g++

INC     := -Isrc
CFLAGS  := -std=c++14 -Wall -Wextra -Wpedantic -g -O3 -ffast-math -march=native
LDFLAGS := -g -O3
LIBS    := -lsfml-graphics -lsfml-window -lsfml-system
OUT     := cs533

MODULES := sim vis
SRC_DIR := src $(addprefix src/,$(MODULES))
OBJ_DIR := obj $(addprefix obj/,$(MODULES))
SRC     := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
OBJ     := $(patsubst src/%.cpp,obj/%.o,$(SRC))

vpath %.cpp $(SRC_DIR)

define make-goal
$1/%.o: %.cpp
	$(CXX) $(CFLAGS) $(INC) -MMD -c $$< -o $$@
endef

all: checkdirs build

clean:
	rm -f $(OUT)
	rm -rf obj/

build: $(OBJ)
	$(LD) $^ -o $(OUT) $(LDFLAGS) $(LIBS)

checkdirs: $(OBJ_DIR)

$(OBJ_DIR):
	@mkdir -p $@

$(foreach bdir,$(OBJ_DIR),$(eval $(call make-goal,$(bdir))))

.PHONY: all checkdirs clean

-include $(OBJ:%.o=%.d)
