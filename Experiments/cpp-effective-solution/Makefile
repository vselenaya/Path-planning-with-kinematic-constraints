CC = g++  # компилятор

# флаги, с которыми компилируется код
CFLAGS = -Wall -Werror -Wextra -g -Ofast -Wno-stack-protector -ffast-math -O3 -mavx -mavx2 -mfma -unroll-loops -Wno-error=terminate

OBJECTS = obj/KC_heap.o obj/KC_searching.o obj/KC_structs.o obj/KC_testing.o obj/KC_search_params.o
HEADERS = include/rassert.hpp include/common.hpp include/KC_astar.hpp include/KC_heap.hpp include/KC_searching.hpp include/KC_structs.hpp include/KC_search_params.hpp

OUTPUT = test_astar  # как называется исполняемая программа

$(OBJECTS): obj/%.o  : src/%.cpp $(HEADERS)  # компиляция
	$(CC) $(CFLAGS) -I include -c  $< -o $@

$(OUTPUT): obj $(OBJECTS)  # этап линковки в единую программу
	$(CC) $(CFLAGS) -I include -o $@ $(OBJECTS)

obj: 
	mkdir -p obj

clean:
	rm -rf obj
	rm -rf $(OUTPUT)