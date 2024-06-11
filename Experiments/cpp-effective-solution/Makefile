CC = g++  # компилятор

# ==== флаги для компиляции кода ====
ERR =  -Wall -Werror -Wextra  # стандартные флаги, которые следят за корректностью кода
OPT1 = -O3 -flto -fuse-linker-plugin  # оптимизация: O3 (максимальный уровень оптимизации), -flto и -fuse-linker-plugin позволяют оптмиизировать сразу все файлы вместе
OPT2 = -ffast-math  # делает математику быстрой (но портит точность - в данном программе нам это не очень важно)
OPT3 =  -march=native  # использование оптимизаций под процессор
CFLAGS = $(ERR) $(OPT1) $(OPT2) $(OPT3)  # собираем флаги вместе;
# при отладке полезен ещё флаг -g, он позволяет valgrind показывать номер строки с ошибкой

OBJECTS = obj/KC_heap.o obj/KC_searching.o obj/KC_structs.o obj/KC_testing.o obj/KC_search_params.o
HEADERS = include/rassert.hpp include/common.hpp include/KC_astar.hpp include/KC_heap.hpp include/KC_searching.hpp include/KC_structs.hpp include/KC_search_params.hpp

OUTPUT = test_astar  # как называется исполняемая программа

$(OBJECTS): obj/%.o  : src/%.cpp $(HEADERS)  # компиляция
	$(CC) $(CFLAGS) -I include -c  $< -o $@

$(OUTPUT): obj $(OBJECTS)  # этап линковки в единую программу
	$(CC) $(CFLAGS) -I include -o $@ $(OBJECTS)

obj: 
	mkdir -p obj
	mkdir -p res

clean:
	rm -rf obj
	rm -rf $(OUTPUT)