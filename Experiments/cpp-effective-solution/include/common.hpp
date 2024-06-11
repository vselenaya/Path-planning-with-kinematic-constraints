/*
Данный код реализует эффективный вариант алгоритмов (поиск на state lattice и поиск на графе типов)
для их тестирования на многих картах и сценариях.
В этом файле перечислены различные настройки, на основании которых будет работать код.
Если хочется запустить тестирования с разными настройками (например, если есть два control set - один для
16 дискретных углов, другой для 8), то можно просто запустить несколько копий данной программы, и в каждой из
них описать все настройки в данном файле нужным образом.
*/

#pragma once

#define ANGLE_NUM 16  // количество дискретных направлений
#define MAX_TYPES 1500  // максимальное количество типов, которое будет использоваться на графе типов
#define MAX_TESTS 10000  // максимальное количество тестов (пары дискретных состояний, между которыми искать путь) для каждой карты
#define MAX_MAP_WIDTH 1200  // максимальные размеры карты, на которой будет производиться тестирование.
#define MAX_MAP_HEIGHT 1200
#define MAX_INFO 500  // максимальное количество различных информаций для склеивания вершин на графе типов


// !!! MAX_INFO должен быть >= ANGLE_NUM (для использования fast (списка битов в качестве CLOSED) в дереве поиска)