#include "KC_structs.hpp"
#include "KC_heap.hpp"
#include "KC_searching.hpp"




struct StateLatticeParams {
    /*
    Данная структура содержит все необходимые настройки для поиска алгоритмом A* на state lattice (базовое решение).
    Её смысл в следующем: перед началом поиска инициализируется эта структура, она содержит необходимое для поиска:
    старт/финиш, дерево поиска, функции получения соседей и тд. Далее экземпляр этой структуры просто подаётся на 
    вход алгоритму A* и он начинает поиск.
    Если требуется провести поиск с другими настройками (с другой эвристикой, например), то не нужно менять код A* - достаточно
    лишь написать новую структуру такого вида.
    */

    Map *task_map;  // карта, где будет осуществляться поиск

    Vertex *start, *finish;  // указатели на дискретные состояния (в виде Vertex), между которыми искать траекторию на state lattice
    long double R;  // R и A - параметры, которые указывают, какие именно вершины считать целевыми относительно finish (до каких искать путь)
    int A;

    SearchTree *ast;  // указатель на дерево поиска, где будет осуществляться поиск

    string mode;  // эта переменная может быть только PRIM или COST, и указывает, какое именно базовое решение
                  // должно реализовываться с настройками данного класса:
                  // PRIM - в качестве стоимости ребра на state lattice используется длина примитива,
                  //        в качестве эвристики - евклидово расстояние до финиша;
                  // COST - в качестве стоимости ребра используется длина коллизионного следа примитива,
                  //        в качестве эвристики - octile distance

    ControlSet *control_set;  // указатель на используемый control_set
    

    StateLatticeParams(Vertex *start, Vertex *finish, Map *map, ControlSet *control_set, bool use_fast_closed = true,
                       string mode = "PRIM", long double R = 3.0, int A = 1);
    ptrVertex get_start_vertex();
    bool is_goal(ptrVertex v);
    bool check_prim(int i, int j, Primitive* prim);
    void get_successors(ptrVertex v, vector <ptrVertex> &list);
    long double compute_cost(ptrVertex v1, ptrVertex v2);
    long double heuristic(ptrVertex v);
};




struct TypesGraphParams {
    /*
    Данная структура аналогична StateLatticeParams, но содержит настройки для поиска на графе типов.
    */

    Map *task_map; 

    Vertex *start, *finish;  // указатели на дискретные состояния (в виде Vertex), между которыми искать траекторию 
                             // (Важно! несмотря на то, что поиск теперь на графе типов, исходные данные для поиска - 
                             // это все равно состояния -> но по ним уже сделаем начальную типовую ячейку (это делает
                             // get_start_vertex) и определим целевые ячейки в is_goal)
    long double R; 
    int A;

    SearchTree *ast;  
    TypeInfo *type_info;  // указатель на используемый набор типов
    

    TypesGraphParams(Vertex *start, Vertex *finish, Map *map, TypeInfo *type_info, bool use_fast_closed = true,
                    long double R = 3.0, int A = 1);
    ptrVertex get_start_vertex();
    bool is_goal(ptrVertex v);
    void get_successors(ptrVertex v, vector <ptrVertex> &list);
    long double compute_cost(ptrVertex v1, ptrVertex v2);
    long double heuristic(ptrVertex v);
};
