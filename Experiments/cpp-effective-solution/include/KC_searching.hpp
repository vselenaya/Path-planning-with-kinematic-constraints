#pragma once

#include <vector>
#include <queue>
#include <unordered_set>

#include "KC_heap.hpp"
#include "KC_structs.hpp"

using namespace std;




struct Map {
    /*
    Данная структура описывает карту (дискретное рабочее пространство), на которой
    будет производиться поиск.
    */

    vector <vector <bool>> cells;  // это двумерная булева матрица, где элемент на позиции (i,j) равен 1, если соответствующая
                                   // клетка рабочего пространства занята и 0, если свободна (в ней может находиться агент).
    int width, height;  // размеры карты: ширина и высота

    Map();
    void read_file_to_cells(string file_map, bool obs=true);
    bool in_bounds(int i, int j);
    bool traversable(int i, int j);
};




struct SearchNode {
    /*
    Данная структура описывает вершину поиска SearchNode, которая требуется в алгоритме A*.
    Как известно, вершина поиска является некоторой обёрткой над самой вершиной графа (в котором производится поиск),
    к которой добавляем некоторая информация: g,f значения, указатель на родителя (при раскрытии которого появилась
    данная SearchNode).
    */

    ptrVertex vertex;  // вершина графа, в котором производится поиск (это как бы указатель на неё)
    long double g, f;  // g- и f- значения этой вершины (h-значения вычисляется как f-g)
    ptrSearchNode parent;  // указатель на родителя

    SearchNode(ptrVertex);
};




/*
Далее описаны три структуры, которые отвечают соответственно за:
    сравнение двух SearchNode (это нужно, чтобы вытаскивать их из OPEN  порядке увеличения f-значения),
    хеширование Vertex (нужно, чтобы можно было вставлять Vertex в хеш-множество (set), которым может быть CLOSED),
    сравнение на равенство двух Vertex (нужно снова, чтобы использовать их в хеш-множестве).
(в C++ нельзя просто описать функцию - нужно завести отдельную структуру, а в ней уже описать функцию (сравнения, хеширования,...))

Замечание: все хеш-множества устроены по такому принципу: при добавлении/поиска объекта в нём от этого объекта
берётся хеш (для этого нужно его уметь хэшировать), который играет роль ключа или адреса этого объекта в множестве.
Иногда могут возникать коллизии (когда хеши двух объектов совпадают) - в этот момент программа должна понять, это
и правда один и тот же объект, или всё же разные (для этого нужно уметь проверять объекты на равенство).
*/

struct NodeCompare {
    bool operator() (ptrSearchNode const (&n1), ptrSearchNode const (&n2));
};

struct VertexHash {
    size_t operator()(ptrVertex const &v) const;
};

struct VertexEqual {
    bool operator() (ptrVertex const &v1, ptrVertex const &v2) const;
};




struct SearchTree {
    bool use_fast_closed;  // использовать ли fast_closed (если False, то в качестве CLOSED будет обычный set_closed в виде хеш-множества) 

    // описываем очередь с приоритетами, которая будет играть роль списка OPEN;
    // для этого указываем: что она будет хранить (в данном случае ptrSearchNode, которые указывают на реальные
    // используемые экземпляры SearchNode в рукописной куче MyHEAP), какой контейнер использовать (тут ничего необычного -
    // обычный std::vector подойдёт) и структуру, в которой описана функция сравнения двух элементов этой очереди (здесь
    // как раз нужна описанная ранее структура NodeCompare))
    // (подробнее: https://stackoverflow.com/questions/20826078/priority-queue-comparison)
    priority_queue <ptrSearchNode, vector <ptrSearchNode>, NodeCompare> open;
    
    // в качестве списка CLOSED можно использовать хеш-множество - std::unordered_set;
    // для этого указываем, что она будет хранить (ptrVertex - как бы указатель на Vertex) и две структуры с функцией
    // хеширования и сравнения на равенство (VertexHash и VertexEqual)
    // (подробнее: https://stackoverflow.com/questions/7222143/unordered-map-hash-function-c)
    unordered_set <ptrVertex, VertexHash, VertexEqual> set_closed;  

    // или же, в качестве CLOSED можно использовать набор битов, который для каждого элемента (каждой Vertex) хранит
    // бит=1 (если вершина раскрыта и считается находящейся в CLOSED) и бит=0, если вершина не раскрыта (используем
    // именно биты, а не True/False из bool, так как bool занимает 1 байт = 8 битов -> больше памяти...).
    // (вообще такой CLOSED должен быть гораздо быстрее, так как одно дело при вставке нового элемента в CLOSED
    // изменить один бит, а другое - взять хеш, вставить в хеш-таблицу и тд...)
    vector <uint8_t> fast_closed;  // в C++ uint8_t - это тип данных всегда равный 1 байту = 8 битов -> вектор из них
                                   // можно рассматривать как набор битов (если вектор длины N, то это набор из 8*N бит)

    // также требуется хранить вектор всех SearchNode, чьи вершины были раскрыты (это нужно, во-первых, чтобы
    // потом легко очистить память, удалив их, а во-вторых, чтобы можно было восстановить путь - для этого от финальной
    // SearchNode требуется по указателям на родителя пройти до самого начала пути - но для этого все SearchNode на пути
    // должны быть ещё не удалены!):
    vector <ptrSearchNode> expanded_nodes;


    SearchTree(bool fast);
    bool open_is_empty();
    void add_to_open(ptrSearchNode item);
    void add_to_closed(ptrSearchNode item);
    bool was_expanded(ptrVertex item);
    ptrSearchNode get_best_node_from_open();
    ~SearchTree();
};




long double euclid_dist(int i1, int j1, int i2, int j2);
long double octile_distance(int i1, int j1, int i2, int j2);
