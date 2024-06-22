#pragma once

#include <vector>
#include <string>
#include <tuple>

#include "KC_heap.hpp"

struct Primitive;

using namespace std;




struct Vertex {
    /*
    Это общая структура, которая описывает вершину графа, в котором производится поиск.
    В данной работе в качестве вершины рассматриваются только дискретные состояния или типовые ячейки,
    поэтому код структуры содержит только соответствующие поля. 

    Замечание: некоторые поля могут быть либо у дискретного состояния, либо у типовой ячейки -> чтобы
    не тратить память под хранение обоих полей, удобно положить их в union (все поля, что лежат
    внутри одного union имеют одинаковое смещение, то есть занимают одну и ту же память, накладываясь друг
    на друга -> это позволяет экономить память (например, если в union лежат две переменные типа int, то на
    них вместе выделяется 4 байта памяти, i-ый бит этой памяти является одновременно битом каждой переменной),
    однако пользоваться одновременно полями из одного union нельзя, так как изменение одного изменит и другое
    (память-то общая для них))
    */

    int i, j;  // координаты клетки рабочего пространства (они есть и у дискретного состояния, и у типовой ячейки) 

    union {
        int theta;  // угол, номер дискретного направления (есть только у дискретного состояния)
        int info;  // дополнительная информация (с помощью которой производится склеивание) - есть только в типовой ячейке
    };

    int type;  // тип, номер конфигурации (есть только у типовой ячейки) 
               // для типовой ячейки тип всегда >= 0, а для дискретного состояния положим тип = -1 (так сможет отличать по Vertex,
               // что она обозначает - типовую ячейку или дискретное состояние)  

    Vertex (int i, int j, int theta);
    Vertex (int i, int j, int type, int info);
};




struct Primitive {
    /*
    Данная структура предназначена для хранения уже сгенерированного примитива control set.
    Так как все примитивы control set выходят из нулевых координат, то они задаются свои стартовым углом
    (дискретным направлением), под которым выходят, а также целевым состоянием. Заметим, что для описания
    целевого состояния используется экземпляр Vertex, так как он в коде играет роль и типовой ячейки, и
    дискретного состояния одновременно (отличаются они используемыми полями внутри Vertex).
    */

    int start_theta;  // начальный угол (= номер дискретного направления), из которого стартует примитив (координаты начала всегда = 0,0)
    ptrVertex goal;  // целевое состояние, куда ведет примитив
    vector <int> collision_in_i;  // два вектора, которые содержат координаты i и j клеток коллизионного следа примитива
    vector <int> collision_in_j;
    long double length;  // длина примитива
    long double collision_cost;  // стоимость его коллизионного следа
    int turning;  // на сколько примитив поворачивает

    Primitive();
    void add_collision(int i, int j);
    void calc_collision();
    ~Primitive();
};




struct ControlSet {
    /*
    Данная структура описывает набор примитивов control set.
    */

    int theta_amount;
    vector <vector <Primitive*>> control_set;  // тут для каждого номера i дискретного направления хранится список выходящих под этим направлением примитивов

    ControlSet();
    void load_primitives(string file);
    vector <Primitive*> &get_prims_by_heading(int heading);
    ~ControlSet();
};




struct TypeInfo {
    /*
    Данная структура описывает всю необходимую информацию, связанную
    с номерами конфигураций (= типами).
    */

    // вектор, в котором по типу type получаем список соседей ячеек этого типа в виде троек (di, dj, t):
    // (di, dj - сдвиг в этого соседа из текущей ячейки, t - тип соседа)
    vector <vector <tuple <int, int, int>>> successors;  

    // по номеру дискретного направления theta получаем тип начальной ячейки, в конфигурации
    // которой начинаются примитивы в этом угле:
    vector <int> start_type_by_theta;  

    // по типу type и номеру дискретного направления theta получаем, является ли ячейка этого типа целевой,
    // то есть заканчивается ли в ней примитив под углом theta:
    vector <vector <bool>> is_goal_by_theta_type;

    // по предыдущему списку может быть долго проверять, является ли ячейка целевой -> вводим ещё один список:
    // по типу type он будет выдавать номер дискретного угла (если в ячейке этого типа ВСЕ заканчивающиеся примитивы имеют
    // такой финальный угол), -1 (если эта ячейка вообще не целевая, в ней примитивы не кончаются), -2 (если в этой
    // ячейке заканчиваются примитивы под разными углами - и вот тогда понадобится предыдущий двумерный список)
    vector <int> goal_theta_by_type;  

    // по типу type -> получаем доп. информацию (в виде числа int), с помощью которой можно склеивать вершины:
    vector <int> add_info_by_type;  

    TypeInfo();
    void load_types(string file);
};