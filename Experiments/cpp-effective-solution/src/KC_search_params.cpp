#include "KC_search_params.hpp"
#include "rassert.hpp"
#include "common.hpp"

extern MyHEAP *HEAP;




StateLatticeParams::StateLatticeParams(Vertex *start, Vertex *finish, Map *map, ControlSet *control_set, bool use_fast_closed,
                                       string mode, long double R, int A) {
    /*
    Конструктор. Инициализирует данный экземпляр.
    Переменная типа bool use_fast_closed указывает, использовать ли быструю версию (через список битов) CLOSED.
    */    

    task_map = map;

    this->start = start;
    this->finish = finish;
    this->R = R;
    this->A = A;

    this->control_set = control_set;

    ast = new SearchTree(use_fast_closed);  // создаём дерево поиска
    this->mode = mode;

    rassert(mode == "PRIM" || mode == "COST", "Не правильный mode в StateLatticeParams!");
}


ptrVertex StateLatticeParams::get_start_vertex() {
    /*
    Данная функция должна вернуть ptrVertex той вершины графа (в данном случае state lattice), с которой
    начинать искать траекторию.
    */

    ptrVertex v = HEAP->new_Vertex(start->i, start->j, start->theta);  // создаём копию вершины start - так как она и есть
                                                                        // вершина (дискретное состояние), откуда начинать поиск
    return v;
}


bool StateLatticeParams::is_goal(ptrVertex v) {
    /*
    Данная функция должна проверить, является ли вершина v целевой, нужно ли на ней прекратить поиск.
    Поиск мы прекращаем на вершинах, чьи координаты находятся в радиусе R от финиша, а номер дискретного угла отличается <=A.
    */

    long double dist = euclid_dist(v->i, v->j, finish->i, finish->j);  // расстояние по координатам
    int d = abs(v->theta - finish->theta);
    int angle_dist = min(d, ANGLE_NUM - d);  // расстояние по углу с учётом его цикличности по модулю ANGLE_NUM

    return (dist <= R) && (angle_dist <= A);    
}


bool StateLatticeParams::check_prim(int i, int j, Primitive* prim) { 
    /*
    Данная функция проверяет, что примитив prim из координат (i, j) не задевает препятствия.
    Замечание: prim является примитивом control set, то есть выходит из координат (0,0). Поэтому,
    так как нас в данной функции интересует его копия из (i,j), нужно не забывать делать его
    параллельный перенос на (i,j).
    */

    for (size_t k = 0; k < prim->collision_in_i.size(); k ++) {  // проверяем, что каждая клетка коллизионного следа свободна от препятствий
        int i_coll = prim->collision_in_i[k];
        int j_coll = prim->collision_in_j[k];
        if (!(task_map->in_bounds(i_coll+i, j_coll+j) && task_map->traversable(i_coll+i, j_coll+j)))  // не забываем делать параллельный перенос клетки на (i,j)
            return 0;
    }
    return 1;
}


void StateLatticeParams::get_successors(ptrVertex v, vector <ptrVertex> &list) {
    /*
    Данная функция генерирует последователей вершины v и складывает их в список list. Так как последователи
    в state lattice определяются по примитивам, то в этой же функции в вершинах-последователях сохраняется примитив
    to_prim, по которому в эту вершину попали.
    */

    for (Primitive *prim: control_set->get_prims_by_heading(v->theta)) {  // перебираем примитивы, выходящие из дискретного состояния v
                                                                            // (ими будут копии (сдвинутые параллельным переносом на v->i, v->j) тех примитивов control_set, которые начинаются под дискретным углом этого состояния)
        if (check_prim(v->i, v->j, prim) == 1) {  // если примитив prim не задевает препятствия
            ptrVertex u = HEAP->new_Vertex(v->i + prim->goal->i,
                                           v->j + prim->goal->j,
                                           prim->goal->theta);  // этот примитив ведёт в такую вершину
            u->to_prim = prim;  // запоминаем примитив
            list.push_back(u);
        }
    }
}


long double StateLatticeParams::compute_cost(ptrVertex v1, ptrVertex v2) {
    /*
    Данная функция вычисляет стоимость перехода по ребру (= примитиву) из вершины v1 в вершину v2.
    */

    rassert(v2->to_prim->goal->i + v1->i == v2->i &&
            v2->to_prim->goal->j + v1->j == v2->j &&
            v2->to_prim->start_theta == v1->theta &&
            v2->to_prim->goal->theta == v2->theta, "Примитив, сохранённый в v2 не является ребром из v1 в v2!");
    
    if (mode == "PRIM")  // в случае PRIM стоимость = лина примитива
        return v2->to_prim->length;
    else if (mode == "COST")  // в COST стоимость = длина коллизионного следа
        return v2->to_prim->collision_cost;
    else {
        return -1;
    }
}


long double StateLatticeParams::heuristic(ptrVertex v) {
    /*
    Данная вершина оценивает оставшееся расстояние до целевой вершины от вершины v.
    */

    if (mode == "PRIM")
        return euclid_dist(v->i, v->j, finish->i, finish->j);
    else if (mode == "COST")
        return octile_distance(v->i, v->j, finish->i, finish->j);
    else
        return -1;
}




TypesGraphParams::TypesGraphParams(Vertex *start, Vertex *finish, Map *map, TypeInfo *type_info, bool use_fast_closed,
                    long double R, int A) {
    /*
    Конструктор. Инициализирует данный экземпляр.
    Переменная типа bool use_fast_closed указывает, использовать ли быструю версию (через список битов) CLOSED.
    */    

    task_map = map;

    this->start = start;
    this->finish = finish;
    this->R = R;
    this->A = A;

    this->type_info = type_info;

    ast = new SearchTree(use_fast_closed);  // создаём дерево поиска
}


ptrVertex TypesGraphParams::get_start_vertex() {
    /*
    Данная функция должна вернуть ptrVertex той вершины графа (в данном случае графа типов), с которой
    начинать искать траекторию. То есть нужно вернуть начальную ячейку, сопоставленную start.
    */

    size_t start_type = type_info->start_type_by_theta[start->theta];  // получаем тип начальной ячейки, где примитивы под этм углом
    int add_info = type_info->add_info_by_type[start_type];  // получили информацию для склеивания
    ptrVertex v = HEAP->new_Vertex(start->i, start->j, start_type, add_info);  // создаём начальную типовую ячейку
    /*
    Замечание: в качестве add_info, на основании которой производить склеивание, мы указали информацию из имеющейся
    структуры. Заметим, что если вдруг нам хочется вообще не склеивать вершины, а искать на полном графе типов, достаточно
    в качестве add_info указать сам тип start_type (и аналогично при генерации соседей в качестве add_info использовать 
    сам тип ячейки) -> тогда ячейка будет склеиваться сама с собой (то есть склеиваний не будет).
    */
    return v;
}


bool TypesGraphParams::is_goal(ptrVertex v) {
    /*
    Данная функция должна проверить, является ли вершина (= типовая ячейка) v целевой, нужно ли на ней прекратить поиск.
    Этот код будет делать аналогичное, что делал код в Питоне.
    */

    long double dist = euclid_dist(v->i, v->j, finish->i, finish->j);  // расстояние по координатам
    if (dist > R)
        return 0;  // если расстояние большое, то ячейка точно не целевая

    size_t type = v->type;
    for (int ft = finish->theta - A; ft <= finish->theta + A; ft ++) {  // перебираем все углы ft, которые подходят (с точки зрения меры A) для того, чтобы типовая ячейка,
                                                                        // в которой оканчивается примитив под таким углом, считалась целевой
        int _ft = ((ft % ANGLE_NUM) + ANGLE_NUM) % ANGLE_NUM;  // делаем угол правильным - от 0 до ANGLE_NUM-1
        if (type_info->is_goal_by_theta_type[_ft][type] == 1)  // если в type и правда кончается такой примитив,
            return 1;  // ячейка целевая
    }

    return 0;    
}


void TypesGraphParams::get_successors(ptrVertex v, vector <ptrVertex> &list) {
    /*
    Данная функция генерирует последователей вершины v и складывает их в список list.
    */

    for (auto triple: type_info->successors[v->type]) {  // получаем соседей типа v->type
        int di = get<0>(triple);
        int dj = get<1>(triple);
        size_t t = get<2>(triple);

        if (task_map->in_bounds(v->i+di, v->j+dj) && task_map->traversable(v->i+di, v->j+dj))  // если они не заняты препятствием, то добавляем в массив
            list.push_back(HEAP->new_Vertex(v->i+di, v->j+dj, t, type_info->add_info_by_type[t]));
    }
}


long double TypesGraphParams::compute_cost(ptrVertex v1, ptrVertex v2) {
    /*
    Данная функция вычисляет стоимость перехода по ребру из вершины v1 в вершину v2.
    */
    
    return euclid_dist(v1->i, v1->j, v2->i, v2->j);
}


long double TypesGraphParams::heuristic(ptrVertex v) {
    /*
    Данная вершина оценивает оставшееся расстояние до целевой вершины от вершины v.
    */

    return octile_distance(v->i, v->j, finish->i, finish->j);
}
