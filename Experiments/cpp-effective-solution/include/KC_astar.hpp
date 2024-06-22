#pragma once

#include <fstream>
#include <KC_searching.hpp>
#include <KC_structs.hpp>
#include <KC_search_params.hpp>

extern MyHEAP* HEAP;




struct ResultSearch {
    /*
    Данная структура описывает то, что возвращает алгоритм A*.
    */

    bool find_path;  // найден ли путь
    int steps;  // количество шагов, затраченных алгоритмом
    ptrSearchNode final_node;  // финальная вершина в поиске (если путь нашелся, то она совпадает с последней вершиной пути -> 
                               // -> от нее путь можно восстановить) 
    
    ResultSearch(bool find_path, int steps, ptrSearchNode final_node) {
        /*
        Конструктор.
        */

        this->find_path = find_path;
        this->steps = steps;
        this->final_node = final_node;
    }


    void print(ofstream &stream, string NAME) {
        /*
        Данная функция должна вывести результат алгоритма с именем NAME в файл stream.
        */

        stream << "result " << NAME << ": " << find_path << " " << steps;
        if (find_path)
            stream << " " << final_node->g << endl;  // выводим стоимость пути (это g-значение)
        else
            stream << " " << -1 << endl;
    }
};




template <typename T>
static inline void add_start_node_to_open(T *p) {
    /*
    Данная функция создаёт и добавляет стартовую SearchNode в список OPEN.
    */

    ptrSearchNode start_node = HEAP->new_SearchNode(p->get_start_vertex());
    start_node->g = 0;  // у начальной вершины g-значение = 0
    start_node->f = p->heuristic(start_node->vertex);  // f-значение = g-значение + h-значение
    p->ast->add_to_open(start_node);
}


static inline void set_parent(void *p, ptrSearchNode current, ptrSearchNode new_node) {
    /*
    Данная функция должна установить родителя для вершины поиска new_node, которая является
    результатом раскрытия вершины из current.
    В случае поиска на state lattice всё просто: родителем new_node будет сама current. А вот
    при поиске на графе типов нет смысла в качестве родителя хранить предыдущую вершину (= типовую
    ячейку) - это расточительно. По теореме 3 из диплома мы знаем, что восстановление пути можно
    производить просто отщипывая по примитиву с конца пути, а примитив идёт из одной целевой
    ячейки (так как он там начался <=> какой-то там закончился) в другую целевую (она целевая,
    так как он в ней кончился) -> в качестве родителя типовой ячейки стоит хранить предыдущую
    целевую ячейку на пути (а восстановление пути (траектории) будет реализуемо, например, так: сначала выписать
    цепочку целевых ячеек на пути (пользуясь как раз этими родителями в вершинах), а потом, хоть
    перебором подобрать примитивы между этими целевыми ячейками, чтобы они стыковались друг с другом
    и не пересекали с препятствиями... это чуть сложнее, чем то, как делали в коде на Питоне, но зато
    позволяет в списке CLOSED запоминать только целевые ячейки, а все остальные удалять - так как ими
    всё равно пользоваться не будем).
    */

    if (current->vertex->type == -1) {  // если тип вершины = -1, то это дискретное состояние -> поиск на state lattice->
        new_node->parent = current;  // -> просто установили родителя
        return;
    }

    if (((TypesGraphParams *)p)->type_info->goal_theta_by_type[current->vertex->type] != -1)  // если current содержит в качестве вершины целевую ячейку
        new_node->parent = current;  // то current и будет родителем
    else
        new_node->parent = current->parent;  // иначе предыдущей целевой ячейкой на пути к new_node будет родитель current 

    if (((TypesGraphParams *)p)->type_info->goal_theta_by_type[new_node->vertex->type] == -1)  // если вершина самой new_node не целевая, то как уже сказано, её можно удалить при вставке в CLOSED
        new_node->mem_after_closed = 0;
}


template <typename T>
static inline ptrSearchNode StepAstar(T *p, vector <pair <ptrVertex, long double>> &succ_list) {
    /*
    Данная функция производит одну итерацию поиска алгоритмом A*:
        извлечение вершины из OPEN, её раскрытие, перемещение её в CLOSED.
    Возвращаемое значение такое: v - SearchNode, если поиск нашел путь и NULL_Node в остальных случаях.
    */

    ptrSearchNode current = p->ast->get_best_node_from_open();  // извлекаем SearchNode с минимальным f-значением 
    if (current == NULL_Node)
        return NULL_Node;

    ptrVertex v = current->vertex;  // получаем соответствующую вершину
    if (p->is_goal(v))  // дошли до целевой -> путь найден
        return current;  // возвращаем вершину поиска, на которой найден путь

    succ_list.clear();  // очищаем список соседей
    p->get_successors(v, succ_list);  // теперь наполняем соседями v
    for (auto edge: succ_list) {  // пересчитываем расстояния до соседей u у вершины v
        ptrVertex u = edge.first;
        if (p->ast->was_expanded(u) == 0) {
            ptrSearchNode new_node = HEAP->new_SearchNode(u);
            new_node->g = current->g + edge.second;  // новое расстояние до соседа
            new_node->f = new_node->g + p->heuristic(u);
            set_parent(p, current, new_node);
            p->ast->add_to_open(new_node);
        } else
            HEAP->delete_Vertex(u);  // если вершина уже раскрыта - удаляем дубликат u
    }
    
    p->ast->add_to_closed(current);  // после раскрытия помещаем вершину в список CLOSED
    return NULL_Node;  // в других случаях (если путь еще не найден), возвращаем NULL
}




template <typename T>
ResultSearch AstarSearch(T *p) {
    /*
    Данная функция запускает алгоритм A* с поданными на вход настройками поиска p.
    В качестве T может быть либо StateLatticeParams, либо TypesGraphParams, либо иная структура, у которой есть
    все необходимые настройки для поиска (функции is_goal, get_successors, и тд).
    */
    
    add_start_node_to_open(p);  // создаём и добавляем в OPEN начальную вершину поиска
    
    int step = 0;  // количество шагов алгоритма
    vector <pair <ptrVertex, long double>> list;  // инициализируем список соседей (один на весь поиск, чтобы не тратить время на его создание)
    
    while (p->ast->open_is_empty() == 0) {  // ищем путь, пока OPEN не кончился
        step += 1;
        ptrSearchNode node = StepAstar(p, list);  // делаем один шаг алгоритма A*
        if (!(node == NULL_Node))  // если вернули вершину поиска -> путь найден -> выходим из алгоритма
            return ResultSearch(1, step, node);
    }

    return ResultSearch(0, step, NULL_Node);  // если вышли из while -> так и не нашли путь
}




ResultSearch PARALL(StateLatticeParams *prims, TypesGraphParams *types, int T) {
    /*
    Данная функция реализует алгоритм PARALL_T, который производит независимый поиск сразу двумя
    алгоритмами: базовым решением с параметрами prims и альтернативным решением со склеиванием с параметром types.
    */

    add_start_node_to_open(prims);  // добавили начальные вершины в каждое дерево поиска
    add_start_node_to_open(types);

    int steps = 0;
    vector <pair <ptrVertex, long double>> list;

    while(1) {
        bool use_types = (types->ast->open_is_empty() == 0);  // эта переменная показывает, нужно ли ещё искать поиск альтернативным решением
                                                              // (если OPEN опустел, значит можно остановиться, так как путь он им не найти)
        if (prims->ast->open_is_empty() == 1)  // если OPEN базового решения опустел, значит путь точно не найти
            return ResultSearch(0, steps, NULL_Node);

        steps += 1;

        if (use_types == 1) {  // если нужно, делаем шаги альтернативным решением
            ptrSearchNode node = StepAstar(types, list);
            if (!(node == NULL_Node))
                return ResultSearch(1, steps, node);
        }
        
        if (steps % T == 0 || use_types == 0) {  // раз в T шагов (или если types уже не используем) делаем итерацию базового решения
            ptrSearchNode node = StepAstar(prims, list);
            if (!(node == NULL_Node))
                return ResultSearch(1, steps, node);
        }
    }
}
