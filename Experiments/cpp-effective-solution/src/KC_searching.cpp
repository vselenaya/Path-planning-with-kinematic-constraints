#include <fstream>
#include <sstream>

#include "KC_searching.hpp"
#include "KC_heap.hpp"
#include "common.hpp"
#include "rassert.hpp"

extern MyHEAP* HEAP;




Map::Map() {
    /*
    Конструктор. Изначально просто устанавливает карту пустой.
    */

    width = height = 0;
}


void Map::read_file_to_cells(string _file, bool obs) {
    /*
    Функция, которая считывает карту из файла _file. Этот файл содержит карту в виде нескольких строк одинаковой
    длины, каждая строка отвечает за очередной ряд клеток рабочего пространства. В строке находятся символы
    . (значит, что клетка свободная), #, @, T (значит что клетка занята препятствием). Перед самой картой в файле
    идут ещё 4 строки с доп информацией - так выглядят карты коллекции MovingAI.

    Также есть параметр obs - если он True, то учитываем препятствия, иначе считаем карту пустой.
    */

    ifstream file(_file);
    rassert(file.is_open() == 1, "Файл с картой не открылся! Возможно, несуществующий файл");
    
    string line;  // строка (= ряд клеток рабочего пространства)
    string temp;  // временная строка для считывания информации

    while (getline(file, line)) {  // считываем по строке line из файла с картой
        if (line.find("type") == 0) {  // первые 4 строки (первая из них со слова type начинается) пропускаем - а далее уже идёт сама карта
            getline(file, line);
            getline(file, line);
            getline(file, line);
            continue;
        }
        if (line.size() == 0)  // пустые строки пропускаем
            continue;

        vector <bool> cells_row;  // ряд клеток в виде булева вектора (в нём элемент = 1, если клетка занята и 0 иначе)

        for (char c: line) {  // рассматриваем по символу в строке
            if (c == '.')
                cells_row.push_back(0);  // свободная клетка
            else if (c == '#' || c == '@' || c == 'T') {
                if (obs)
                    cells_row.push_back(1);  // занятая клетка
                else
                    cells_row.push_back(0);
            } else
                continue;
        }

        if (cells.size() > 0)
            rassert(cells[0].size() == cells_row.size(), "Все ряды клеток на карте должны быть одинаковой длины (карта прямоугольная)!");

        cells.push_back(cells_row);  // добавляем очередной ряд клеток
    }
    file.close();

    height = cells.size();  // вычисляем размеры карты
    width = cells[0].size();

    rassert(0 <= height && height < MAX_MAP_HEIGHT && 0 <= width && width < MAX_MAP_WIDTH,
            "Слишком большая карта! Измените ограничения MAX_MAP_HEIGHT и WIDTH!");
}


bool Map::in_bounds(int i, int j) {
    /*
    Данная функция проверяет, находится ли клетка (i,j) в пределах карты.
    */

    return (0 <= i && i < height) && (0 <= j && j < width);
}


bool Map::traversable(int i, int j) {
    /*
    Данная функция проверяет, является ли клетка (i,j) свободной.
    */

    return (cells[i][j] == 0);  // у свободной клетки соответствующий элемент в матрице равен 0
}




SearchNode::SearchNode(ptrVertex v) {
    /*
    Конструктор, который создаёт SearchNode, которая будет состоять из вершины v.
    */

    vertex = v;
    parent = NULL_Node;  // изначально родителя нет, фиксируем это
    mem_after_closed = 1;  // по умолчанию помним все вершины
}




bool NodeCompare::operator() (ptrSearchNode const (&n1), ptrSearchNode const (&n2)) {
    /*
    Функция структуры NodeCompare, которая сравнивает две вершины поиска. Эта функция должна вернуть True,
    если первая меньше второй и False иначе. Так как функция нужна для упорядочивания вершин в списке OPEN,
    то сравнивать будем по f-значению.

    Замечание: std::priority_queue упорядочивает элементы по убыванию, поэтому, чтобы брать из него элементы
    по возрастанию f-значения, меньшей считаем SearchNode с большим f.
    */

    return n1->f > n2->f;  
}


// Следующий набор функций позволяет комбинировать хеши - то есть получать хеш от набора элементов, каждый
// из которых уже умеет быть хэшируемым:
// (подробнее https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x)
inline void hash_combine(std::size_t& seed) { (void) seed; }
template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    hash_combine(seed, rest...);
}


size_t VertexHash::operator()(ptrVertex const &v) const {
    /*
    Итак, данная функция должна посчитать хеш от вершины Vertex, то есть некоторое уникальное число size_t, которое
    характеризует данную вершину. Заметим, что в качестве вершины может быть либо дискретное состояние (тогда его характеризуют
    числа i, j, theta - их нужно хэшировать), либо типовая ячейка (её характеризуют координаты i,j, а также доп информация
    info - так как именно на её основе производится склеивание вершин (а склеивание вершин это как раз и означает, что
    вершины в CLOSED отождествляются - у них одинаковый хеш, они равны с точки зрения функции сравнения на равенство)). Таким
    образом, хэшировать нужно набор из чисел i, j, а также либо theta либо info - в зависимости от того, какая вершина. Но на
    самом деле внутри Verte поля theta и info находятся под одним union, а потому всегда совпадают -> можно хэшировать любое
    из них (только при использовании Vertex как дискретного состояния theta=info=дискретный угол, а при использовании как
    типовой ячейке theta=info=информация для склеивания).
    */
    size_t new_hash = 0;  // инициализируем хеш, далее в него записываем хеш от набора v->i, v->j, v->theta:
    hash_combine(new_hash, v->i, 
                           v->j,
                           v->theta);  // v->theta = v->info всегда
    return new_hash;
}


bool VertexEqual::operator() (ptrVertex const &v1, ptrVertex const &v2) const {
    /*
    Данная функция должна проверить на равенство две вершины. Как и в предыдущей функции, вершины равны,
    если у них равны i,j (координаты), а также угол направления theta (если вершина играет роль дискретного
    состояния) или информация info (если вершина - типовая ячейка). Но так как theta = info (так как они по одним
    union), то что из этого использовать - не важно.
    */
    return (v1->i == v2->i && 
            v1->j == v2->j && 
            v1->theta == v2->theta);
}




SearchTree::SearchTree(bool fast) {
    /*
    Конструктор. Просто инициализирует дерево поиска.
    */

    use_fast_closed = fast;

    if (fast)  // если используем массив в качестве CLOSED, инициализируем его
        fast_closed.assign(MAX_MAP_HEIGHT * 1ll * MAX_MAP_WIDTH * 1ll * MAX_INFO / 8 + 1, 0);  // пока он заполнен 0 (в CLOSED пусто)
}


static inline size_t index_in_closed(ptrVertex item) {
    /*
    Данная функция по вершине item вычисляет номер бита в списке fast_closed. Причём делается это однозначно (для разных
    вершин (с точки зрения функции сравнения VertexEqual::operator()) будет разный индекс, для одинаковых - одинаковый),
    так как бит с этим номером должен характеризовать, лежит ли вершина в CLOSED или не лежит.

    Так как функция VertexEqual::operator() сравнивает вершины по i,j,theta, то данная функция должна взять этот 
    набор и однозначно сопоставить ему номер бита.
    */

    // запишем имеющийся набор чисел, в комментариях после ":" указан диапазон значений
    int i = item->i;  // координата: 0 ... MAX_MAP_HEIGHT-1
    int j = item->j;  // координата: 0 ... MAX_MAP_WIDTH-1
    int theta = item->theta;  // угол направления theta = информация для отличия info: 0 ... MAX_INFO-1

    rassert(0 <= i && i < MAX_MAP_HEIGHT &&
            0 <= j && j < MAX_MAP_WIDTH &&
            0 <= theta && theta < MAX_INFO &&
            theta == item->info,
            "Некорректные компоненты item!\n");  // проверяем, что все элементы находятся в нужно диапазоне значений

    // получаем одно число num по набору i,j,theta
    // (так как все эти числа i,j,theta лежат в указанных ранее диапазонах, то такое число num однозначно для каждого набора)
    size_t num = theta * 1ll * MAX_MAP_HEIGHT * 1ll * MAX_MAP_WIDTH + i * 1ll * MAX_MAP_WIDTH + j;
    return num;
}


bool SearchTree::open_is_empty() {
    /*
    Возвращаем True, если OPEN пусть и False иначе.
    */

    return (open.size() == 0);
}


void SearchTree::add_to_open(ptrSearchNode item) {
    /*
    Добавляем очередную вершинку поиска в OPEN.
    */

    open.push(item);
}


void SearchTree::add_to_closed(ptrSearchNode item) {
    /*
    Добавляем очередную вершину (которую раскрыли) в список CLOSED. Точнее в функцию подаётся вся SearchNode,
    но в CLOSED попадает только вершина.
    !!! Перед вызовом этой функции, в поле mem_after_closed внутри item должно быть записано число 1, если эту 
    вершину поиска item нужно сохранить (для возможности восстановить путь) и 0, если можно удалять вершину. Для
    поиска на state lattice вершины нужно сохранять все, а вот для поиска на графе типов - достаточно только целевые ячейки.
    Это будет работать только при использовании fast_closed (при set_closed сохраним все SearchNode, тобы потом удобно удалить
    также Vertex, которые попали в set_closed)!
    */

    ptrVertex v = item->vertex;

    if (use_fast_closed == 0) {  // если в качестве CLOSED используется обычное множество,
        set_closed.insert(v);  // добавляем в него
        expanded_nodes.push_back(item);
        return;
    } 

    if (item->mem_after_closed == 1)  // иначе добавляем в CLOSED, но только те, что нужно
        expanded_nodes.push_back(item);
    else
        HEAP->delete_SearchNode(item);  // иначе больше нам вершина не нужна (так как попавшие в CLOSED больше не трогаются -> удаляем)  !!! так не делаем -удалим в деструкторе вместе с SearchNode

    size_t num = index_in_closed(v);  // иначе - получаем номер бита, соответствующий данной вершине
    fast_closed[num / 8] |= (1 << (num % 8));  // устанавливаем этот бит в 1 (это значит, что вершина раскрыта)
}    


bool SearchTree::was_expanded(ptrVertex item) {
    /*
    Проверяем, что вершина item раскрыта (то есть оказалась в CLOSED).
    */

    if (use_fast_closed == 0)
        return (set_closed.count(item) > 0);
    
    size_t num = index_in_closed(item);
    return (fast_closed[num / 8] & (1 << (num % 8)));  // проверяем, что нужный бит = 1
}


ptrSearchNode SearchTree::get_best_node_from_open() {
    /*
    Функция, которая возвращает лучшую SearchNode (а точнее как бы указатель на неё - виде ptrSearchNode) из
    текущего OPEN. Лучшая имеется в виду - с минимальным f-значением, как и нужно в A*.
    */

    while (1) {
        if (open_is_empty())  // если OPEN опустел, а до сих пор не нашли ->
            return NULL_Node;  // -> возвращаем, что ничего нет.

        ptrSearchNode best_node = open.top();  // извлекаем лучшую вершину (вершину с минимальным f-значением, так как именно в таком порядке в OPEN (=очереди с приоритетами) они сортируются)
        open.pop();  // выкидываем её из очереди с приоритетами

        if (was_expanded(best_node->vertex) == 1)  // если соответствующая вершина раскрыта, значит она дубликат
            HEAP->delete_SearchNode(best_node);  // удаляем ДУБЛИКАТ
        else 
            return best_node;            // если НЕ дубликат, то возвращаем найденную вершину     
    }
}


SearchTree::~SearchTree() {
    /*
    Это деструктор. Он должен очистить занимаемую память.
    */

    while (open.empty() == 0) {  // сначала удаляем все вершины поиска, находящиеся в OPEN
        ptrSearchNode node = open.top();
        HEAP->delete_SearchNode(node);
        open.pop();
    }
    
    for (ptrSearchNode node: expanded_nodes)  // теперь очищаем ещё вершины из CLOSED (удаляем SearchNode -> соответствующие Vertex тоже удалятся)
        HEAP->delete_SearchNode(node);
}
