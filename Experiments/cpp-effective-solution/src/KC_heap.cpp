#include "KC_heap.hpp"
#include "KC_structs.hpp"
#include "KC_searching.hpp"

extern MyHEAP* HEAP;  // показываем, что где-то в коде будет объявлена глобальная переменная HEAP - экземпляр рукописной "кучи" MyHEAP




ptrVertex::ptrVertex() {
    ind = -1;
}


ptrVertex::ptrVertex(int i) {
    ind = i;
}


Vertex* ptrVertex::operator-> () const {
    /*
    Данная функция определяет оператор "->" для класса ptrVertex. Он будет возвращать
    указатель на экземпляр Vertex, который хранится по индексу ind (который является полем класса ptrVertex)
    внутри рукописной кучи MyHEAP. Таким образом, с ptrVertex можно обращаться в точности как с указателем
    Vertex*: можно получать доступ к полям структуры Vertex через "->" от ptrVertex.

    Замечание: надпись const в задании функции говорит, что данный метод константный (то есть не меняет объект).
    */

    return &(HEAP->vertexs[ind]);
}


bool ptrVertex::operator==(ptrVertex other) {
    /*
    Функция проверки на равенство. Одинаковыми считаем те, у которых индекс
    ind одинаковый. Данная функции позволит для ptrVertex писать == NULL_Vertex.
    */

    return ind == other.ind;
}




ptrSearchNode::ptrSearchNode() {
    ind = -1;
}


ptrSearchNode::ptrSearchNode(int i) {
    ind = i;
}


SearchNode* ptrSearchNode::operator->() const {
    return &(HEAP->nodes[ind]);
}


bool ptrSearchNode::operator==(ptrSearchNode other) {
    return ind == other.ind;
}




MyHEAP::MyHEAP() {
    N = 100000;  // при инициализации MyHEAP заводим по N = 100000 экземпляров Vertex и SearchNode
    vertexs.assign(N, Vertex(0, 0, 0));  // заводим какие-угодно экземпляры - например, с параметрами 0,0,0
    nodes.assign(N, SearchNode(NULL_Vertex));

    for (int i = 0; i < N; i ++) {  // изначально все экземпляры не используются -> все индексы с 0 по N-1 добавляем
        index_free_nodes.push_back(i);
        index_free_vertexs.push_back(i);
    }
}


void MyHEAP::resize() {
    /*
    Данная функция вызывается, если все имеющиеся экземпляры используются (index_free_... опустел).
    В этом случае данная функция добавляет (выделяет из памяти) ещё N экземпляров -> суммарно становится 2*N.
    Таким образом, при каждом вызове этой функции количество имеющихся в памяти экземпляров Vertex и SearchNode
    увеличивается ровно в 2 раза -> поэтому любое наперёд заданное их количество будет достигнуто за log_2 (двоичный
    логарифм) вызовов этой функции, что немного (а значит количество обращений к реальной памяти будет небольшим (а
    значит много времени не потратится), хотя за каждое обращение будет выделять много элементов).
    */

    vertexs.resize(2 * N, Vertex(0, 0, 0));  // если вдруг текущего числа N экземпляров перестало хватать -> выделяем ещё столько же из памяти
    nodes.resize(2 * N, SearchNode(NULL_Vertex));  // (заметим, при resize вектор может поменять положение в памяти -> указатели на его элементы перестанут быть корректными...
                          // но это не страшно, так как определённый в ptrVertex и ptrSearchNode operator-> будет получать указатели заново при каждом обращении)           

    for (int i = N; i < 2 * N; i ++) {  // новые и пока ещё неиспользуемые элементы (индексы) добавляем
        index_free_nodes.push_back(i);
        index_free_vertexs.push_back(i);
    }

    N *= 2;  // увеличиваем итоговое количество
}


int MyHEAP::get_ind(vector <int> &index) {
    /*
    Данная функция должна вернуть любой элемент из поданного ей на вход вектора index.
    Этот вектор будет одним из двух (free_index_nodes или free_index_vertexs), а элементом для
    возвращения будет индекс неиспользуемого экземпляра (SearchNode или Vertex).
    */

    int ready = index.size();  // получаем размер вектора
    if (ready == 0) {  // если он 0, значит неиспользуемые элементы кончились -> делаем resize
        resize();
        ready = index.size();
    }

    int ind = index[ready-1];  // берём самый последний элемент вектора
    index.pop_back();  // выкидываем его (так как теперь экземпляр с этим индексом будет использоваться)
    return ind;
}


ptrVertex MyHEAP::new_Vertex(int i, int j, int theta) {
    /*
    Данная вершина должна выделять новую вершину Vertex из памяти (аналогично new Vertex) и возвращать
    индекс (в обёртке ptrVertex) в векторе, по которому этот экземпляр будет лежать.
    Переменные i,j,theta задают параметры (как в конструкторе класса Vertex), с которыми нужная вершина должна оказаться.
    */

    int ind = get_ind(index_free_vertexs);  // берём индекс свободного экземпляра в векторе
    vertexs[ind] = Vertex(i, j, theta);  // по этому индексу сохраняем экземпляр с нужными параметрами -> теперь он будет использоваться, как будто его выделлили из памяти
    return ptrVertex(ind);  // возвращаем индекс выделенного экземпляра
}


ptrVertex MyHEAP::new_Vertex(int i, int j, int type, size_t info) {  // далее аналогичные предыдущей функции
    int ind = get_ind(index_free_vertexs);
    vertexs[ind] = Vertex(i, j, type, info);
    return ptrVertex(ind);
}


ptrSearchNode MyHEAP::new_SearchNode(ptrVertex v) {
    int ind = get_ind(index_free_nodes);
    nodes[ind] = SearchNode(v);  // вызываем конструктор SearchNode с эти же параметром
    return ptrSearchNode(ind);
}


void MyHEAP::delete_Vertex(ptrVertex v) {  // данная функция вызывается, когда вершина v больше не используется (занимаемая ею память высвобождается)
    index_free_vertexs.push_back(v.ind);  // для этого просто её индекс добавляем в список незанятых
}


void MyHEAP::delete_SearchNode(ptrSearchNode node) {  // освобождаем память от SearchNode
    delete_Vertex(node->vertex);  // сначала удаляем содержащуюся в ней вершину
    index_free_nodes.push_back(node.ind);  // а теперь и саму Search Node помечаем неиспользуемой
}
