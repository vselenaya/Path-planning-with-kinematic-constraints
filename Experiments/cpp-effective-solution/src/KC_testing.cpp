#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <random>  // для random_device и другого вероятностного
#include <fstream>  // для ifstream
#include <sstream>  // для stringstream
#include <string>

#include "KC_heap.hpp"
#include "KC_searching.hpp"
#include "KC_structs.hpp"
#include "KC_astar.hpp"
#include "common.hpp"
#include "rassert.hpp"

using namespace std;

MyHEAP *HEAP;  




static void load_scenes(vector <Vertex *> &starts, vector <Vertex *> &goals, string SCEN_FILE, int samples=3) {
    /*
    Данная функция должна подготовить сценарии тестирования для фиксированной карты. Каждый сценарий является
    просто парой начальной дискретного состояния и финишного дискретного состояния, между которыми искать траекторию.
    Поэтому данная функция заполняет два вектора starts и goals этими состояниями.
    В качестве опоры функция использует файл SCEN_FILE (который находится в коллекции MovingAI вместе с файлом карты). 
    В этом файле перечислены координаты начальной и целевой клетки (они гарантированно не заняты препятствием и по логике
    должны были генерироваться случайно) -> данной функции остаётся сгенерировать угол направления для каждой из них - это
    делается случайно и равномерно.
    Переменная samples указывает по сколько направлений генерировать для клетки (стартовой и целевой).
    */

    std::random_device dev;  // создаём генератор случайных чисел
    std::mt19937 rng(dev());
    rng.seed(12345);  // фиксируем его seed, чтобы при перезапуске результаты были одинаковыми
    std::uniform_int_distribution<std::mt19937::result_type> uni_dist(1, ANGLE_NUM);  // фиксируем равномерное распределение от от 1 до ANGLE_NUM включительно
    // (подробнее: https://stackoverflow.com/questions/13445688/how-to-generate-a-random-number-in-c)

    ifstream scens(SCEN_FILE);
    string line, temp;
    getline(scens, temp);  // считываем первую строку сценария (там просто доп информация)

    while (getline(scens, line)) {
        stringstream stream(line);

        int start_i, start_j, goal_i, goal_j;
        stream >> temp >> temp >> temp >> temp >> start_j >> start_i >> goal_j >> goal_i;  // читаем из файла координаты начальной и целевой клетки
                                                                                           // (важно! сначала там указана j-координата, затем i)
        
        int start_theta, goal_theta;
        for (int i = 0; i < samples; i ++) {
            start_theta = uni_dist(rng)-1;  // генерируем стартовое и финишное направления (случайно, равномерно)
            goal_theta = uni_dist(rng)-1;  // делаем -1, чтобы угол от 0 до ANGLE_NUM-1

            starts.push_back(new Vertex(start_i, start_j, start_theta));  // создаём вершины для поиска
            goals.push_back(new Vertex(goal_i, goal_j, goal_theta));
        }
    }
    scens.close();

    cout << "Сценарии тестирования загружены!" << endl;
}




void test_algorithm(string PRIM_FILE, string TYPES_FILE, string MAP_FILE, string SCEN_FILE, string RESULT_FILE) {
    /*
    Данная функция проводит тестирования алгоритмов PRIM, COST, TYPES, PARALL_20, _100, _500 на карте
    MAP_FILE со сценариями SCEN_FILE и сохраняет результат в RESULT_FILE.
    В алгоритмах используются control set из PRIM_FILE и типы с TYPES_FILE.
    */

    Map *map = new Map();
    map->read_file_to_cells(MAP_FILE);  // загрузили карту
    cout << "Тестируется карта: " << MAP_FILE << ", размеры: " << map->width << " " << map->height << endl;

    ControlSet *control_set = new ControlSet();
    control_set->load_primitives(PRIM_FILE);  // загрузили примитивы
    cout << "Используются прмитивы из файла: " << PRIM_FILE << endl;

    TypeInfo *type_info = new TypeInfo();
    type_info->load_types(TYPES_FILE);  // загрузили типы
    cout << "Используются типы из файла: " << TYPES_FILE << endl;

    vector <Vertex *> starts;
    vector <Vertex *> goals;
    load_scenes(starts, goals, SCEN_FILE);  // загрузили сценарии
    cout << "Используются сценарии тестирования из файла: " << SCEN_FILE << endl;
    int N = starts.size();  // количество всего тестов

    ofstream resfile(RESULT_FILE);  // файл для результата тестов
    rassert(resfile.is_open() == 1, "Файла для результатов не существует!");
    resfile << "TOTAL TESTS: " << N << endl; 


    clock_t t0;
    double dur;
    ResultSearch res = ResultSearch(0, 0, NULL_Node);

    for (int i = 0; i < min(N, MAX_TESTS); i ++) {  // проводим не более MAX_TESTS тестирований

        // информация о тесте:
        resfile << "=== Test: " << i << " ===" << endl;  
        resfile << "start: " << starts[i]->i << " " << starts[i]->j << " " << starts[i]->theta << endl;
        resfile << "goal: " << goals[i]->i << " " << goals[i]->j << " " << goals[i]->theta << endl;
        resfile << "---" << endl;

        // === алгоритм PRIM ===
        StateLatticeParams *prims = new StateLatticeParams(starts[i], goals[i], map,
                                                           control_set, true, "PRIM");
        t0 = clock();
        res = AstarSearch(prims);
        dur = (double)(clock() - t0) / CLOCKS_PER_SEC;

        res.print(resfile, "PRIM");
        resfile << "time PRIMS: " << dur << endl;  // время работы алгоритма
        delete prims->ast;  // очистка памяти
        delete prims;
        if (res.find_path == 1) HEAP->delete_SearchNode(res.final_node);  // удаляем последнюю вершину (она уже не в OPEN и не в CLOSED -> ее нужно отдельно удалять)
        resfile << "---" << endl;

        
        // === алгоритм COST ===
        prims = new StateLatticeParams(starts[i], goals[i], map,
                                       control_set, true, "COST");
        t0 = clock();
        res = AstarSearch(prims);
        dur = (double)(clock() - t0) / CLOCKS_PER_SEC;

        res.print(resfile, "COST");
        resfile << "time COST: " << dur << endl;
        delete prims->ast;  // очистка памяти
        delete prims;
        if (res.find_path == 1) HEAP->delete_SearchNode(res.final_node); 
        resfile << "---" << endl;

        // === улучшение (TYPES) ===
        TypesGraphParams *types = new TypesGraphParams(starts[i], goals[i], map,
                                                       type_info, true);
        t0 = clock();
        res = AstarSearch(types);
        dur = (double)(clock() - t0) / CLOCKS_PER_SEC;

        res.print(resfile, "TYPES");
        resfile << "time TYPES: " << dur << endl;
        delete types->ast;  // очистка памяти
        delete types;
        if (res.find_path == 1) HEAP->delete_SearchNode(res.final_node); 
        resfile << "---" << endl;
        
        
        // === PARALL ===
        vector <int> Ts = {20, 100, 500};
        for (int T: Ts) {
            prims = new StateLatticeParams(starts[i], goals[i], map,
                                           control_set, true, "COST");
            types = new TypesGraphParams(starts[i], goals[i], map,
                                         type_info, true);

            t0 = clock();
            res = PARALL(prims, types, T);
            dur = (double)(clock() - t0) / CLOCKS_PER_SEC;

            res.print(resfile, "PARALL");
            resfile << "time PARALL " << T << ": " << dur << endl;
            delete types->ast;
            delete types;
            delete prims->ast;
            delete prims;
            if (res.find_path == 1) HEAP->delete_SearchNode(res.final_node); 
            resfile << "---" << endl;
        }
    }

    // очищаем память!
    for (int i = 0; i < N; i ++) {
        delete starts[i];
        delete goals[i];
    }

    resfile.close();
    delete map;
    delete control_set;
    delete type_info;

    cout << "Тестирование карты " << MAP_FILE << " завершено!!!"  << endl;
}




void make_path(Vertex *start, Vertex *finish, 
               string MAP_FILE, string PRIM_FILE, string TYPE_FILE,
               string mode, string RES_FILE) {
    /*
    Данная функция ищет путь (траекторию) из start в finish способом, указанным в mode (то есть
    либо на state lattice, либо на графе типов). Ищется на траектория на карте из MAP_FILE. В случае
    использования поиска на state lattice должен быть указан PRIM_FILE (TYPE_FILE можно указать как "").
    В случае поиска на графе типов наоборот - понадобится файл TYPE_FILE.
    Результат поиска, а именно найденный на графе путь (в случае state lattice это будет последовательность
    дискретных состояний, примитивами между которыми образуется траектория; в случае поиска на графе типов
    это будет последовательность целевых ячеек в пути - в файле KC_astar.hpp уже обсуждалось, что для
    восстановления пути на графе типов достаточно помнить только целевые ячейки на пути), будет сохранён
    в файл RES_FILE.
    */

    ofstream resfile(RES_FILE);
    rassert(resfile.is_open() == 1, "Файла для результатов не существует!");

    resfile << "Начальное состояние (i,j,theta): " << start->i << " " << start->j << " " << start->theta << endl;
    resfile << "Финиш (i,j,theta): " << finish->i << " " << finish->j << " " << finish->theta << endl;
    resfile << "Траектория ищется алгоритмом: " << mode << endl;

    Map *map = new Map();
    map->read_file_to_cells(MAP_FILE);

    if (mode == "COST" || mode == "PRIM") {
        
        ControlSet *control_set = new ControlSet();
        control_set->load_primitives(PRIM_FILE);
        StateLatticeParams *prim = new StateLatticeParams(start, finish, map, control_set, true, mode, 0.0, 0);
        ResultSearch res = AstarSearch(prim);

        if (res.find_path == 0)
            resfile << "Путь не найден!!!" << endl;
        else {
            resfile << "Путь найден! Стоимость: " << res.final_node->g << endl;
            resfile << "Последовательность дискретных состояний i,j,theta, начиная с конца:" << endl;
            ptrSearchNode node = res.final_node;
            while (!(node == NULL_Node)) {
                resfile << node->vertex->i << " " << node->vertex->j << " " << node->vertex->theta << endl;
                node = node->parent; 
            }
        }

        if (res.find_path == 1) HEAP->delete_SearchNode(res.final_node); 
        delete prim->ast;
        delete prim;
        delete control_set;

    } else if (mode == "TYPES") {

        TypeInfo *types_info = new TypeInfo();
        types_info->load_types(TYPE_FILE);
        TypesGraphParams *types = new TypesGraphParams(start, finish, map, types_info, true, 0.0, 0);  // в качестве погрешности R и A установим, например, 0 и 0 (путь будет точно в заданное искаться)
        ResultSearch res = AstarSearch(types);

        if (res.find_path == 0)
            resfile << "Путь не найден!!!" << endl;
        else {
            resfile << "Путь найден! Стоимость: " << res.final_node->g << endl;
            resfile << "Последовательность целевых типовых ячеек i,j,type, начиная с конца:" << endl;
            ptrSearchNode node = res.final_node;
            while (!(node == NULL_Node)) {  // так как мы специально хранили в качестве parent только целевые вершины, то просто проходимся по ним
                rassert(types_info->goal_theta_by_type[node->vertex->type] != -1, "Все ячейки, указанные в parent на пути должны быть целевыми!");
                resfile << node->vertex->i << " " << node->vertex->j << " " << node->vertex->type << endl;
                node = node->parent; 
            }
        }

        if (res.find_path == 1) HEAP->delete_SearchNode(res.final_node); 
        delete types->ast;
        delete types;
        delete types_info;

    } else {
        throw runtime_error("Некорректный mode!");
    }

    resfile.close();
    delete map;
    return;
}




//=====================================

int main() {

    /*

    // Здесь простой вариант примера работы программы:

    HEAP = new MyHEAP();  // инициализируем переменную "кучи"

    // приведём примеры, когда алгоритмы COST и TYPES находят разные траектории (из-за того, что склеивание вершин в TYPES
    // ломает гарантии оптимальности):
    vector <tuple <int, int, int>> starts = {make_tuple(130, 186, 3),
                                             make_tuple(177, 10, 10),
                                             make_tuple(177, 10, 11),
                                             make_tuple(181, 89, 10),
                                             make_tuple(141, 119, 9),
                                             make_tuple(156, 41, 2),
                                             make_tuple(143, 160, 10)};

    vector <tuple <int, int, int>> goals =  {make_tuple(133, 186, 12),
                                             make_tuple(177, 11, 2),
                                             make_tuple(177, 11, 14),
                                             make_tuple(182, 87, 11),
                                             make_tuple(137, 125, 15),
                                             make_tuple(162, 38, 14),
                                             make_tuple(141, 152, 14)};   

    for (size_t i = 0; i < starts.size(); i ++) {
        Vertex *start = new Vertex(get<0>(starts[i]), get<1>(starts[i]), get<2>(starts[i]));  // генерируем начальное и финишное состояние 
        Vertex *goal = new Vertex(get<0>(goals[i]), get<1>(goals[i]), get<2>(goals[i]));
        string res_file = "res/test" + to_string(i+1);
        make_path(start, goal, "maps/Milan_1_256.map", "data/main_control_set.txt", "data/main_types.txt", "COST", res_file+"-COST.txt");
        make_path(start, goal, "maps/Milan_1_256.map", "data/main_control_set.txt", "data/main_types.txt", "TYPES", res_file+"-TYPES.txt");//,
        delete start;
        delete goal;
    }                                 
    
    cout << HEAP->N << " " << HEAP->index_free_nodes.size() << " " << HEAP->index_free_vertexs.size() << endl; 
    delete HEAP;

    */
    
    
    // Здесь производим тестирование многими процессами:

    string pref_map = "maps/";  // где лежат карты
    string pref_prim = "data/";  // где лежат примитивы
    string pref_res = "res/";  // куда класть результаты

    vector <string> maps = {"Milan_1_256", "Moscow_0_512", "Labyrinth", "WheelofWar", "AR0700SR", "w_woundedcoast"};  // карты, на которых хотим тестировать
    vector <string> cs = {"main_control_set", "big_control_set", "short_control_set"};  // control_set  
    vector <string> types = {"main_types", "big_types", "short_types"};  // соответствующие типы  
    rassert(cs.size() == types.size(), "Для каждого control_set должны быть свои типы!");

    int iters_map = maps.size();
    int iters_prim = cs.size();
    vector <int> pids;

    for (int i = 0; i < iters_map; i ++) {  // для каждой карты и каждого набора примитивов запускаем отдельный fork (отдельная копия программы)!
        for (int j = 0; j < iters_prim; j ++) {
            int pid = fork();

            if (pid < 0) {
                cout << "Ошибка fork!" << endl;
                exit(1);
            }

            if (pid > 0) {  // pid > 0 возвращается в родительский процесс
                cout << "! Создан процесс: " << pid << ", i = " << i << endl;
                pids.push_back(pid);
                continue;
            }
            
            if (pid == 0) {  // pid = 0 возвращается в копию программы
                HEAP = new MyHEAP;  // в ней создаём глобальную переменную кучи

                string map = pref_map + maps[i] + ".map";
                string scen = pref_map + maps[i] + ".map.scen";
                string control_set = pref_prim + cs[j] + ".txt";
                string type = pref_prim + types[j] + ".txt";
                string res = pref_res + maps[i] + "_" + cs[j] + ".txt";
                //cout << control_set << " " << type << " " << map << " " << scen << " " << res << endl;
                test_algorithm(control_set, type, map, scen, res);  // запускаем тестирование

                // выводим общее количество экземпляров в куче, а также количество свободных экземпляров после конца
                // программы (если везде правильно работали с памятью и не забывали удалять неиспользуемые вершины, то
                // эти три числа должны быть равны! иначе это утечка памяти)
                cout << HEAP->N << " " << HEAP->index_free_nodes.size() << " " << HEAP->index_free_vertexs.size() << endl;
                delete HEAP;

                return 0;
            }
        }
    }

    cout << "Тестирование начато, процессы запущены..." << endl;

    int status;  // в основном процессе дожидаемся, пока все закончат работу
    for (int p: pids)
        waitpid(p, &status, 0);
    
    return 0;
}
