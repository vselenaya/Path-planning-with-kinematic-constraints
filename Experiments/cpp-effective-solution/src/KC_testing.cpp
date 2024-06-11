#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <random>  // для random_device и другого вероятностного
#include <fstream>  // для ifstream
#include <sstream>  // для stringstream

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

//=====================================

int main() {
    /*
    Самый простой способ запустить тестирование:
    HEAP = new MyHEAP();  // инициализируем переменную "кучи"
    test_algorithm("data/main_control_set.txt", "data/main_types.txt",
                   "maps/Milan_1_256.map", "maps/Milan_1_256.map.scen",
                   "res.txt");//,
    cout << HEAP->N << " " << HEAP->index_free_nodes.size() << " " << HEAP->index_free_vertexs.size() << endl; 
    delete HEAP;
    */

    string pref_map = "maps/";  // где лежат карты
    string pref_prim = "data/";  // где лежат примитивы
    string pref_res = "res/";  // куда класть результаты

    vector <string> maps = {"Milan_1_256", "Moscow_0_512", "Labyrinth"};  // карты, на которых хотим тестировать
    vector <string> cs = {"main_control_set"};  // control_set
    vector <string> types = {"main_types"};  // соответствующие типы
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
