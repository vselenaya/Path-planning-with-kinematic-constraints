#include <cmath>  // для функции sqrtf64x квадратного корня
#include <fstream>
#include <sstream>
#include <map>

#include "KC_structs.hpp"
#include "common.hpp"
#include "rassert.hpp"

extern MyHEAP* HEAP;



Vertex::Vertex(int i, int j, int theta) {
    /*
    Данная функция будет конструктором класса Vertex, когда он используется
    в качестве дискретного состояния с координатами i,j и направлением theta.
    */

    this->i = i;
    this->j = j;
    this->theta = theta;
    type = -1;  // у дискретного состояния тип всегда -1
}


Vertex::Vertex(int i, int j, int type, int info) {
    /*
    Данная функция будет конструктором класса Vertex, когда он используется
    в качестве типовой ячейки с координатами i,j, типом type, а также
    с информацией info, которая будет использована для склеивания вершин.
    */

    this->i = i;
    this->j = j;
    this->type = type;
    this->info = info;
}




Primitive::Primitive() {
    /*
    Конструктор: при инициализации примитива просто резервируем в памяти вершину Vertex
    (которая будет использоваться в виде дискретного состояния).
    */
    goal = HEAP->new_Vertex(0, 0, 0);
}


void Primitive::add_collision(int i, int j) {
    /*
    Функция для добавления клетки (i,j) в список коллизий.
    */

    collision_in_i.push_back(i);
    collision_in_j.push_back(j);
}


void Primitive::calc_collision() {  
    /*
    Функция считает стоимость коллизионного следа примитива.
    Заметим, что тут важно, чтобы клетки (i,j) в collision_in_i и collision_in_j
    были перечислены в порядке прохождения по ним примитива.
    */

    collision_cost = 0;
    for (size_t k = 0; k + 1 < collision_in_i.size(); k ++) {  // перебираем номер клетки (= степень в примитиве)
        int di = collision_in_i[k+1] - collision_in_i[k];  // считаем шаговый сдвиг из k-ой клетки в k+1-ую
        int dj = collision_in_j[k+1] - collision_in_j[k];
        collision_cost += sqrtf64x(di * di + dj * dj);  // добавляем стоимость перехода (1 по стороне, sqrt(2) по углу)
    }
}


Primitive::~Primitive() {
    /*
    Деструктор. Его задача высвободить всю память, которая была выделена под примитив. Все
    векторы (std::vector) освободят свою память автоматически, поэтому единственное, что
    нужно освободить - это выделенную вершину, использовавшуюся в качестве целевого дискретного состояния.
    */

    HEAP->delete_Vertex(goal);
}




ControlSet::ControlSet() {
    /*
    Конструктор.
    */

    control_set.assign(ANGLE_NUM, vector <Primitive *>());  // инициализируем список примитивов для каждого дискретного угла
}


static void check_theta(int theta) {
    /*
    Данная функция проверяет, что номер дискретного направления theta корректен с точки зрения указанного
    значения в common.hpp
    */

    (void) theta;
    rassert(0 <= theta && theta < ANGLE_NUM, "Угол направления (его номер) должен быть от 0 до ANGLE_NUM-1!");
}


void ControlSet::load_primitives(string _file) {
    /*
    Данная функция загружает из файла _file примитивы (файл имеет тот формат, что генерируется
    кодом на Питоне), заполняя все структуры. 
    */

    ifstream file(_file);  // файл с примитивами открываем в виде потока
    string line;  // очередная строка файла

    Primitive* prim;  // сюда записываем примитив
    int theta;  // угол, из которого примитив выходит
    string temp;  // временная строка для считывания информации

    // основной цикл для считывания примитивов - в нем на каждой итерации читаем очередную строку line 
    // из file; цикл прервётся, когда в file кончатся непрочитанные строки (while получит 0):
    while (getline(file, line)) {
        stringstream stream(line);  // превращаем строку в поток для считывания из неё через ">>"

        if  (line.find("===== prim description: =====") == 0) {
            prim = new Primitive();  // создаём новый примитив (делаем это через стандартную "кучу" из new, так как загрузка примитивов делается заранее и не тормозит алгоритм)
            continue;
        }

        if (line.find("start heading (number):") == 0) {
            stream >> temp >> temp >> temp >> theta;  // каждое ">>" считывает до очередного пробела -> сначала три раза
                                                      // считываем какие-то общие слова в неиспользуемую переменную temp, а далее
                                                      // считываем число theta (так уж устроен формат файла, что сначала в строке три куска
                                                      // текста (разделённые пробелом), а затем идёт номер дискретного угла)
            check_theta(theta);
        }

        if (line.find("goal state (i, j, heading num):") == 0) {
            stream >> temp >> temp >> temp >> temp >> temp >> temp >> prim->goal->i >> prim->goal->j >> prim->goal->theta;  // благодаря определённому в ptrVertex оператору "->" можем считывать поля goal, обращаясь к ним через "->"
            check_theta(prim->goal->theta);
        }

        if (line.find("length is:") == 0)
            stream >> temp >> temp >> prim->length;

        if (line.find("turning on:") == 0)
            stream >> temp >> temp >> prim->turning;

        if (line.find("trajectory is:") == 0) {
            while (1) {  // в этом цикле просто в никуда читаем координаты точек примитива (они были нужны только для отрисовки, в коде C++ они не нужны)
                getline(file, line);
                if (line.find("---") == 0)
                    break;
            }
        }

        if (line.find("collision is:") == 0) {
            while (1) {
                getline(file, line);
                if (line.find("---") == 0)
                    break;
                stringstream stream(line);
                int i, j;
                stream >> i >> j;
                prim->add_collision(i, j);
            }
        }
                
        if (line.find("prim end") == 0) {  // после окончания прочтения примитива, сохраняем информацию о нём:
            prim->start_theta = theta;
            prim->calc_collision();  // подсчитываем стоимость коллизионного следа
            control_set[theta].push_back(prim);  // добавляем примитив в control_set
        }
    }
    
    file.close();

    cout << "Примитивы загружены..." << endl;
}


vector <Primitive*> & ControlSet::get_prims_by_heading(int heading) {
    /*
    Данная функция должна вернуть список примитивов, начинающихся в угле heading.
    Чтобы не тратить время на копирования, функция возвращает просто ссылку (&) на
    этот список. 
    */

    return control_set[heading];
}


ControlSet::~ControlSet() {
    /*
    Деструктор. Высвобождает память.
    */

    for (auto prims_list: control_set) 
        for (Primitive *prim: prims_list)
            delete prim;  // просто удаляем каждый примитив.
}




TypeInfo::TypeInfo() {
    /*
    Конструктор. Инициализируем все списки исходя из максимально возможных количество типов и углов (которые
    указаны в common.hpp)
    */
    
    successors.assign(MAX_TYPES, vector <tuple <int, int, int>>()); 
    start_type_by_theta.assign(ANGLE_NUM, -1); 
    is_goal_by_theta_type.assign(ANGLE_NUM, vector <bool> (MAX_TYPES, 0));  // изначально заполняем все нулями (пока нет целевых ячеек)
    add_info_by_type.assign(MAX_TYPES, 0); 
    goal_theta_by_type.assign(MAX_TYPES, -1);  // пока что ни одна ячейка не целевая - только -1 ставим
}


static void check_type_limit(int type) {
    /*
    Данная функция должна проверить, что тип type укладывается в заданное ограничение.
    */
   
    (void) type;
    rassert(0 <= type && type < MAX_TYPES, "В файле слишком много типов! Увеличьте значение MAX_TYPES в файле common.hpp! Также все типы >= 0!");
}


static size_t get_next_info(string &s) {
    /*
    Данная функция по строке s с информацией для склеивания вершин (эта строка в файле с типами берётся)
    возвращает информацию в виде size_t-переменной. Для этого эта функция просто нумерует все встречаемые
    строки, а номер возвращает в качестве результата.
    */

    static size_t free_info = 0;  // переменная, обозначающая текущий свободный номер (так как переменная static, 
                                  // она заводится только один раз - при первом запуске функции (и инициализируется 0),
                                  // а дальше между запусками функции её значение сохраняется)
    static map <string, size_t> all_info;  // словарь, который строке сопоставляет приписанный ей номер (словарь тоже static,
                                           // поэтому при новом запуске функции он помнит своё значение с предыдущих запусков)

    if (all_info.count(s) == 0) {  // если поданная на вход строка ещё не пронумерована (нет в словаре)
        rassert(free_info < MAX_INFO, "Слишком много различных info-значений! Измените лимит MAX_INFO! в файле common.hpp!");
        all_info[s] = free_info;  // нумеруем её
        free_info += 1;  // сдвигаем число для номера следующих строк
    }

    return all_info[s];  // возвращаем номе строки
}


void TypeInfo::load_types(string _file) {
    /*
    Функция, которая загружает всю необходимую информацию из файла file с типами (этот файл
    генерируется кодом на Питоне после нумерации конфигураций).
    */

    ifstream file(_file);
    string line;  // строка файла с типами
    string temp;  // временная строка для считывания информации

    // основной цикл для считывания, на каждой итерации считывается по строке файла:
    while (getline(file, line)) {
        stringstream stream(line);  // превращаем строку в поток для считывания из неё

        if  (line.find("control-set-start with theta:") == 0) {
            int theta, type;
            stream >> temp >> temp >> temp >> theta >> temp >> temp >> type;
            check_theta(theta);
            check_type_limit(type);
            start_type_by_theta[theta] = type;
            continue;
        }

        if (line.find("in goal type:") == 0) {
            int theta, type;
            stream >> temp >> temp >> temp >> type >> temp >> temp >> temp >> temp >> temp;
            check_type_limit(type);
            
            int cnt = 0;
            while (stream >> theta) {  // пока можно, считываем углы theta, в которых заканчиваются примитивы в ячейке с типом type
                cnt += 1;
                check_theta(theta);
                is_goal_by_theta_type[theta][type] = 1;  // указываем, что в ячейке типа type заканчивается примитив в угле theta
                goal_theta_by_type[type] = theta;
                if (cnt >= 2)
                    goal_theta_by_type[type] = -2;  // если несколько финальных углов, то указываем -2
            }

            continue;
        }

        if (line.find("start type is:") == 0) {
            int type;
            stream >> temp >> temp >> temp >> type;
            check_type_limit(type);

            while (1) {
                getline(file, line);
                stringstream stream(line);

                if (line.find("---") == 0)
                    break;

                int di, dj, t;
                stream >> di >> dj >> t;
                successors[type].push_back(make_tuple(di, dj, t));  // запоминаем соседей у типа
            }

            continue;
        }

        if (line.find("add_info for type:") == 0) {
            int type;
            stream >> temp >> temp >> temp >> type >> temp;
            check_type_limit(type);

            string str_info;
            getline(stream, str_info);  // сохраняем в строку всю информацию для склеивания для типа type из файла
            add_info_by_type[type] = get_next_info(str_info);  // получаем номер этой информации

            continue;
        }
    }
    
    file.close();

    cout << "Типы загружены..." << endl;
}
