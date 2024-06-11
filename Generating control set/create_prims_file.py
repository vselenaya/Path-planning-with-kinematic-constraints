"""
В данном файле перечислены функции для сохранения сгенерированных примитивов в файл.
Затем из полученного файла в любой программе можно загрузить эти примитивы (например,
с помощью функции load_primitives в классе ControlSet) и использовать для поиска.
"""



from typing import List, Tuple
import copy  # copy.deepcopy позволяет копировать экземпляр класса целиком, вместе со всеми внутренними структурами

import sys
sys.path.append("../common/")
from KC_structs import *
from KC_graphics import *



def get_prim_collision(traj: ShortTrajectory) -> List[Tuple[int, int]]:
    """
    Функция возвращает список пар вида (i,j) координат клеток, по которым проходит траектория. То есть это 
    коллизионный след траектории. Координаты (i,j) клеток перечислены в порядке прохода по ним траектории 
    (то есть в порядке увеличения их степени внутри неё).

        traj: траектория, экземпляр ShortTrajectory.
    """
    
    ij_coord = {}  # словарь: по паре координат (i,j) хранит её же саму - это нужно просто для запоминания, что данная пара уже есть
                   # (используем именно словарь dict() (а не set), так как он сохраняет порядок вставки элементов!)
    
    for x, y in zip(traj.sample_x(ds=0.01), traj.sample_y(0.01)):  # семплируем точки с траектории с очень маленьким шагом (чтобы не пропустить ни одну клетку)
        i = int(round(y, 0))  # округляем числа до целого -> получаем координаты клетки (i,j), в которую попадает точка (x,y) траектории
        j = int(round(x, 0))
        ij_coord[(i, j)] = (i, j)  # добавляем в коллизионный след эту клетку (если она уже встречалась, ничего не произойдёт)
        
    return ij_coord.values()  # возвращаем список клеток в порядке появления 



def save_primitive(file: str, prim: ShortTrajectory, theta_discrete: Theta) -> None: 
    """
    Функция, которая сохраняет примитив в файл. Таким оразом, сохраняется каждый элемент control set.

        file: файл, куда сохранять,
        prim: сгенерированная траектория, примитив - образец класса ShortTrajectory,
        theta_discrete: используемая дискретизация угла.

    Замечание: при вызове этой функции примитив уже должен быть сгенерированным, то есть его параметры подобраны
    и prim.final_state() должен быть равен prim.goal.
    """
    
    with open(file, "a") as f:  # !!! файл открываем на дописывание (модификатор 'a' = append), чтобы не затереть предыдущие записи
        f.write(f"===== prim description: =====\n")  # запись примитива начинается с этой строки

        # сначала пишем начальное дискретное направление, в котором выходит примитив:
        # (координаты не пишем, так как примитивы control set начинаются всегда в координатах (0,0))
        f.write(f"start heading (number): {theta_discrete.num_angle(prim.start.theta)}\n") \
        
        # теперь указываем дискретное состояние, в которое идёт примитив:
        # (так как состояние дискретное, то переходим к координатам вида (i,j) - номер строки и столбца - для
        # этого просто меняем местами x и y; в данном случае i и j - это просто целочисленные значения y и x соответственно)
        f.write(f"goal state (i, j, heading num): {int(prim.goal.y)} {int(prim.goal.x)} {theta_discrete.num_angle(prim.goal.theta)}\n")

        f.write(f"length is: {prim.length}\n")  # длина примитива как траектории
        f.write(f"turning on: {theta_discrete.dist(prim.start.theta, prim.goal.theta)}\n")  # на сколько дискретных направлений поворачивает агент при движении по примитиву
        
        # здесь описываем траекторию примитива (снова в (x,y) координатах, так как это описание именно кривой на плоскости, как её и генерировали)
        f.write(f"trajectory is:\n")  
        for x, y in zip(prim.sample_x(ds=0.1), prim.sample_y(ds=0.1)):
            f.write(f"{x} {y}\n")  # просто сохраняем точки (x,y), семплированные с примитива с некоторым шагом (например, шаг 0.1 - не очень маленький, чтобы точек было не очень много и файл не стал большим)
        f.write(f"---\n")                # (это точки нужны, чтобы нарисовать график примитива (кривую на плоскости) и посмотреть, как он выглядит)

        # здесь описываем коллизионный след примитива, клетки перечисляются в том порядке, в котором по ним проходит примитив
        f.write(f"collision is:\n")
        for i, j in get_prim_collision(prim):
            f.write(f"{i} {j}\n")
        f.write(f"---\n")

        f.write("prim end\n")  # запись примитива окончена
        
        
      
def save_and_show(file: str, prim: ShortTrajectory, theta_discrete: Theta, 
                  central_sym: bool = True, show: bool = True, ax: Optional[matplotlib.axes.Axes] = None):
    """
    Функция для сохранения примитива в файл и его отрисовки, а также для возможности сохранять сразу набор примитивов,
    отличающихся симметрией.

        file: куда сохранять примитив,
        prim: сам примитив, траектория класса ShortTrajectory,
        theta_discrete: используемая дискретизация,
        central_sym: нужно ли помимо самого prim сохранять так же ещё 3 его версии, отличающихся центральной симметрией (повороты на +\pi/2),
        show: нужно ли отрисовывать сохраняемые примитивы,
        ax: задаёт matplotlib.axes, где рисовать траекторию.
        
    Замечание: если show и central_sym равны True, то будет нарисовано 4 примитива (исходной = 3 копии поворотом) разными цветами: красный, зелёный, синий, желтый
    """
    
    x, y, theta, k = prim.goal.x, prim.goal.y, prim.goal.theta, prim.goal.k  # сохраняем финальное состояние, куда идёт примитив
    current_prim = copy.deepcopy(prim)  # копируем текущий примитив, чтобы его не портить и работать вместо этого с копией current_prim
    
    save_primitive(file, current_prim, theta_discrete)  # сохраняем сам примитив
    if show: show_trajectory(current_prim, col="r", ax=ax)  # рисуем исходный примитив красным
        
    if central_sym:  # если нужно, то еще центральной симметрией по кругу (по +pi/2 радиан) сохраняем остальные три версии примитивы
        current_prim.start.theta += np.pi/2
        current_prim.goal = State(-y, x, theta+np.pi/2, k)
        save_primitive(file, current_prim, theta_discrete)
        if show: show_trajectory(current_prim, col="g", ax=ax)
            
        current_prim.start.theta += np.pi/2  # поворачиваем примитив на +pi/2 (заметим, что для поворота достаточно обновить стартовый угол и целевое состояние, так как
        current_prim.goal = State(-x, -y, theta+2*np.pi/2, k)  # параметры примитива не поменяются: кривизна и длина не меняются от поворота -> после изменения угла и цели,
        save_primitive(file, current_prim, theta_discrete)     # функции sample_x и sample_y будут давать точки уже повёрнутого примитива)
        if show: show_trajectory(current_prim, col="b", ax=ax)
            
        current_prim.start.theta += np.pi/2
        current_prim.goal = State(y, -x, theta+3*np.pi/2, k)
        save_primitive(file, current_prim, theta_discrete)
        if show: show_trajectory(current_prim, col="y", ax=ax)
