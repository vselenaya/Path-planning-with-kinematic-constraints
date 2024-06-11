"""
В данном файле перечислены основные функции для отрисовки результатов поиска кинематически-согласованной траектории:
    рисование сетки (grid), рисование траекторий, онлайн-анимация траектории в процессе подбора параметров,
    рисование карты (дискретного рабочего пространства) с препятствиями.
"""




import matplotlib.axes
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from IPython.display import clear_output
from KC_structs import *
from KC_searching import *




def draw_grid(ax: matplotlib.axes.Axes, xs: int = -8, ys: int = -8, xf: int = 8, yf: int = 8, tick_step: int = 1) -> None:
    """ 
    Функция для рисования сетки (grid) и нанесения отметок на координатные оси.
    
        от (xs, ys) до (xf, yf): координаты прямоугольника, внутри которого будет находиться график с сеткой,
        ax: задаёт matplotlib.axes, в которых рисовать сетку,
        tick_step: шаг, с которым делать отметки на координатных осях.
        
    Замечание: сетка рисуется так, чтобы целые координаты указывали в центр клеток (то есть линии сетки будут в полуцелвых ккордианах)
    """
    
    plt.axis("equal")  # масштаб по осям графика одинаковый
    
    ax.set(xlim=(xs, xf), xticks=np.arange(xs, xf, tick_step),
           ylim=(ys, yf), yticks=np.arange(ys, yf, tick_step))
    
    for y in np.arange(ys+0.5, yf, 1):
        ax.axhline(y, color='grey', alpha=0.45)  # горизонтальные линии сетки (линии рисуются с коэффициентом прозрачности 0.45)
        
    for x in np.arange(xs-0.5, xf, 1):
        ax.axvline(x, color='grey', alpha=0.45)  # вертикальные линии
        



def plot_arrow(x: float, y: float, theta: float, 
               length: float = 1.0, width: float = 0.5, 
               fc: str = "r", ec: str = "k", 
               ax: Optional[matplotlib.axes.Axes] = None) -> None:
    """
    Функция для рисования стрелки в определённом направлении.
    
        x, y, theta: координаты начала и угол направления стрелки
        ax: задаёт matplotlib.axes, где рисовать стрелку (если None, то просто в plt рисуется)
        line, width: размеры стрелки
        fc, ec: цвет стрелки
    """
    
    board = plt if (ax is None) else ax  # определяем, где рисовать стрелку
    board.arrow(x, y, length * np.cos(theta), length * np.sin(theta),
                fc=fc, ec=ec, head_width=width, head_length=width)




def show_trajectory(traj: ShortTrajectory, col: str = 'r', arrow: bool = True, ax: Optional[matplotlib.axes.Axes] = None) -> None:
    """
    Функция для рисования траектории.
    
        traj: траектория в виде объекта ShortTrajectory,
        col: цвет кривой,
        arrow: рисовать ли стрелку, изображающую конец траектории (её координаты и угол соответствуют финальному состоянию траектории),
        ax: задаёт matplotlib.axes, где рисовать траекторию.
    """
    
    board = plt if (ax is None) else ax  # определяем, где рисовать
    
    xc = traj.sample_x()  # получаем набор точек кривой, изобразив которые, получим вид траектории
    yc = traj.sample_y()
    board.plot(xc, yc, "-"+col)
    
    if arrow:
        final = traj.goal  # стрелка рисуется в целевом состоянии (куда должна вести траектория при правильных параметрах)
        plot_arrow(final.x, final.y, final.theta, fc=col, ax=ax)
        
        
        

def redraw_trajectory(prim: ShortTrajectory, col: str = 'r') -> None:
    """
    Функция, перерисовывающая траекторию (полезна для демонстрации изменений траектории
    в процессе подбора параметров).
    
        traj: текущая траектория как объект ShortTrajectory,
        col: цвет траектории.
    """
    
    xc = prim.sample_x()  # точки траектории 
    yc = prim.sample_y()      
    x, y, theta = prim.goal.x, prim.goal.y, prim.goal.theta  # финальные координаты и угол направления (к которым траектория стремится)
    
    clear_output(wait=True)  # очищаем предыдущий вывод
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.axis("equal")
    ax.grid(True)
    
    ax.plot(xc, yc, "-"+col)  # рисуем текущую траекторию
    ax.arrow(x, y, 1 * np.cos(theta), 1 * np.sin(theta),
             fc=col, ec="k", head_width=0.5, head_length=0.5)
    
    plt.pause(0.01)
    fig.canvas.draw()
    plt.show()




def draw_primitive(prim: Primitive, col: str = 'r', arrow: bool = True, 
                   ax: Optional[matplotlib.axes.Axes] = None, theta: Optional[Theta] = None) -> None:
    """
    Рисование конкретного (уже сгенерированного) примитива.
    
        prim: примитив, экземпляр класса Primitive,
        col: цвет кривой,
        arrow: рисовать ли стрелку, изображающую конец примитивы (её координаты и угол соответствуют финальному состоянию),
        ax: задаёт matplotlib.axes, где рисовать траекторию,
        theta: дискретизация угла, какую использовать - аналогично классу Theta должно быть написано.
    """
    
    board = plt if (ax is None) else ax  # определяем, где рисовать
    xc = prim.x_coords
    yc = prim.y_coords
    plt.axis("equal")
    
    board.plot(xc, yc, "-"+col)
    if arrow and theta is not None:
        final = prim.goal
        plot_arrow(final.j, final.i, theta[final.theta], fc=col, ax=ax)
        
        
        
    
def draw_task_map(task_map: Map, start: DiscreteState, goal: DiscreteState, 
                  dpi: int = 200, scale: float = 1.0, A: int = 1, R: float = 3, theta: Optional[Theta] = None) -> matplotlib.axes.Axes:
    """
    Функция, которая рисует карту (дискретное рабочее пространство), а также старт и цель на нём.
    
        task_map: карта, экземпляр класса Map,
        start, goal: начальное и финишное состояние (именно состояние DiscreteState, так как траектория мобильного агента (являясь
                функцией изменения состояния) всегда задаётся начальным и конечным состояниями);  даже в случае поиска на графе типов,
                всё равно сначала задаются дискретные состояния, по которым затем считаются начальная и целевые ячейки)
        dpi: разрешение картинки (dots per inch = количество пикселей на дюйм)
        scale: масштаб - во сколько раз больше/меньше рисовать картинку
        A,R: настройки целевых вершин (в каком диапазоне от финишной вершины goal находятся целевые вершины, до которых ищется путь)
        theta: дискретизация угла, какую использовать
    """
    
    w, h = 10 * (task_map.width / max(task_map.width, task_map.height)), 10 * (task_map.height / max(task_map.width, task_map.height))  # получаем ширину/высоту картинки в пропорциях карты
        
    fig = plt.figure(figsize=(scale*w, scale*h), dpi=dpi)  # создаём фигуру (это как бы холст, на котором рисуем)
    ax = fig.add_subplot(111)  # добавляем оси для рисования КОНКРЕТНОГО графика - 111 означает, что добавляем единственные (то есть
                               # на всем fig будет только один график) оси, расположение которых 1,1 по вертикали/горизонтали


    # ===== рисуем карту =====
    xs, ys = 0, 0
    xf, yf = task_map.width, task_map.height
    _d = int((xf - xs) / 30) + 1  # шаг, с которым делаем насечки на осях  
    
    ax.set(xlim=(xs, xf), xticks=np.arange(0, xf, _d),        # устанавливаем лимит по каждой из осей, а также насечки на них (насечки только в целых точках, так как нам интересны только целочисленные x и y, которые играют роль i,j)
           ylim=(ys, yf)[::-1], yticks=np.arange(0, yf, _d))  # (так как на карте i-ось (номер строки с клетками) растёт вниз, то для этого просто ось y направляем вниз! (на картинке в качестве i и j будут y и x соответственно) -> для этого [::-1])
                                                              # (Заметим: до этого мы рисовали ось i как обычно, так как было не важно, но тут всё-таки ось вниз)
    ax.axis('equal')  # масштаб по осям одинаковый                                      


    # рисуем сетку, которая будет обозначать клетки дискретного рабочего пространства; координаты (i,j) клетки будут написаны на осях 
    # (y ось будет в качестве i (для этого её переворачивали), x ось в качестве x (с ней ничего не делали, так как x и j обе вправо))
    for y in np.arange(ys-0.5, yf, 1):
        ax.axhline(y, color='grey', alpha=0.45)  # горизонтальные линии
    for x in np.arange(xs-0.5, xf, 1):
        ax.axvline(x, color='grey', alpha=0.45)  # вертикальные линии


    # ===== рисуем препятствия =====
    for i in range(task_map.height):
        for j in range(task_map.width):
            if not task_map.traversable(i, j) == 1:
                fig.gca().add_patch(plt.Rectangle((j-0.5, i-0.5), 1, 1, color='g', alpha=0.25))  # j - аналог x-координаты, i - аналог y-координаты (если ее ось вниз)
                # замечание: fig.gca() получает текущие оси: то есть в данном случае fig.gca() и ax  - одно и то же! (просто используем fig.gca() для разнообразия)


    # ===== рисуем цель и старт=====
    circle = plt.Circle((goal.j, goal.i), R)  # круг радиуса R, который показывает допустимое отклонение координат целевых вершин от финишного состояния
    fig.gca().add_patch(circle)
    plot_arrow(goal.j, goal.i, theta[goal.theta], fc='red')  # направление (от него +-A может отличаться направление целевых вершин) финишного состояния
    
    fig.gca().add_patch(plt.Rectangle((start.j-0.5, start.i-0.5), 1, 1, color='g', alpha=1))
    plot_arrow(start.j, start.i, theta[start.theta], fc='green')  # начальное направление
    
    
    return ax  # возвращаем оси, в которых всё это рисовалось
