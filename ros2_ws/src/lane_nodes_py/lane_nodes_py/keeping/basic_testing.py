from typing import List, Tuple

from ros2_ws.src.lane_nodes_py.lane_nodes_py.keeping.keeping import Keeping

from ischedule import schedule, run_loop

import matplotlib.pyplot as plt

def main():
    keeping = Keeping(1, 1, 1)
    path: List[Tuple[float, float]] = perabola()
    keeping.path_grid.set_path(path)
    schedule(keeping.movement_output_callback, interval=0.1)
    run_loop(return_after=10)
    print("Vehicle History: " + str(keeping.path_grid.history))
    x1, y1 = zip(*path)
    x2, y2 = zip(*keeping.path_grid.history)
    plt.plot(x1, y1, c='#4CAF50', label="Desired Path")
    plt.plot(x2, y2, color='r', ls=':', label="Vehicle Path")
    plt.ylabel('y')
    plt.xlabel('x')
    plt.legend(loc='upper right')
    plt.show()

def sinusodal_curve():
    path: List[Tuple[float, float]] = []
    for i in range(-1, 110):
        path += [(i / 10, -np.cos(0.62831853071 * (i / 10)) + 1)]
    return path


def perabola():
    path: List[Tuple[float, float]] = []
    for i in range(-1, 110):
        path += [(i / 10, (-1/2)*((i/10) - 5)**2 + 5)]
    return path


if __name__ == '__main__':
    main()
