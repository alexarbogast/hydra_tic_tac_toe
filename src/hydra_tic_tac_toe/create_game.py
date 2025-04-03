import numpy as np
from enum import Enum
from pyrobopath.toolpath import Contour, Toolpath
from pyrobopath.scheduling import DependencyGraph

from .tic_tac_toe import simulate_tic_tac_toe


class Materials(Enum):
    GAME_BOARD = 0
    X = 1
    O = 2


def create_game_board(w, z):
    paths = []
    paths.append([[-1 / 6 * w, 1 / 2 * w, z], [-1 / 6 * w, -1 / 2 * w, z]])
    paths.append([[1 / 6 * w, 1 / 2 * w, z], [1 / 6 * w, -1 / 2 * w, z]])
    paths.append([[-1 / 2 * w, 1 / 6 * w, z], [1 / 2 * w, 1 / 6 * w, z]])
    paths.append([[-1 / 2 * w, -1 / 6 * w, z], [1 / 2 * w, -1 / 6 * w, z]])

    toolpath = Toolpath()
    toolpath.contours = [
        Contour(np.array(p), Materials.GAME_BOARD.value) for p in paths
    ]
    return toolpath


def create_X(w, offset):
    paths = [
        [[-1 / 2 * w, 1 / 2 * w, 0], [1 / 2 * w, -1 / 2 * w, 0]],
        [[1 / 2 * w, 1 / 2 * w, 0], [-1 / 2 * w, -1 / 2 * w, 0]],
    ]
    path = np.array(paths) + np.array(offset)
    contours = [Contour(p, Materials.X.value) for p in path]
    return contours


def create_O(w, offset):
    N_POINTS = 50
    path = [
        [w / 2 * np.cos(t), w / 2 * np.sin(t), 0]
        for t in np.linspace(0, 2 * np.pi, N_POINTS)
    ]
    path = np.array(path) + np.array(offset)
    return Contour(path, Materials.O.value)


def create_tic_tac_toe(w, z):
    w13 = 1 / 3 * w
    offsets = [
        [-w13, w13, z],
        [0, w13, z],
        [w13, w13, z],
        [-w13, 0, z],
        [0, 0, z],
        [w13, 0, z],
        [-w13, -w13, z],
        [0, -w13, z],
        [w13, -w13, z],
    ]
    offsets = np.array(offsets)

    # simluate game and create toolpaths
    game = simulate_tic_tac_toe()
    game = [3 * i + j for i, j in game]

    symbol_w = w13 * 0.6
    contours = []
    for i in range(len(game)):
        place = game[i]
        if i % 2 == 0:  # X
            contours.extend(create_X(symbol_w, offsets[place]))
        else:  # O
            contours.append(create_O(symbol_w, offsets[place]))

    toolpath = Toolpath()
    toolpath.contours = contours

    # create dependency graph
    dg = DependencyGraph()
    dg.add_node("start")
    dg.set_complete("start")
    dg.add_node(0, ["start"])

    for i in range(1, len(toolpath.contours)):
        dg.add_node(i, [i - 1])

    return toolpath, dg
