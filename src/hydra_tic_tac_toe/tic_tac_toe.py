import math
import random


def check_winner(board):
    b = board
    # Check rows, columns
    for i in range(3):
        if b[i][0] == b[i][1] == b[i][2] and b[i][0] != " ":
            return b[i][0]
        if b[0][i] == b[1][i] == b[2][i] and b[0][i] != " ":
            return b[0][i]

    # Check diagonals
    if b[0][0] == b[1][1] == b[2][2] and b[0][0] != " ":
        return b[0][0]
    if b[0][2] == b[1][1] == b[2][0] and b[0][2] != " ":
        return b[0][2]

    return None


def is_moves_left(board):
    return any(" " in row for row in board)


def minimax(board, depth, is_maximizing):
    winner = check_winner(board)
    if winner == "O":
        return 10 - depth
    elif winner == "X":
        return depth - 10
    elif not is_moves_left(board):
        return 0

    if is_maximizing:
        best = -math.inf
        for i in range(3):
            for j in range(3):
                if board[i][j] == " ":
                    board[i][j] = "O"
                    best = max(best, minimax(board, depth + 1, False))
                    board[i][j] = " "
        return best
    else:
        best = math.inf
        for i in range(3):
            for j in range(3):
                if board[i][j] == " ":
                    board[i][j] = "X"
                    best = min(best, minimax(board, depth + 1, True))
                    board[i][j] = " "
        return best


def find_best_move(board):
    best_val = -math.inf
    best_move = (-1, -1)

    for i in range(3):
        for j in range(3):
            if board[i][j] == " ":
                board[i][j] = "O"
                move_val = minimax(board, 0, False)
                board[i][j] = " "
                if move_val > best_val:
                    best_val = move_val
                    best_move = (i, j)

    return best_move


def simulate_tic_tac_toe():
    board = [[" " for _ in range(3)] for _ in range(3)]
    history = []

    # random first move to keep things interesting
    move = random.randint(0, 8)
    board[move // 3][move % 3] = "X"
    history.append((move // 3, move % 3))

    while is_moves_left(board):
        row, col = find_best_move(board)
        board[row][col] = "O"
        history.append((row, col))

        if check_winner(board):
            break

        if is_moves_left(board):
            row, col = find_best_move(board)
            board[row][col] = "X"
            history.append((row, col))

        if check_winner(board):
            break

    return history
