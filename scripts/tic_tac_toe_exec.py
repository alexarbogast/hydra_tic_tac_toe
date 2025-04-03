import rospy

from pyrobopath_ros import ScheduleExecution
from pyrobopath.toolpath_scheduling import animate_multi_agent_toolpath_full

from hydra_tic_tac_toe import create_game_board, create_tic_tac_toe

NAME = "tic_tac_toe_demo"
LIMITS = ((-1.0, 1.0), (-1.0, 1.0))


class TicTacToeExec:
    def __init__(self):
        self.sched_exec = ScheduleExecution()
        self.board_width = 0.35
        self.path_height = 0.0

    def run(self):
        self.sched_exec.move_home()
        self.draw_game_board()
        self.play_tic_tac_toe()
        rospy.spin()

    def draw_game_board(self):
        toolpath = create_game_board(self.board_width, self.path_height)

        # schedule toolpath
        self.sched_exec.schedule_toolpath(toolpath)

        # animate_multi_agent_toolpath_full(
        #     toolpath,
        #     self.sched_exec._schedule,
        #     self.sched_exec.agent_models,
        #     limits=LIMITS,
        # )

        self.sched_exec.execute_schedule()

    def play_tic_tac_toe(self):
        toolpath, dg = create_tic_tac_toe(self.board_width, self.path_height)

        # schedule toolpath
        self.sched_exec.schedule_toolpath(toolpath, dg)

        # animate_multi_agent_toolpath_full(
        #     toolpath,
        #     self.sched_exec._schedule,
        #     self.sched_exec.agent_models,
        #     limits=LIMITS,
        # )

        self.sched_exec.execute_schedule()


if __name__ == "__main__":
    rospy.init_node(NAME)
    try:
        demo = TicTacToeExec()
        demo.run()
    except rospy.ROSInterruptException:
        pass
