"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (0, 9)
        self.y_range = (0, 9)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        # self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 0.01, 9],
            [0, 9, 9, 0.01],
            [0.01, 0, 9, 0.01],
            [9, 0.01, 0.01, 9]
        ]
        return obs_boundary

    # @staticmethod
    # def obs_rectangle():
    #     obs_rectangle = [
    #     ]
    #     return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [1.14405752, 7.95232046, 0.44],
            [4.61150435, 7.6002248, 1],
            [8.23308769, 8.40551806, 0.625],
            [6.00100406, 3.53770561, 1.2],
            [2.31897332, 3.68443021, 1.6],
            [2.97253480, 4.48112465, 0.77],
            [8.55851600, 1.62836656, 0.62],
        ]

        return obs_cir