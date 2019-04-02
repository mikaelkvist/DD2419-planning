import math
import numpy as np

class SimpleMotion:
    def __init__(self):
        self.path_pos = None

    def set_raise(self, current_pose, height):
        self.path_pos = [(current_pose.position.x, \
                          current_pose.position.y, \
                          0.4, \
                          current_pose.position.w)]

    def set_lower(self, current_pose, height):
        self.path_pos = []
        nr_steps = int(height/0.1)
        steps = [i for i in range(nr_steps)]
        for i in steps:
            self.path_pos.append((current_pose.position.x, \
                                  current_pose.position.y, \
                                  i*0.1, \
                                  current_pose.position.w))

    def set_turn(self, current_pose):
        self.path_pos = []
        nr_steps = 4
        angles = np.linspace(0, 2*math.pi, nr_steps+1)[1:]
        for i in angles[::-1]:
            self.path_pos.append((current_pose.position.x, \
                                  current_pose.position.y, \
                                  current_pose.position.z, \
                                  current_pose.position.w + i*180/math.pi))

    # def set_turn_step(self, current_pose, new_angle):
    #     self.path_pos = []
    #     # print(current_pose, new_angle)
    #
    #     aim = ((new_angle + math.pi) % (2*math.pi)) - math.pi
    #     now = ((current_pose[3] + math.pi) % (2*math.pi)) - math.pi
    #
    #     if 0 < now < math.pi/2:
    #         if math.pi/2 < aim < math.pi:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   math.pi/2*180/math.pi))
    #         elif -math.pi/2 < aim < 0:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   0*180/math.pi))
    #         elif -math.pi < aim < -math.pi/2:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   -math.pi/2*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   0*180/math.pi))
    #     elif math.pi/2 < now < math.pi:
    #         if 0 < aim < math.pi/2:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   math.pi/2*180/math.pi))
    #         elif -math.pi/2 < aim < 0:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   0*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   math.pi/2*180/math.pi))
    #         elif -math.pi < aim < -math.pi/2:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   math.pi*180/math.pi))
    #     elif -math.pi/2 < now < 0:
    #         if 0 < aim < math.pi/2:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   0*180/math.pi))
    #         elif math.pi/2 < aim < math.pi:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   math.pi/2*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   0*180/math.pi))
    #         elif -math.pi < aim < -math.pi/2:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   -math.pi/2*180/math.pi))
    #     elif -math.pi < now < -math.pi/2:
    #         if 0 < aim < math.pi/2:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   0*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                             current_pose[1], \
    #                             current_pose[2], \
    #                             -math.pi/2*180/math.pi))
    #         elif math.pi/2 < aim < math.pi:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   math.pi*180/math.pi))
    #         elif -math.pi/2 < aim < 0:
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   aim*180/math.pi))
    #             self.path_pos.append((current_pose[0], \
    #                                   current_pose[1], \
    #                                   current_pose[2], \
    #                                   -math.pi/2*180/math.pi))

    def get_waypoint(self):
        response = None
        p = self.path_pos.pop()
        if self.path_pos == []:
            response = True
        return p, response
