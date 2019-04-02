import matplotlib.pyplot as plt
import matplotlib.animation as animation

import numpy as np
import math
import time
import datetime

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class waypointGenerator:
    def __init__(self, stuff):
        self.pub_image = rospy.Publisher('path_planning/map', Image, queue_size=2)
        self.bridge = CvBridge()


        self.real2map_scale = stuff[0]
        self.d_drone = stuff[1]*4
        self.map_shape = stuff[2]
        self.origin = stuff[3]
        self.objects_list = stuff[4]
        self.edge_map = stuff[5]
        self.nodes = stuff[6]

        self.dx = int((self.d_drone+0.0)*self.real2map_scale)
        self.dy = int((self.d_drone+0.0)*self.real2map_scale)
        self.dangle = 0
        self.occupancy = np.zeros( (self.map_shape[0], self.map_shape[1], 2) )
        self.inflate()
        self.path_idx = []
        self.waypoints = []
        self.this_waypoint = None
        self.start_goal = []
        self.mode = None
        self.visited_map = np.zeros(self.map_shape)

        self.show = True

        if self.show:
            self.nodes_map = np.zeros(self.map_shape)
            self.marker_map = np.zeros(self.map_shape)
            self.fill_nodes_map()

        self.map_record = []
        plot=None


    def init(self):
        global plot
        plot.set_data(self.map_record[0])
        return plot,

    def update(self, j):
        global plot
        plot.set_data(self.map_record[j])
        return [plot]

    def show_record(self):
        global plot
        plt.clf()
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=5, metadata=dict(artist='Me'), bitrate=1800)

        n_frames=len(self.map_record)
        fig = plt.figure()
        plot = plt.matshow(self.map_record[0], fignum=0)

        ani = animation.FuncAnimation(fig, self.update, init_func = self.init, \
                frames=n_frames, interval=30, blit=True)
        path = '/home/mikael/project_ws2/src/planning/src/files'
        name = 'planning-{date:%Y-%m-%d %H:%M:%S}.mp4'.format( date=datetime.datetime.now() )
        ani.save(path + '/' + name, writer=writer)
        print('all done!')

    def fill_nodes_map(self):
        for i in range(len(self.nodes)):
            n = tuple(self.nodes[i,1:3].astype('int'))
            if self.nodes[i,7] != 'False': # close to marker
                self.marker_map[n[0]-2:n[0]+2,n[1]-2:n[1]+2] = 0.7
            else:
                self.nodes_map[n] = 1

    def get_closest_node(self,x,y):
        sum = np.sum(abs(np.subtract(self.node_idx_dict.keys(), [x,y])), axis=1)
        return self.node_idx_dict.keys()[np.argmin(sum)]


    def calc_path(self):
        self.path_idx = []
        self.waypoints = []
        response = None
        if isinstance(self.start_goal[1], int):
            # to gate/through gate
            self.inflate()
            response = self.a_star_path(self.start_goal[0], self.start_goal[1])
            print(self.path_idx)
        elif isinstance(self.start_goal[1], np.ndarray):
            # marker
            response = False
            radius = int(0.1*self.real2map_scale) #####################\
            #                 ###################################3
            for goal in self.start_goal[1]:
                goal = goal[0] if isinstance(goal, np.ndarray) else goal
                idxs = np.array([[x,y] \
                        for x in range(self.nodes[goal,1].astype('int')-radius, self.nodes[goal,1].astype('int')+radius+1) \
                        for y in range(self.nodes[goal,2].astype('int')-radius, self.nodes[goal,2].astype('int')+radius+1)])
                idxs = (idxs[:,0], idxs[:,1])
                if self.show:
                    map = np.zeros(self.map_shape)
                    map[idxs] = 1
                    self.show_map(map)

                if np.sum(self.occupancy[idxs, 0]) == 0:   #######################################
                    self.inflate()
                    response = self.a_star_path(self.start_goal[0], goal)
                    if response is True:
                        break
        elif self.mode == 'safe':
            self.inflate()
            response = self.a_star_inv(self.start_goal[0])
        elif self.mode == 'explore':
            bad_nodes = []
            response = False
            not_visited = self.nodes[np.where(self.nodes[:,8]=='False'),:][0]
            while response is False and len(bad_nodes) < not_visited.shape[0]:
                condition1 = np.array(not_visited[:,6]=='False').astype('int')
                condition2 = abs(np.array(np.isin(not_visited[:,0].astype('int'),\
                                np.array(bad_nodes))).astype('int')-1)
                total_cond = np.multiply(condition1, condition2)
                accepted_idx = not_visited[np.where(total_cond==1), 0].astype('int')
                accepted_nodes = self.nodes[accepted_idx, :3].astype('int')[0]
                if len(accepted_nodes) > 0:
                    goal_accepted = np.argmin(np.linalg.norm(np.subtract(accepted_nodes[:,1:], \
                                    self.nodes[self.start_goal[0], 1:3].astype('int')), axis=1))
                    goal = self.nodes[accepted_nodes[goal_accepted,0],0].astype('int')
                    self.inflate()
                    response = self.a_star_path(self.start_goal[0], goal)
                    if response is False:
                        bad_nodes.append(goal)
        self.start_goal = []
        if response is False:
            return self.this_waypoint, False
        return self.get_waypoint()


    def get_waypoint(self):
        if self.this_waypoint is not None:
            self.nodes[self.get_idx(self.this_waypoint[:2]).astype('int'),8] = True
        if self.start_goal != []:
            return self.calc_path()

        # now there are waypoints
        this_idx = self.get_idx([400,200]) if self.this_waypoint is None else self.get_idx(self.this_waypoint[:2])
        next_waypoint = self.waypoints.pop(0)
        next_idx = self.get_idx(next_waypoint[:2])
        if next_idx != this_idx:
            self.inflate()
            if self.get_edge_cost([this_idx, next_idx], True) > 1:
                # step is invalid
                goal_idx = next_idx if self.waypoints == [] else self.get_idx(self.waypoints[-1][:2])
                self.start_goal = [this_idx, goal_idx]
                self.path_idx = []
                self.waypoints = []
                return self.get_waypoint()

        # all good
        response = None if len(self.waypoints) > 0 else True
        self.this_waypoint = next_waypoint
        if self.start_goal != []:
            return self.get_waypoint()
        return next_waypoint, response

    def get_idx(self, pos):
        if isinstance(pos[0], float):
            return np.argmin(np.sum(abs(np.subtract(\
                            self.nodes[:,3:5].astype('float'), [pos[0], pos[1]])), axis=1))
        else:
            return np.argmin(np.sum(abs(np.subtract(\
                            self.nodes[:,1:3].astype('int'), [pos[0], pos[1]])), axis=1))

    def set_start_goal(self, start, goal, spec=None):
        self.mode = None
        if goal is None and spec == 'marker':
            nodes = self.nodes[np.where(self.nodes[:,7]!='False'),:3].astype('int')[0]
            goal = np.array([self.nodes[nodes[n,0].astype('int'),0].astype('int') \
                    for n in np.argsort(np.linalg.norm(np.subtract(nodes[:,1:], \
                    self.nodes[start, 1:3].astype('int')), axis=1))])
            self.mode = 'marker'
        elif goal is None and spec == 'safe':
            goal = None
            self.mode = 'safe'
        elif goal is None and spec == 'explore':
            goal = None
            self.mode = 'explore'
        self.start_goal = [start, goal]
        # self.get_waypoint()

    def set_uncertainties(self, dx, dy, dangle):
        self.dx = int((self.d_drone + dx)*self.real2map_scale)
        self.dy = int((self.d_drone + dy)*self.real2map_scale)
        self.dangle = dangle

    def inflate(self):
        self.occupancy[:,:,:] = 0
        # idxs_x = []
        # idxs_y = []
        theta1 = [0, math.pi]
        theta2 = np.linspace(0, math.pi, 100)
        dx_list = [self.dx]
        dy_list = [self.dy]
        for k in range(len(dx_list)):
            for i in range(len(self.objects_list)):
                startx, starty, stopx, stopy, angle, idxs, height = self.objects_list[i]
                startx = int(startx)
                starty = int(starty)
                stopx = int(stopx)
                stopy = int(stopy)
                for i in range(len(idxs[0])):
                    pt1 = (idxs[0][i] + int(dx_list[k]*math.cos(angle+math.pi/2)), idxs[1][i] + int(dy_list[k]*math.sin(angle+math.pi/2)))
                    if  0 <= pt1[0] < self.map_shape[0] and 0 <= pt1[1] < self.map_shape[1]:
                        # idxs_x.append(pt1[0])
                        # idxs_y.append(pt1[1])
                        self.occupancy[pt1[0], pt1[1], 0] = 1
                        self.occupancy[pt1[0], pt1[1], 1] = height
                    pt2 = (idxs[0][i] - int(dx_list[k]*math.cos(angle+math.pi/2)), idxs[1][i] - int(dy_list[k]*math.sin(angle+math.pi/2)))
                    if  0 <= pt2[0] < self.map_shape[0] and 0 <= pt2[1] < self.map_shape[1]:
                        # idxs_x.append(pt2[0])
                        # idxs_y.append(pt2[1])
                        self.occupancy[pt2[0], pt2[1], 0] = 1
                        self.occupancy[pt2[0], pt2[1], 1] = height
                for t in theta2:
                    pt1 = (startx + int(dx_list[k]*math.cos(angle - math.pi/2 - t)), starty + int(dy_list[k]*math.sin(angle - math.pi/2 - t)))
                    if  0 <= pt1[0] < self.map_shape[0] and 0 <= pt1[1] < self.map_shape[1]:
                        # idxs_x.append(pt1[0])
                        # idxs_y.append(pt1[1])
                        self.occupancy[pt1[0], pt1[1], 0] = 1
                        self.occupancy[pt1[0], pt1[1], 1] = height
                    pt2 = (stopx  + int(dx_list[k]*math.cos(angle - math.pi/2 + t)), stopy  + int(dy_list[k]*math.sin(angle - math.pi/2 + t)))
                    if  0 <= pt2[0] < self.map_shape[0] and 0 <= pt2[1] < self.map_shape[1]:
                        # idxs_x.append(pt2[0])
                        # idxs_y.append(pt2[1])
                        self.occupancy[pt2[0], pt2[1], 0] = 1
                        self.occupancy[pt2[0], pt2[1], 1] = height
        # self.occupancy[idxs_x, idxs_y] = 1
        self.occupancy[0+self.dx,:, 0] = self.occupancy[self.map_shape[0]-self.dx,:, 0] = 1
        self.occupancy[:,0+self.dy, 0] = self.occupancy[:,self.map_shape[1]-self.dy, 0] = 1
        self.occupancy[0+self.dx,:, 1] = self.occupancy[self.map_shape[0]-self.dx,:, 1] = 1000
        self.occupancy[:,0+self.dy, 1] = self.occupancy[:,self.map_shape[1]-self.dy, 1] = 1000

    def idx_sector(self, xstart, ystart, length, angle):
        def add_pt(alist, blist, pt):
            size = 1
            for i in range(-size,size+1):
                for j in range(-size,size+1):
                    alist.append(pt[0]+i)
                    blist.append(pt[1]+j)

        nr_length = int(length) if length > 0 else 1
        l = np.array(np.linspace(0, length, nr_length))
        nr_theta = int(360/math.pi*self.dangle) if self.dangle > 0 else 1
        theta1 = np.array([angle-self.dangle, angle+self.dangle])
        theta2 = np.array(np.linspace(angle-self.dangle, angle+self.dangle, nr_theta))
        idxs_x = []
        idxs_y = []
        for i in range(len(l)):
            pt1 = (xstart + int(l[i]*math.cos(theta1[0])), ystart + int(l[i]*math.sin(theta1[0])))
            if  0 <= pt1[0] < self.map_shape[0] and 0 <= pt1[1] < self.map_shape[1]:
                add_pt(idxs_x, idxs_y, pt1)
            pt2 = (xstart + int(l[i]*math.cos(theta1[1])), ystart + int(l[i]*math.sin(theta1[1])))
            if  0 <= pt2[0] < self.map_shape[0] and 0 <= pt2[1] < self.map_shape[1]:
                add_pt(idxs_x, idxs_y, pt2)
        for t in theta2:
            pt = (xstart + int(length*math.cos(t)), ystart + int(length*math.sin(t)))
            if  0 <= pt[0] < self.map_shape[0] and 0 <= pt[1] < self.map_shape[1]:
                add_pt(idxs_x, idxs_y, pt)
        return idxs_x, idxs_y

    def path_idx2path_pos(self):
        self.waypoints = []
        for i in range(1, len(self.path_idx)):
            idx1 = self.path_idx[i-1]
            idx2 = self.path_idx[i]

            n1x = self.nodes[idx1, 3].astype('float')
            n1y = self.nodes[idx1, 4].astype('float')
            n2x = self.nodes[idx2, 3].astype('float')
            n2y = self.nodes[idx2, 4].astype('float')
            direction = self.edge_map[idx1, idx2, 2]
            height = self.edge_map[idx1, idx2, 3]

            self.waypoints.append([n1x, n1y, 0.4, direction])
            if height > 0:
                self.waypoints.append([n1x, n1y, 0.4+height, direction])
                self.waypoints.append([n2x, n2y, 0.4+height, direction])
            self.waypoints.append([n2x, n2y, 0.4, direction])

        if self.waypoints == []:
            self.waypoints = [self.this_waypoint]

    def get_edge_cost(self, edge, show=False):
        if self.mode in ['safe', 'marker', 'explore'] and self.edge_map[edge[0], edge[1], 4] > 0:
            return 1000

        start = self.nodes[edge[0],1:3].astype('int')
        stop  = self.nodes[edge[1],1:3].astype('int')
        length = self.edge_map[edge[0],edge[1],1]*self.real2map_scale
        angle  = self.edge_map[edge[0],edge[1],2]*math.pi/180
        height = self.edge_map[edge[0],edge[1],3]
        idxs = self.idx_sector(start[0], start[1], length, angle)
        if self.show and show:
            map = np.zeros(self.map_shape)
            map[idxs] = 0.5
            self.show_map(map)

        if self.edge_map[edge[0], edge[1], 4] == 0 and max(self.occupancy[idxs[0], idxs[1], 1]) > height:
            return 1000
        return 1


        # if self.edge_map[edge[0], edge[1], 4] == 0 and np.sum(self.occupancy[idxs[0], idxs[1], 0]) > 0:
        #     return 1000
        # return 1

    def reconstruct_path(self, came_from, current, pts_map):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        total_path = total_path[::-1]

        self.path_idx = total_path
        self.path_idx2path_pos()

        if self.show:
            map = np.zeros(self.map_shape)
            for i in range(1, len(self.path_idx)):
                edge = [self.path_idx[i-1], self.path_idx[i]]
                start = self.nodes[edge[0],1:3].astype('int')
                length = self.edge_map[edge[0],edge[1],1]*self.real2map_scale
                angle = self.edge_map[edge[0],edge[1],2]*math.pi/180
                map[self.idx_sector(start[0], start[1], length, angle)] = 0.3
            self.show_map(map + pts_map)

        return len(self.path_idx) > 0

    def heuristic_cost_estimate(self, start, goal):
        return np.linalg.norm(np.subtract(self.nodes[start, 1:3].astype('int'),self.nodes[goal, 1:3].astype('int')))

    def score_of(self, x, score_dict):
        if x not in score_dict.keys():
            return 1e99
        else:
            return score_dict[x]

    def a_star_path(self, start, goal):

        if self.show:
            pts_map = np.zeros(self.map_shape)
            s = self.nodes[start, 1:3].astype('int')
            g = self.nodes[goal, 1:3].astype('int')
            pts_map[s[0]-1:s[0]+1,s[1]-1:s[1]+1] = 1
            pts_map[g[0]-2:g[0]+2,g[1]-2:g[1]+2] = 1
            map = self.occupancy[:,:,0] + pts_map

        closed_set = set()
        open_set = set()
        open_set.add(start)
        came_from = {}

        g_score = {}
        g_score[start] = 0

        f_score = {}
        f_score[start] = self.heuristic_cost_estimate(start, goal)
        while len(open_set) > 0:
            open_set_in_f = np.array([[x,self.score_of(x, f_score)] for x in open_set])
            current = open_set_in_f[np.argmin(open_set_in_f[:,1]),0].astype('int')
            if current == goal:
                success = self.reconstruct_path(came_from, current, pts_map)
                if success:
                    return True

            open_set.remove(current)
            closed_set.add(current)

            for neighbor in np.where(self.edge_map[current,:]>0)[0]:

                if neighbor not in closed_set:
                    edge_cost = self.get_edge_cost([current, neighbor])
                    neighbor_g_score = g_score[current] + \
                            self.heuristic_cost_estimate(current, neighbor)*edge_cost

                    if neighbor not in open_set and edge_cost == 1:
                        open_set.add(neighbor)
                    if neighbor_g_score < self.score_of(neighbor, g_score):
                        came_from[neighbor] = current
                        g_score[neighbor] = neighbor_g_score
                        f_score[neighbor] = g_score[neighbor] + self.heuristic_cost_estimate(neighbor, goal)

        if self.show:
            self.show_map(pts_map)
        return False

    def a_star_inv(self, start):

        if self.show:
            pts_map = np.zeros(self.map_shape)
            s = self.nodes[start, 1:3].astype('int')
            pts_map[s[0]-1:s[0]+1,s[1]-1:s[1]+1] = 1
            map = self.occupancy[:,:,0] + pts_map

        came_from = {}
        open_list = []
        open_list.append(start)
        closed_set = set()

        occupancy_pts = np.transpose(np.vstack(np.where(self.occupancy[:,:,0]==1)))

        while len(open_list) > 0:
            current = open_list.pop(0)
            current_dist = np.min(np.linalg.norm(np.subtract(\
                    occupancy_pts, self.nodes[current, 1:3].astype('int')), axis=1))
            if current_dist > 0.3*self.real2map_scale:
                success = self.reconstruct_path(came_from, current, pts_map)
                if success:
                    return True
            closed_set.add(current)

            for neighbor in np.where(self.edge_map[current,:]>0)[0]:
                if neighbor not in closed_set:
                    start_dist = np.linalg.norm(np.subtract(self.nodes[start, 1:3].astype('int'), \
                            self.nodes[neighbor, 1:3].astype('int')))
                    edge_cost = self.get_edge_cost([current, neighbor])
                    if neighbor not in open_list and edge_cost == 1 and \
                            start_dist > 0.3*self.real2map_scale:
                        came_from[neighbor] = current
                        open_list.append(neighbor)

        if self.show:
            self.show_map(pts_map)
        return False

    def show_map(self, map):
        map = self.occupancy[:,:,0] + self.nodes_map + map #+ self.marker_map #self.visited_map + map

        # plt.clf()
        # plt.imshow(np.rot90(map), cmap='gray')
        # plt.ion()
        # plt.show()
        # plt.pause(0.001)

        self.map_record.append(np.rot90(map))
        try:
            # dtype, n_channels = self.bridge.encoding_as_cvtype2('8UC3')
            # print(dtype, n_channels)
            # im = np.rot90(map).dtype(dtype)
            map = np.array(np.rot90(map)*255, dtype=np.uint8)
            # im = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)
            msg = self.bridge.cv2_to_imgmsg(map, encoding='8UC1')
            self.pub_image.publish(msg)
        except CvBridgeError as e:
            print(e)
