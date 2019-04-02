import json
import cv2
import math
import numpy as np
import time

class Graph:

    deg2rad = math.pi/180

    real2map_scale = 100 #50
    d_gateW = 0.5
    d_drone = 0.05

    d_grid = 0.5
    d_2gate = d_drone + d_grid/2  #0.3

    @classmethod
    def scale(cls, x):
        return int(x*cls.real2map_scale)

    def __init__(self, JSON_world):
        self.world = JSON_world
        self.length_x = self.world['airspace']['max'][0] - self.world['airspace']['min'][0]
        self.length_y = self.world['airspace']['max'][1] - self.world['airspace']['min'][1]
        self.len_x_neg = self.world['airspace']['min'][0]
        self.len_x_neg = -self.len_x_neg if self.len_x_neg < 0 else self.len_x_neg
        self.len_y_neg = self.world['airspace']['min'][1]
        self.len_y_neg = -self.len_y_neg if self.len_y_neg < 0 else self.len_y_neg
        # self.rotation = cv2.getRotationMatrix2D((self.scale(self.length_x/2), self.scale(self.length_y/2)), 90, 1)
        self.rotation = cv2.getRotationMatrix2D((200,200), 90, 1)

        self.map_shape = (self.scale(self.length_x), self.scale(self.length_y))
        self.origin = (self.scale(self.len_x_neg), self.scale(self.len_y_neg))

        self.occupancy   = np.zeros( (self.map_shape[0], self.map_shape[1], 2) )
        self.occupancy_height = np.zeros(self.map_shape)
        self.nodes       = np.zeros(self.map_shape)
        self.edges       = np.zeros(self.map_shape)
        self.marker_map  = np.zeros(self.map_shape)
        self.closest_map = np.zeros(self.map_shape)

        self.objects_list = []
        self.height_dict = dict()
        self.add_walls()
        self.add_gates()
        self.inflate()

        self.nodes2 = []
        self.add_gate_nodes()
        self.add_grid_nodes()

        self.nodes2 = np.insert(np.array(sorted(self.nodes2, key=lambda x: \
                        (x[0], x[1]))), 0, range(len(self.nodes2)), axis=1)
        self.edge_map = np.zeros((self.nodes2.shape[0], self.nodes2.shape[0], 5))

        self.node_closest_to_marker()
        self.connect_nodes()
        self.convert_nodes_pos()
        self.populate_edge_map()

        # Height later

        self.show_map(self.occupancy[:,:,0] + self.nodes + self.edges + self.marker_map + self.closest_map)

    def node_closest_to_marker(self):
        nodes_all = self.nodes2[:,1:3].astype('int')

        for m in self.world['markers']:
            x, y = m['pose']['position'][:2]
            x_idx = self.pos2idx(x, 'x')
            y_idx = self.pos2idx(y, 'y')
            if m['pose']['orientation'][0] in [0.0,90.0]:
                if m['pose']['orientation'][0] == 90.0:
                    # floor
                    closest_node = self.nodes2[self.get_idx([x_idx, y_idx]), 1:3].astype('int')

                elif m['pose']['orientation'][0] == 0.0:
                    # wall
                    angle = m['pose']['orientation'][2]+90
                    angle = math.pi/180*angle
                    direction = np.array([math.cos(angle), math.sin(angle)])
                    nodes = np.array([[x-x_idx,y-y_idx] for x,y in nodes_all])
                    angles = np.arctan2(nodes[:,1], nodes[:,0])
                    angles = np.subtract(angles, angle)
                    angles = np.mod(angles+math.pi, 2*math.pi)-math.pi
                    angles = abs(angles)
                    nodes = np.array(nodes_all)[np.where(angles<70*math.pi/180)]
                    nodes = nodes[np.where(np.sum(abs(np.subtract(nodes, [x_idx, y_idx])), axis=1)>0)]
                    closest_node = nodes[np.argmin(np.sum(abs(np.subtract(nodes, [x_idx, y_idx])), axis=1))]

                self.marker_map[x_idx-2:x_idx+2,y_idx-2:y_idx+2] = 1
                self.marker_map[closest_node[0],closest_node[1]] = 0.5
                self.closest_map[closest_node[0]-2:closest_node[0]+2,\
                                 closest_node[1]-2:closest_node[1]+2] = 0.7

                self.nodes2[(self.nodes2[:,1:3].astype('int') == closest_node)\
                            .all(axis=1).nonzero()[0][0], 7] = m['id']

    def convert_nodes_pos(self):
        for i in range(len(self.nodes2)):
            self.nodes2[i,3] = self.nodes2[i,1].astype('float')\
                                    /self.real2map_scale-self.len_x_neg
            self.nodes2[i,4] = self.nodes2[i,2].astype('float')\
                                    /self.real2map_scale-self.len_y_neg

    def populate_edge_map(self):
        gate_idxs = self.nodes2[np.where(self.nodes2[:,6] != 'False'), 0].astype('int')
        for idx in gate_idxs[0]:
            for idx2 in gate_idxs[0]:
                if  self.nodes2[idx, 6][0] == self.nodes2[idx2, 6][0] and \
                    self.nodes2[idx, 6][-1] != self.nodes2[idx2, 6][-1]:

                    self.edge_map[idx, idx2, 0] = 1
                    self.edge_map[idx2, idx, 0] = 1
                    self.edge_map[idx, idx2, 4] = self.nodes2[idx, 6][0]
                    self.edge_map[idx2, idx, 4] = self.nodes2[idx, 6][0]

        for idx in range(len(self.nodes2)):
            neighbors = np.where(self.edge_map[idx,:,0] == 1)[0]
            for idx2 in neighbors:
                pos1 = self.nodes2[idx, 3:5].astype('float')
                pos2 = self.nodes2[idx2,3:5].astype('float')
                length = np.linalg.norm(np.subtract(pos1, pos2))
                angle  = math.atan2(pos2[1]-pos1[1], pos2[0]-pos1[0])*180/math.pi

                self.edge_map[idx, idx2, 1] = length
                self.edge_map[idx, idx2, 2] = angle

                # calc height ??

    def calc_idxs(self, start, stop):
        xstart = start[0]
        ystart = start[1]
        deltax = stop[0] - xstart
        deltay = stop[1] - ystart
        length = math.sqrt((deltax)**2 + (deltay)**2)
        angle = math.atan2(deltay, deltax)

        l = np.array(np.linspace(0, length, int(length)))
        idxs_x = []
        idxs_y = []
        for i in range(len(l)):
            pt = (start[0] + int(l[i]*math.cos(angle)), start[1] + int(l[i]*math.sin(angle)))
            if  0 <= pt[0] < self.map_shape[0] and 0 <= pt[1] < self.map_shape[1]:
                idxs_x.append(pt[0])
                idxs_y.append(pt[1])
        return idxs_x, idxs_y

    def calc_angle(self, start, stop):
        return math.atan2(stop[1]-start[1], stop[0]-start[0])

    def calc_start_stop(self, middle, angle, halv_length):
        start = [middle[0] - int(halv_length*math.cos(angle)), middle[1] - int(halv_length*math.sin(angle))]
        stop  = [middle[0] + int(halv_length*math.cos(angle)), middle[1] + int(halv_length*math.sin(angle))]
        return start, stop, self.calc_angle(start, stop), self.calc_idxs(start, stop)

    def add_gate_nodes(self):
        for g in self.world['gates']:
            id = g['id']
            x = g['position'][0]
            y = g['position'][1]
            angle = g['heading']*self.deg2rad
            pts = [(self.pos2idx(x+self.d_2gate*math.cos(angle), 'x'), \
                    self.pos2idx(y+self.d_2gate*math.sin(angle), 'y')), \
                   (self.pos2idx(x-self.d_2gate*math.cos(angle), 'x'), \
                    self.pos2idx(y-self.d_2gate*math.sin(angle), 'y'))]

            for i in range(len(pts)):
                self.nodes[pts[i][0],pts[i][1]] = 1
                self.nodes2.append( [pts[i][0], pts[i][1], 0, 0, False, '%d-%d' % (id, (i+1)%2), False, False] )

            self.edges[self.calc_idxs(pts[0],pts[1])] = 0.5

    def add_grid_nodes(self):
        min_x = int(self.world['airspace']['min'][0]/self.d_grid)+1
        max_x = int(self.world['airspace']['max'][0]/self.d_grid)
        min_y = int(self.world['airspace']['min'][1]/self.d_grid)+1
        max_y = int(self.world['airspace']['max'][1]/self.d_grid)
        pos = np.array([[self.pos2idx(0 + i*self.d_grid, 'x'), self.pos2idx(0 + j*self.d_grid, 'y')] \
                    for i in range(min_x,max_x) for j in range(min_y,max_y)])
        size = 6 #2 ##############################################\
        ####################################################
        for p in pos:
            if np.sum(self.nodes[p[0]-size:p[0]+size,p[1]-size:p[1]+size] + self.occupancy[p[0]-size:p[0]+size,p[1]-size:p[1]+size, 0]) == 0:
                self.nodes[p[0], p[1]] = 1

                self.nodes2.append( [p[0], p[1], 0, 0, True, False, False, False] )

    def connect_nodes(self):
        def are_we_neighbors(me, my_neighbor):
            size = 2  ############################################\
            ############################################

            ix, iy = self.calc_idxs(me, my_neighbor)
            idxs = np.array(list({(ix[i]+x, iy[i]+y) for i in range(len(ix)) for x in range(-size,size+1) for y in range(-size,size+1)}))

            if idxs.shape[0] > 0: #and np.sum(self.occupancy[idxs[:,0], idxs[:,1]]) == 0:
                heights = np.array([self.height_dict[tuple(x)] for x in idxs if tuple(x) in self.height_dict])
                heights = [0] if len(heights) == 0 else heights
                height = max(heights)
                if height < 1000:
                    me_idx = self.get_idx(me)
                    neighbor_idx = self.get_idx(my_neighbor)

                    self.edge_map[me_idx, neighbor_idx, 0] = 1
                    self.edge_map[neighbor_idx, me_idx, 0] = 1
                    self.edge_map[me_idx, neighbor_idx, 3] = height
                    self.edge_map[neighbor_idx, me_idx, 3] = height
                    self.edges[ix, iy] = 0.5

        dist = self.scale(5*self.d_grid) ## search dist, 1.5  ####################\
        ############################################
        neighbor_add = [(x,y) for x in [-self.scale(self.d_grid), 0, self.scale(self.d_grid)] \
                       for y in [-self.scale(self.d_grid), 0, self.scale(self.d_grid)] \
                       if not (x==0 and y==0)]

        nodes_all = self.nodes2[:,1:3].astype('int')
        # for n in nodes_all:
        #     neighbors = np.array([n[0],n[1]]) + neighbor_add
        #     for nn in neighbors:
        #         if len((nodes_all == nn).all(axis=1).nonzero()[0]) > 0:
        #             are_we_neighbors(n, (nn[0],nn[1]))

        for n in nodes_all:
            neighbors_idxs = np.argwhere(np.linalg.norm(nodes_all - n, axis=1)<dist)
            if len(neighbors_idxs)>0:
                neighbors = [nodes_all[i] for i in neighbors_idxs]
                for nn in neighbors:
                    if not ( (n[0]==nn[0][0]) and (n[1]==nn[0][1]) ):
                        are_we_neighbors(n, (nn[0][0], nn[0][1]))

        for gn in self.nodes2[(self.nodes2[:,6] != 'False').nonzero(), 1:3].astype('int')[0]:
            neighbors_idxs = np.argwhere(np.linalg.norm(nodes_all - gn, axis=1)<dist)
            if len(neighbors_idxs) > 0:
                neighbors = [nodes_all[i] for i in neighbors_idxs]
                for nn in neighbors:
                    if not ((gn[0] == nn[0][0]) and (gn[1] == nn[0][1])):
                        are_we_neighbors(gn, (nn[0][0],nn[0][1]))

    def add_walls(self):
        for w in self.world['walls']:
            start = w['plane']['start']
            stop = w['plane']['stop']
            x1 = float(self.pos2idx(start[0], 'x'))
            y1 = float(self.pos2idx(start[1], 'y'))
            x2 = float(self.pos2idx(stop[0], 'x'))
            y2 = float(self.pos2idx(stop[1], 'y'))
            angle = self.calc_angle((x1,y1), (x2,y2))
            idxs = np.array(self.calc_idxs((x1,y1), (x2,y2))).astype('int')
            height = stop[2]
            self.objects_list.append([x1,y1,x2,y2, angle, idxs, height])
            for i in range(len(idxs[0])):
                self.height_dict[ (idxs[0][i], idxs[1][i]) ] = height

    def add_gates(self):
        for g in self.world['gates']:
            x = g['position'][0]
            y = g['position'][1]
            angle = (g['heading']+90)*self.deg2rad
            x1 = float(self.pos2idx(x, 'x'))
            y1 = float(self.pos2idx(y, 'y'))
            start, stop, angle, idxs = self.calc_start_stop((x1,y1), angle, self.scale(self.d_gateW/2))
            idxs = np.array(idxs).astype('int')
            height = 1000
            self.objects_list.append([start[0], start[1], stop[0], stop[1], angle, idxs, height])
            for i in range(len(idxs[0])):
                self.height_dict[ (idxs[0][i], idxs[1][i]) ] = height

    def get_idx(self, pos):
        return np.argmin(np.sum(abs(np.subtract(\
                        self.nodes2[:,1:3].astype('int'), [pos[0], pos[1]])), axis=1))

    def pos2idx(self, value, x_or_y):
        if x_or_y == 'x':
            return self.scale(value + self.len_x_neg)
        else:
            return self.scale(value + self.len_y_neg)

    def show_map(self, map):
        cv2.imshow('black_white', cv2.warpAffine(map, self.rotation, map.shape))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def inflate(self): #objects_list, lenx, leny, shape):
        self.occupancy = np.zeros( (self.map_shape[0], self.map_shape[1], 2) )
        idxs_x = []
        idxs_y = []
        theta1 = [0, math.pi]
        theta2 = np.linspace(0, math.pi, 100)
        for i in range(len(self.objects_list)):
            startx, starty, stopx, stopy, angle, idxs, height = self.objects_list[i]
            startx = int(startx)
            starty = int(starty)
            stopx = int(stopx)
            stopy = int(stopy)
            # idxs = object_idxs_list[i]
            for i in range(len(idxs[0])):
                pt1 = (idxs[0][i] + int(self.scale(self.d_drone)*math.cos(angle+math.pi/2)), idxs[1][i] + int(self.scale(self.d_drone)*math.sin(angle+math.pi/2)))
                if  0 <= pt1[0] < self.map_shape[0] and 0 <= pt1[1] < self.map_shape[1]:
                    idxs_x.append(pt1[0])
                    idxs_y.append(pt1[1])
                    self.occupancy[pt1[0], pt1[1], 1] = height
                pt2 = (idxs[0][i] - int(self.scale(self.d_drone)*math.cos(angle+math.pi/2)), idxs[1][i] - int(self.scale(self.d_drone)*math.sin(angle+math.pi/2)))
                if  0 <= pt2[0] < self.map_shape[0] and 0 <= pt2[1] < self.map_shape[1]:
                    idxs_x.append(pt2[0])
                    idxs_y.append(pt2[1])
                    self.occupancy[pt2[0], pt2[1], 1] = height
            for t in theta2:
                pt1 = (startx + int(self.scale(self.d_drone)*math.cos(angle - math.pi/2 - t)), starty + int(self.scale(self.d_drone)*math.sin(angle - math.pi/2 - t)))
                if  0 <= pt1[0] < self.map_shape[0] and 0 <= pt1[1] < self.map_shape[1]:
                    idxs_x.append(pt1[0])
                    idxs_y.append(pt1[1])
                    self.occupancy[pt1[0], pt1[1], 1] = height
                pt2 = (stopx  + int(self.scale(self.d_drone)*math.cos(angle - math.pi/2 + t)), stopy  + int(self.scale(self.d_drone)*math.sin(angle - math.pi/2 + t)))
                if  0 <= pt2[0] < self.map_shape[0] and 0 <= pt2[1] < self.map_shape[1]:
                    idxs_x.append(pt2[0])
                    idxs_y.append(pt2[1])
                    self.occupancy[pt2[0], pt2[1], 1] = height
        self.occupancy[idxs_x, idxs_y, 0] = 1
        self.occupancy[0+self.scale(self.d_drone),:, 0] = self.occupancy[self.map_shape[0]-self.scale(self.d_drone),:, 0] = 1
        self.occupancy[:,0+self.scale(self.d_drone), 0] = self.occupancy[:,self.map_shape[1]-self.scale(self.d_drone), 0] = 1
        self.occupancy[0+self.scale(self.d_drone),:, 1] = self.occupancy[self.map_shape[0]-self.scale(self.d_drone),:, 1] = 1000
        self.occupancy[:,0+self.scale(self.d_drone), 1] = self.occupancy[:,self.map_shape[1]-self.scale(self.d_drone), 1] = 1000


def save_graph():
    t0 = time.time()
    name = 'nav_challenge.world.json'
    # name = 'part2.world.json'
    path = '/home/mikael/dd2419_ws/src/course_packages/dd2419_resources/worlds_json'
    file = path + '/' + name
    world = json.load(open(file, 'rb'))
    graph = Graph(world)

    real2map_scale = graph.real2map_scale
    d_drone = graph.d_drone
    map_shape = graph.map_shape
    origin = graph.origin
    objects_list = graph.objects_list
    edge_map = graph.edge_map
    nodes = graph.nodes2


    from os.path import abspath, dirname, join
    path = join(dirname(dirname(abspath(__file__))), 'files')

    import pickle

    with open(join(path, 'planning_data_fine.txt'), 'wb') as fp:
            pickle.dump([real2map_scale, d_drone, map_shape, origin, objects_list, edge_map, nodes], fp)
            # pickle.dump(graph, fp)

    t1 = time.time()
    print('time %.3f' % (t1- t0))
