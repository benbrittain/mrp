import copy

import math
class Graph:

    def __init__(self):
        self.nodes = {}

    def add_node(self, key, *xy):

        # x-y coordinates and adjacency list as dictionary
        if len(xy) != 2:
            raise Exception('Coordinates (x, y) should be tuple of length 2')
        self.nodes[key] = [xy, {}]
        self.cache = {}

    def connect(self, node1, node2, weight=None):

        if weight is not None and weight < 0:
            raise Exception('Positive weights only')

        if node1 not in self.nodes or node2 not in self.nodes:
            raise Exception('Nodes not in graph')

        if weight is None:
            n1_xy = self.nodes[node1][0]
            n2_xy = self.nodes[node2][0]
            weight = self.euclidian(n1_xy, n2_xy)

        self.nodes[node1][1][node2] = weight
        self.nodes[node2][1][node1] = weight

    @staticmethod
    def euclidian(p1, p2):
        return math.sqrt((p1[1] - p2[1]) ** 2 + (p1[0] - p2[0]) ** 2)

    def get_connected_nodes(self, n):
        if n not in self.nodes:
            raise Exception('Node not in graph')
        return copy.deepcopy(self.nodes[n][1])

    def cache_path(self, source, dest):
        s_xy = self.nodes[source][0]
        pts = []
        for d in dest:
            d_xy = self.nodes[d][0]
            pts.append(d_xy)
        d_xy = self.nodes[dest[-1]][0]
        self.cache[s_xy, d_xy] = pts

    def get_path_between(self, *sd):

        start_node = self.get_closest_node_to(sd[0])
        end_node = self.get_closest_node_to(sd[1])
        x = self.cache[start_node[0], end_node[0]]

        # Do not add to path start and end nodes already in it
        if start_node[0] != sd[0]:
            x.insert(0, sd[0])
        if end_node[0] != sd[1]:
            x.append(sd[1])
        return x[:]

    def get_closest_node_to(self, start):
        closest = None
        d = float('inf')
        for n in self.nodes:
            dn = self.euclidian(start, self.nodes[n][0])
            if dn < d:
                d = dn
                closest = self.nodes[n]
        return closest

    def pre_compute_shortest_path_pairs(self, dist, pred):
        for source in pred:
            for dest in self.nodes:
                if source == dest:
                    continue
                prev = None
                path = [dest]
                x = dest
                while prev != source:
                    prev = pred[source][x]
                    path.append(prev)
                    x = prev
                path.reverse()
                self.cache_path(source, path[1:])

    def floyd_warshall(self):
        '''
        Algorithm from
        https://jlmedina123.wordpress.com/2014/05/17/floyd-warshall-algorithm-in-python/
        retrofitted to suit me.
        :return:
        '''

        dist = {}
        pred = {}
        for u in self.nodes:
            dist[u] = {}
            pred[u] = {}
            for v in self.nodes:
                dist[u][v] = 1000
                pred[u][v] = -1
            dist[u][u] = 0
            for neighbor in self.nodes[u][1]:
                dist[u][neighbor] = self.nodes[u][1][neighbor]
                pred[u][neighbor] = u
        for t in self.nodes:
            for u in self.nodes:
                for v in self.nodes:
                    newdist = dist[u][t] + dist[t][v]
                    if newdist < dist[u][v]:
                        dist[u][v] = newdist
                        pred[u][v] = pred[t][v]

        self.pre_compute_shortest_path_pairs(dist, pred)



g = Graph()
g.add_node('a',-54.54,9)
g.add_node('b',-54.43,13)
g.connect('a', 'b')

g.add_node('c',-46,12.51)
g.connect('b', 'c')

g.add_node('d',-46,2)
g.connect('c', 'd')

g.add_node('e',-46,-9)
g.connect('d', 'e')

g.add_node('f',-36,12.5)
g.connect('c', 'f')

g.add_node('g',-10,12.5)
g.connect('f', 'g')


g.add_node('h',8,12.6)
g.connect('g', 'h')


g.add_node('i',8,1)
g.connect('h', 'i')


g.add_node('j',8,-9)
g.connect('i', 'j')


g.add_node('k',-28,-9)
g.connect('j', 'k')
g.connect('e', 'k')



g.floyd_warshall()

#print g.get_path_between((-46,10), (8,1))



