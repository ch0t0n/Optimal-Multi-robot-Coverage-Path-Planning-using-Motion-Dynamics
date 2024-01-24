import time
from zmqRemoteApi import RemoteAPIClient

print('Program started')
client = RemoteAPIClient()
sim = client.getObject('sim')
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)


class Drone_simulator:
    def __init__(self, rounded_polygon, k_paths):
        self.rounded_polygon = rounded_polygon + [rounded_polygon[0]]
        self.k_paths = k_paths
        self.edges_3d = self.calc_edges_3d()
        self.color = [[255,0,0],[255,0,255],[0,0,255]]
        self.updated_path = self.equal_lists(k_paths)

    def start_simulation(self):
        sim.startSimulation()

    def stop_simulation(self):
        sim.stopSimulation()

    def equal_lists(self, list):
        # Update lists only for visualization
        mx = len(list[0])
        for item in list:
            item.append(item[-1])
            if mx <= len(item):
                mx = len(item)
        for item in list:
            if mx != len(item):
                dif = mx - len(item)
                for i in range(dif):
                    item.append(item[-1])
        return list

    def calc_edges_3d(self):  # To calculate the edges in the polygon
        edges = []
        for i in range(len(self.rounded_polygon) - 1):
            edges.append([list(self.rounded_polygon[i]), list(self.rounded_polygon[i+1])])
        return edges

    def draw_field(self, height):
        white = [255, 255, 255]
        lineContainer = sim.addDrawingObject(sim.drawing_lines, 2, 0, -1, 9999, white)
        for l in self.edges_3d: # Drawing the field with white lines
            line = l[0] + [height] + l[1] + [height]
            for j in range(len(line)):
                if line[j] != height:
                    line[j] = int(line[j])
            # print(line)
            sim.addDrawingObjectItem(lineContainer, line)

        for i, path in enumerate(self.k_paths):
            lineContainer = sim.addDrawingObject(sim.drawing_lines, 2, 0, -1, 9999, self.color[i])
            for p in range(len(path)-1): # Drawing the lines in the simulator
                line = path[p] + [height] + path[p+1] + [height]
                sim.addDrawingObjectItem(lineContainer, line)

            heli = '/Helicopter['
            drone = '/Quadcopter['
            obj_path = drone+str(i)+']'
            objHandle = sim.getObject(obj_path)
            sim.setObjectPosition(objHandle, -1, path[0]+[height]) # Initiate the position of the robots

    def simulate_path(self, height):
        px = [[] for _ in self.updated_path]
        py = [[] for _ in self.updated_path]
        for i, p in enumerate(self.updated_path):
            px[i], py[i] = zip(*p)

        for j in range(len(px[0])):
            for i in range(len(self.updated_path)):
                obj_path = '/target[' + str(i) + ']'
                p = [px[i][j], py[i][j], height]
                sim.setObjectPosition(sim.getObject(obj_path), -1, p)
            time.sleep(0.1)


