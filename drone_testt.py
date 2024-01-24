import time
from zmqRemoteApi import RemoteAPIClient

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)


def check_colinear(v1, v2, v3):
    if (type(v1) == str) or (type(v2) == str) or (type(v3) == str):
        return False
    x1, y1 = v1
    x2, y2 = v2
    x3, y3 = v3
    val1 = (y2 - y1) * (x3 - x1)
    val2 = (x2 - x1) * (y3 - y1)
    if val1 == val2:
        return True
    else:
        return False

def calc_nodes(polygon):  # Rounding the polygon
    count = len(polygon)
    pol_updated = polygon.copy()
    while count > 0:
        for i in range(len(polygon) - 2):
            co_linear = check_colinear(polygon[i], polygon[i + 1], polygon[i + 2])
            if co_linear:
                polygon[i + 1] = 'f'
        co_linear = check_colinear(polygon[-1], polygon[0], polygon[1])
        if co_linear:
            polygon[0] = 'f'
        count = count - 1
    if 'f' in polygon:
        polygon.remove('f')
    return polygon

def calc_edges(rounded_polygon):  # To calculate the edges in the polygon
    edges = []
    for i in range(len(rounded_polygon) - 1):
        edges.append([rounded_polygon[i], rounded_polygon[i+1]])
    return edges

polygon = [[10, 1], [14,1], [14, 4], [13, 8], [7, 9], [1, 5], [1, 2]]
rounded_polygon = calc_nodes(polygon) + [polygon[0]]
edges = calc_edges(rounded_polygon)

for l in edges:
    l[0].append(0)

print('edges = ', edges)


red = [1,0,0]
red_lineContainer=sim.addDrawingObject(sim.drawing_lines,2,0,-1,9999,red)

for l in edges:
    line = l[0] + l[1]
    print(line)
    sim.addDrawingObjectItem(red_lineContainer,line)
    # time.sleep(2)


# sim.removeDrawingObject(red_lineContainer)



objHandle = sim.getObject('./target')
# p = sim.getObjectPosition(objHandle, -1)
sim.startSimulation()
for p in polygon:
    p[2] = 0.5
    sim.setObjectPosition(objHandle, -1, p)
    print(p)
    time.sleep(5)
# sim.switchThread()  # resume in next simulation step


# client.setStepping(True)
sim.startSimulation()
# client.step()
