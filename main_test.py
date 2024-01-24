import numpy as np
from coverage_path_planning import *
from segmentation_using_dp import *
import polygenerator

use_coppeliaSim_simulator = False  # To use CoppeliaSim simulator

if use_coppeliaSim_simulator:
    from drone_simulation import *
else:
    from visualization import *

def rounding_polygon(polygon):
    x, y = zip(*polygon)
    x = np.array(x)
    y = np.array(y)
    cx = np.mean(x)
    cy = np.mean(y)
    a = np.arctan2(y - cy, x - cx)
    order = a.ravel().argsort()
    x = x[order]
    y = y[order]
    vertices = list(zip(x, y))
    return vertices

def k_polygons(k_segments, cell_lines):
    cell_lines = [[list(x) for x in ele] for ele in cell_lines]
    # print('cell lines =', cell_lines)

    trapezoids = []
    for i in range(len(cell_lines) - 1):
        trapezoids.append([cell_lines[i][0], cell_lines[i][1], cell_lines[i+1][0], cell_lines[i+1][1]])
    # print('trapezoids = ', trapezoids)

    polygons = [[trapezoids.pop(0) for _ in seg] for seg in k_segments]

    polygons = [[line for sub in ele for line in sub] for ele in polygons]

    updated_polygon = [[] for _ in polygons]

    for i in range(len(polygons)):
        for j in range(len(polygons[i])):
            if j%2 == 0:
                updated_polygon[i].append(list(polygons[i][j]))
        for j in range(len(polygons[i]),-1,-1):
            if j%2 == 1:
                updated_polygon[i].append(list(polygons[i][j]))

    res = [[] for _ in updated_polygon]
    for i in range(len(updated_polygon)):
        for j in range(len(updated_polygon[i])):
            if updated_polygon[i][j] not in res[i]:
                res[i].append(updated_polygon[i][j])

    res = [rounding_polygon(x) for x in res]


    # print('polygons = ', polygons)
    # print('updated polygon =', res)
    # for poly in res:
    #     xs, ys = zip(*poly)
    #     plt.plot(xs, ys)
    #     plt.show()
    # print('polygons = ', polygons)
    return res

# Driver code
def main():
    # time.sleep(5)
    # polygon = [(10, 1), (14, 5), (13, 6), (7, 6), (1, 4), (4, 1)]
    # polygon = [(7,1), (7, 6), (1, 4), (4, 1)]
    # polygon = [(8, 1), (14,1), (15, 4), (13, 8), (7, 9), (1, 6), (1, 3)]
    # polygon = [(12, 1), (14, 4), (13, 9), (7, 8), (1, 5), (4, 1)]
    # polygon = [ (10, 2), (14, 4), (13, 8), (5, 9), (1, 5)]
    # polygon = [(1, 1), (10, 1), (10,10), (1,10)]
    # polygon = [(1, 1), (10, 1), (5,10)]
    # polygon = [(1, 1), (10, 1), (15,5), (5,5)]

    # Experiments
    # polygon = [(10, 1), (14, 5), (13, 8), (7, 10), (5, 10), (1, 2)]
    # polygon = [(10, 1), (14, 4), (13, 6), (7, 9), (1, 5), (4, 1)]
    # polygon = [(12, 1), (14, 4), (13, 9), (7, 8), (1, 5), (4, 1)]
    # polygon = list(rounding_polygon(polygon))

    polygon = polygenerator.random_convex_polygon(50)
    poly = np.array(polygon) * 1250
    polygon = [list(p) for p in poly]
    # polygon = [[252.0495835475964, 599.9999999999999], [155.5322519931391, 572.8045993895558], [115.61682987328335, 550.5646984992824], [92.29358212694359, 534.6797724225403], [77.45084086649837, 521.7637299390573], [32.02846768359407, 413.6448759725332], [26.077577815870118, 390.41106192174124], [21.40893681820234, 371.308275263949], [9.073841788186808, 307.3203335238502], [0.0, 190.24870882482162], [15.217598443241407, 147.6670344105923], [21.798036281279195, 130.17004709705768], [145.74960984078513, 51.991501680748094], [158.68508397261994, 43.87269574139821], [272.83153837533524, 21.254416199403376], [381.07622419600534, 0.0], [459.17005618619544, 62.956496431474456], [501.9580190447926, 116.056655492348], [534.1735417305583, 160.59777628915373], [540.9489206575919, 171.6068449215256], [548.7241725932212, 200.5035855225768], [570.4430441110462, 281.62905578088305], [581.5583604534809, 328.76648233621444], [587.6504725002786, 361.6662059949688], [600.0, 437.2172553733617], [596.939271327094, 448.7383442340829], [586.8812781911604, 480.83721887031754], [422.79964198584116, 561.4871836438122], [349.4240205140844, 578.9941168211819], [342.08923652402865, 580.6295031356401]]
    print('\n\npolygon = ', polygon)

    delta = 20 # Coverage parameter
    k = 6  # Number of robots
    vs = 8 # Average Velocity
    height = 0.35

    cpp_object = Coverage_Path_Planning(polygon, delta, vs)
    cell_lines = cpp_object.decomposed_lines()
    # print('Decomposed lines = ', cell_lines)
    area_set = cpp_object.calc_Cellareas()
    segmented_areas = Segmentation(area_set, k).opt_segments
    print('segmented area set = ', segmented_areas)

    polygons = k_polygons(segmented_areas, cell_lines)
    # print('polygons =', polygons)

    k_paths = []
    for poly in polygons:
        print(poly)
        coverage_obj = Coverage_Path_Planning(list(poly), delta, vs)
        curr_path = coverage_obj.sweep_coverage_path()
        k_paths.append(curr_path)
    inputs = []
    for i in k_paths:
        ui = cpp_object.calc_u(i)
        inputs.append(ui)
    # for i in inputs:
        # print('inputs for path ', inputs.index(i), 'is = ', i)

    if use_coppeliaSim_simulator: # CoppeliaSim Simulator
        drone_simulator = Drone_simulator(polygon, k_paths)
        drone_simulator.draw_field(height)
        drone_simulator.start_simulation()
        drone_simulator.simulate_path(height)
        time.sleep(500)
        drone_simulator.stop_simulation()
    else: # Visualizing the simulation using Matplotlib
        ts = delta / vs  # Time step
        visualize = Visualize(polygon, cell_lines, k_paths, inputs)
        # visualize.lines()  # For visualizing the lines
        visualize.motion(0.1)  # For visualizing the motion


if __name__ == "__main__":
    main()