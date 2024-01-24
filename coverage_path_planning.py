from shapely.geometry import Point  # For checking the points inside a Polygon
from shapely.geometry.polygon import Polygon  # For comparing the points in the shape of the Polygon
from shapely.geometry.polygon import LineString # For checking a line
import math

class Coverage_Path_Planning:  # Calculate the coverage paths
    def __init__(self, polygon, delta, vs):
        self.polygon = self.calc_nodes(polygon)
        # print('check', self.polygon)
        self.sorted_polygon = sorted(self.polygon)
        self.rounded_polygon = self.calc_nodes(polygon) + [polygon[0]]
        self.edges = self.calc_edges()
        self.pol_obj = Polygon(polygon)
        self.delta = delta
        self.vs = vs

    def calc_nodes(self, polygon): # Rounding the polygon
        count = len(polygon)
        pol_updated = polygon.copy()
        while count > 0:
            for i in range(len(polygon)-2):
                co_linear = self.check_colinear(polygon[i], polygon[i+1], polygon[i+2])
                if co_linear:
                    polygon[i+1] = 'f'
            co_linear = self.check_colinear(polygon[-1], polygon[0], polygon[1])
            if co_linear:
                polygon[0] = 'f'
            count = count - 1
        if 'f' in polygon:
            polygon.remove('f')
        return polygon

    def calc_edges(self):  # To calculate the edges in the polygon
        edges = []
        for i in range(len(self.rounded_polygon) - 1):
            edges.append([self.rounded_polygon[i], self.rounded_polygon[i+1]])
        return edges

    def calc_distance(self, v1, v2):  # distance between two points
        x1, y1 = v1[0], v1[1]
        x2, y2 = v2[0], v2[1]
        dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return dist

    def calc_midpoint(self, v1, v2):  # midpoint between two points
        x1, y1 = v1[0], v1[1]
        x2, y2 = v2[0], v2[1]
        x3, y3 = (x1+x2)/2, (y1+y2)/2
        return [x3,y3]

    def calc_intersect(self, point, line): # Calculate the nearest point from a point to a line
        x1, y1 = line[0]
        x2, y2 = line[1]
        x0, y0 = point
        if x1 == x2:
            return [x1, y0]
        elif y1 == y2:
            return [x0, y1]
        else:
            m1 = (y1 - y2) / (x1 - x2)
            m2 = - (1 / m1)
            c1 = y1 - m1 * x1
            c2 = y0 - m2 * x0
            x3 = (c2 - c1) / (m1 - m2)
            y3 = m1 * x3 + c1
            return [x3, y3]

    def calc_intersect_par(self, point, line):
        # To calculate the intersection point from a point to the line directly above, below, left or right
        # Input is an edge [[x1,y1],[x2,y2]] and a point [x,y], output is the intersecting points [x_x,y_x], [x_y,y_y]
        x1, y1 = line[0]
        x2, y2 = line[1]
        x0, y0 = point
        ints_points = []
        if x1 == x2:
            ints_points.append([x1,y0])
            return ints_points
        elif y1 == y2:
            ints_points.append([x0, y1])
            return ints_points
        else:
            x_x = x0
            y_x = ((x0 - x1) * (y1 - y2) / (x1 - x2)) + y1
            x_y = ((y0 - y1) * (x1 - x2) / (y1 - y2)) + x1
            y_y = y0
            ints_points.append([x_x, y_x])
            ints_points.append([x_y, y_y])
            return ints_points

    def reduced_points(self, v1, v2):  # Reduce the length of a line in one end
        x1, y1 = v1[0], v1[1]
        x2, y2 = v2[0], v2[1]
        # length = self.calc_distance(v1,v2)
        m2 = 1
        m1 = 9000
        x = (m1 * x2 + m2 * x1)/(m1+m2)
        y = (m1 * y2 + m2 * y1)/(m1+m2)
        return x, y

    def calc_linepoints(self, line):    # Generate several points inside a line
        x1, y1 = line[0]
        x2, y2 = line[1]
        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        m1 = self.delta
        m2 = length - m1
        linepoints = [[x1, y1]]
        while m1 < length:
            x3 = (m1 * x2 + m2 * x1) / (m1 + m2)
            y3 = (m1 * y2 + m2 * y1) / (m1 + m2)
            linepoints.append([x3, y3])
            m1 = m1 + self.delta
            m2 = length - m1
        linepoints.append([x2, y2])
        return linepoints

    def check_Edges(self, node):
        # To calculate the two edges which are the incoming and outgoing edges in a node
        # Input is a node [x,y] and the set of edges [[[x1,y1], [x2,y2]],[[x2,y2], [x3,y3]],...]
        # output is two edges connected by the node
        node_lines = []
        for line in self.edges:
            if (line[0] == node) or (line[1] == node):
                node_lines.append(line)
        sorted_lines = sorted(node_lines)
        return sorted_lines

    def check_colinear(self, v1, v2, v3):
        if (type(v1) == str) or (type(v2) == str) or (type(v3) == str):
            return False
        x1, y1 = v1
        x2, y2 = v2
        x3, y3 = v3
        val1 = (y2-y1)*(x3-x1)
        val2 = (x2-x1)*(y3-y1)
        if val1 == val2:
            return True
        else:
            return False

    def calc_ray_tracing(self): # Calculate the shortest distance from a node to any edge in a polygon
        ls = []
        # print("Polygon = ", self.polygon)
        for v in self.polygon:
            for edge in self.edges:
                colinear = self.check_colinear(v, edge[0], edge[1])
                if (not v in edge) and (not colinear):
                    ints_point = self.calc_intersect(v, edge) # Calculate intersection point
                    p_updated = self.reduced_points(v, ints_point) # Reduce the intersection point for checking
                    if self.pol_obj.contains(Point(p_updated)): # Check if the reduced point is in polygon
                        curr_dist = self.calc_distance(v, ints_point)
                        ls.append((curr_dist, ints_point, v, edge))
        if not ls:
            v1, v2, v3, v4 = self.polygon[0], self.polygon[1], self.polygon[2], self.polygon[-1]
            p1 = self.calc_midpoint(v1,v4)
            p2 = self.calc_midpoint(v2,v3)
            dist = self.calc_distance(p1,p2)
            ls.append((dist, p2, p1, self.edges[1]))
        min_dist, ints_point, node, edge = sorted(ls)[0]
        # print('min dist = ', min_dist, 'Edge = ', edge, 'Node = ', node, 'ins_point =', ints_point, 'Polygon = ', self.polygon)
        # print(sorted(ls))
        return min_dist, ints_point, node, edge

    def reduce_line(self, line): # Reduce the line by delta in both ends
        x1, y1 = line[0]
        x2, y2 = line[1]
        length = self.calc_distance(line[0], line[1])
        if length < self.delta:
            return line
        m1 = self.delta
        m2 = length - self.delta
        x3 = (m1*x2 + m2*x1)/(m1+m2)
        y3 = (m1*y2 + m2*y1)/(m1+m2)
        x4 = (m2*x2 + m1*x1)/(m1+m2)
        y4 = (m2*y2 + m1*y1)/(m1+m2)
        updated_line = [[x3,y3],[x4,y4]]
        return updated_line

    def calc_parallel_lines(self): # Calculate the parallel lines to cover a polygon
        # par = parallel to edge, perp = perpendicular to edge
        min_dist, ints_point, node, edge = self.calc_ray_tracing()
        xn, yn = node
        xi, yi = ints_point
        xl, yl, xh, yh = 0, 0, 0, 0
        if xi-xn == 0:
            xl = xn
            xh = xn
            yl, yh = yn - self.pol_obj.length, yi + self.pol_obj.length
        else:
            m_perp = (yi-yn)/(xi-xn) # Get the ratio for node and intersect
            c_perp = yn - m_perp * xn
            xl = min(xn, xi) - self.pol_obj.length
            xh = max(xn, xi) + self.pol_obj.length
            yl = m_perp * xl + c_perp
            yh = m_perp * xh + c_perp
        perp_line = [[xl,yl], [xh,yh]] # Line perpendicular to the edge
        # print('perp_line = ', perp_line)
        perp_line = self.calc_linepoints(perp_line) # Generating line points in the perpendicular line
        xe1, ye1 = edge[0]
        xe2, ye2 = edge[1]
        if xe1-xe2 == 0:
            m_par = 'inf'
        else:
            m_par = (ye2-ye1)/(xe2-xe1) # Get the ratio for parallel to edge
        c_perp = []
        parallel_lines = []  # List of parallel lines
        for p in perp_line: # every point in the perpendicular line
            x1, y1 = p[0], p[1]
            if type(m_par) == str: # m = infinity
                x2 = x1
                x3 = x1
                y2 = y1 + self.pol_obj.length
                y3 = y1 - self.pol_obj.length
            else:
                x2 = x1 + self.pol_obj.length
                x3 = x1 - self.pol_obj.length
                curr_c = y1 - m_par * x1
                c_perp.append(curr_c)
                y2 = m_par * x2 + curr_c
                y3 = m_par * x3 + curr_c
            par_line = [[x3,y3],[x2,y2]]
            shaped_line = LineString(par_line)
            intersect_line = list(self.pol_obj.intersection(shaped_line).coords)
            if intersect_line:
                parallel_lines.append(intersect_line) # Appending the parallel lines
        # print('parallel lines =', parallel_lines)
        # print('end point = ', parallel_lines[-1])
        return parallel_lines, edge

    def decomposed_areas(self):
        parallel_lines = self.calc_parallel_lines()
        par_lines = parallel_lines[0]
        edge = parallel_lines[1]
        flag = 0

        dist1 = self.calc_distance(edge[0], par_lines[0][0])
        dist2 = self.calc_distance(edge[0], par_lines[-1][0])
        if dist1 <= dist2:
            flag = 0
            par_lines.insert(0, sorted(edge))
        else:
            flag = 1
            par_lines.append(sorted(edge))
        print('parallel lines =', par_lines)
        print('end line = ', par_lines[-1])

        trapezoids = []
        for i in range(len(par_lines)-1):
            trapezoids.append([par_lines[i][0], par_lines[i][1], par_lines[i+1][1], par_lines[i+1][0]])
        print('trapezoids = ', trapezoids)
        area_set = []
        for trap in trapezoids:
            trap_obj = Polygon(trap)
            curr_area = trap_obj.area
            area_set.append(curr_area)
        print('total decomposed area without last cell =', sum(area_set))
        print('total polygon area =', self.pol_obj.area)
        rem_area = self.pol_obj.area - sum(area_set)
        if flag == 0:
            area_set.append(rem_area)
        else:
            area_set.insert(0, rem_area)
        print('decomposed area list = ', area_set)
        print('total decomposed area with last cell =', sum(area_set))
        print('number of parallel lines = ', len(par_lines))
        print('number of area cells = ', len(area_set))

    def check_Sides(self, node):
        # To detect the side of the edges from a node
        # Input is a node, output is a string containing the side
        edge1, edge2 = self.check_Edges(node)  # [[[x1,y1],[x2,y2]], [[x2,y2],[x3,y3]]]
        if edge1[0][0] != node[0]:
            side1 = edge1[0][0]
        else:
            side1 = edge1[1][0]
        if edge2[0][0] != node[0]:
            side2 = edge2[0][0]
        else:
            side2 = edge2[1][0]
        if side1 < node[0] and side2 < node[0]:
            # print("both edges are left to the node ",x)
            return 'bothleft'  # Both edges are left to the node
        elif side1 > node[0] and side2 > node[0]:
            # print("both edges are right to the node ",x)
            return 'bothright'  # Both edges are right to the node
        elif side1 > node[0] > side2:
            # print("first edge right and second edge left to the node ",x)
            return 'rightleft'  # First edge right and second edge left to the node
        elif side1 < node[0] < side2:
            # print("first edge left and second edge right to the node",x)
            return 'leftright'  # First edge left and second edge right to the node

    def calc_Cellareas(self):
        # To calculate the areas of the decomposed cells
        # Output is the set of areas of the cells
        arealines = self.decomposed_lines()
        cell_areas = []
        for i in range(0, len(arealines) - 1, 1):
            a = abs(arealines[i][0][1] - arealines[i][1][1])
            b = abs(arealines[i + 1][0][1] - arealines[i + 1][1][1])
            h = abs(arealines[i][0][0] - arealines[i + 1][0][0])
            area = 0.5 * (a + b) * h  # formula for calculating area of trapezoid
            cell_areas.append(area)
        return cell_areas

    def calc_UpperLowerEdges(self, node):
        # Calculate the upper and lower edges of a node
        # Input is a node, output is the pair of upper and lower edge
        upper, lower = [], []
        for e in self.edges:
            if (e[0][0] < node[0] < e[1][0]) or (e[0][0] > node[0] > e[1][0]):
                if (e[0][1] + e[1][1]) / 2 >= node[1]:
                    upper.append(e)
                else:
                    lower.append(e)
        # print("upper = ", upper)
        # print("lower = ", lower)
        minu, maxl = [], []
        ud, ld = [], []
        if len(upper) != 0:
            for e in upper:
                ints_point = self.calc_intersect_par(node, e)[0]
                ud.append(ints_point)
            ud = sorted(ud, key=lambda x: (x[1]))
            minu = ud[0]  # Minimum upper edge
        if len(lower) != 0:
            for e in lower:
                ints_point = self.calc_intersect_par(node, e)[0]
                ld.append(ints_point)
            ld = sorted(ld, key=lambda x: (x[1]))
            maxl = ld[-1]  # Maximum lower edge
        highlow = [minu, maxl]
        return highlow  # Return the pair of minimum upper edge and maximum lower edge

    def decomposed_lines(self):
        polygon_shaped = Polygon(self.polygon)
        lines = []
        for node in self.polygon:
            ul = self.calc_UpperLowerEdges(node)
            if len(ul) > 0:
                if (self.check_Sides(node) == 'bothleft') or (self.check_Sides(node) == 'bothright'):
                    if polygon_shaped.contains(Point(node[0], node[1])):
                        if len(ul[0]) != 0:
                            lines.append([node, ul[0]])
                        if len(ul[1]) != 0:
                            lines.append([node, ul[1]])
                elif (self.check_Sides(node) == 'rightleft') or (self.check_Sides(node) == 'leftright'):
                    if len(ul[0]) != 0:
                        lines.append([node, ul[0]])
                    if len(ul[1]) != 0:
                        lines.append([node, ul[1]])
        # lines.remove(lines[0])
        s, t = self.sorted_polygon[0], self.sorted_polygon[-1]
        if self.sorted_polygon[0][0] == self.sorted_polygon[1][0]:
            lines.insert(0, [s, self.sorted_polygon[1]])
        else:
            lines.insert(0, [s, s])
        if self.sorted_polygon[-1][0] == self.sorted_polygon[-2][0]:
            lines.append([self.sorted_polygon[-1], self.sorted_polygon[-2]])
        else:
            lines.append([t, t])
        # print("decomposed lines", sorted(lines))
        return sorted(lines)

    def sweep_coverage_path(self):
        parallel_lines = self.calc_parallel_lines()[0]
        reduced_lines = []
        for l in parallel_lines:
            curr_line = self.reduce_line(l)
            reduced_lines.append(curr_line)
        for i, l in enumerate(reduced_lines):
            if i%2==0:
                tmp = reduced_lines[i][1]
                reduced_lines[i][1] = reduced_lines[i][0]
                reduced_lines[i][0] = tmp
        for i, l in enumerate(reduced_lines):
            tmp = self.calc_linepoints(l)
            reduced_lines[i] = tmp
        # print(reduced_lines)
        sweep_path = []
        for l in reduced_lines:
            for i in l:
                sweep_path.append(list(i))
        coverage_path = sweep_path.copy()
        for pos in sweep_path:
            v = Point(pos)
            curr_dist = self.pol_obj.exterior.distance(v)
            if curr_dist < (self.delta/10):
                if pos in sweep_path:
                    coverage_path.remove(pos)
        # print('Length of path = ', len(coverage_path))
        return coverage_path

    def calc_u(self, path):
        u = []
        val = (2 * self.vs) / self.delta
        for i in range(0, len(path) - 2, 1):
            x, y = path[i]
            xn, yn = path[i + 1]
            xnn, ynn = path[i + 2]
            if x == xn and y == yn:
                u.append(-1)
            else:
                if yn > y:
                    u.append(val)
                elif yn < y:
                    u.append(-val)
                else:
                    if ynn > yn:
                        u.append(-val)
                    elif ynn < yn:
                        u.append(val)
                    else:
                        u.append(0)
        u.append(-1)
        u.append(-1)
        return u
