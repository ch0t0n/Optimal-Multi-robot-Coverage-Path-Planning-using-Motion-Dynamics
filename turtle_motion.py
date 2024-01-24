import turtle

colours = ["green", "red", "blue", "purple", "black", "cyan", "olive", "darkblue", "lime", "darkred", "orchid",
           "darkcyan"]


class Turtle:
    # Class visualize for visual plotting of decomposed lines and motion
    def __init__(self, polygon, cell_lines, k_path, inputs, delta):
        self.polygon = polygon + [polygon[0]]
        self.updated_path = self.update_Path(k_path)
        self.updated_inputs = self.update_Path(inputs)
        self.cell_lines = cell_lines
        self.delta = delta

    def update_Path(self, k_path):
        # Update k-paths only for visualization
        mx = 0
        for path in k_path:
            if mx <= len(path):
                mx = len(path)
        for path in k_path:
            if mx != len(path):
                dif = mx - len(path)
                for i in range(dif):
                    path.append(path[-1])
        return k_path

    def check_colinear(self, v1, v2, v3):
        x1, y1 = v1
        x2, y2 = v2
        x3, y3 = v3
        val1 = (y2-y1)*(x3-x1)
        val2 = (x2-x1)*(y3-y1)
        if val1 == val2:
            return 'linear'
        else:
            return 'circular'

    def turtle_motion(self):
        init_bot = turtle.Turtle()
        turtle.screensize(canvwidth=5, canvheight=5)
        turtle.setworldcoordinates(0, 0, 20, 20)
        init_bot.speed(10)
        init_bot.penup()
        # turtle.goto(sorted(polygon)[0])
        # turtle.pendown()
        for i in self.polygon:
            init_bot.goto(i)
            init_bot.pendown()
        init_bot.hideturtle()

        start, goal = [], []
        xs, ys = zip(*self.polygon)
        for i in range(len(self.updated_path)):
            start.append(self.updated_path[i][0])
            goal.append(self.updated_path[i][-1])
        px = [[] for _ in self.updated_path]
        py = [[] for _ in self.updated_path]
        pt = [turtle.Turtle() for _ in self.updated_path]
        for i, p in enumerate(self.updated_path):
            px[i], py[i] = zip(*p)
        print("Starting simulation for the", len(self.updated_path), "robots...")
        for i in range(len(self.updated_path)):
            pt[i].penup()
            pt[i].color(colours[i])
            pt[i].goto(px[i][0], py[i][0])
            pt[i].pendown()
            pt[i].speed(0)

        for j in range(len(px[0])):
            for i in range(len(self.updated_path)):
                if j < (len(px[0])-1):
                    pt[i].setheading(pt[i].towards(px[i][j+1], py[i][j+1]))
                else:
                    pt[i].setheading(pt[i].towards(px[i][j], py[i][j]))

                if 0 < j < (len(px[0]) - 1):
                    v_curr = [px[i][j], py[i][j]]
                    v_prev = [px[i][j-1], py[i][j-1]]
                    v_next = [px[i][j+1], py[i][j+1]]
                    verdict = self.check_colinear(v_curr, v_prev, v_next)
                    if verdict == 'linear':
                        pt[i].goto(px[i][j], py[i][j])
                    else:
                        pt[i].circle(-self.delta/2, 180)
        turtle.done()
