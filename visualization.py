import keyboard  # For using keyboard commands
import matplotlib.pyplot as plt  # For plotting the graph
import time  # For calculating runtime

class Visualize:
    # Class visualize for visual plotting of decomposed lines and motion
    def __init__(self, polygon, cell_lines, k_path, inputs):
        self.polygon = polygon + [polygon[0]]
        self.updated_path = self.equal_lists(k_path)
        self.updated_inputs = self.equal_lists(inputs)
        self.cell_lines = cell_lines

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

    def lines(self):
        # To visualize the decomposition of the polygon
        # Input is the polygon and decomposed lines, output is the graph showing the polygon and decomposed lines
        xs, ys = zip(*self.polygon)
        fig = plt.figure(1)
        # print(lines)
        ax = fig.add_subplot(111)
        ax.set_ylim(0, 10)
        ax.set_xlim(0, 16)
        for i in range(len(self.cell_lines)):
            plt.plot(xs, ys, color='tab:blue')  # draw the polygon
            xn, yn = zip(*self.cell_lines[i])
            plt.plot(xn, yn, color='red')  # draw the decomposition
            for i, j in zip(xn, yn):
                if j > 5:
                    ax.annotate((round(i, 2), round(j, 2)), xy=(i - 0.3, j + 0.2))
                else:
                    ax.annotate((round(i, 2), round(j, 2)), xy=(i + 0.2, j - 0.2))
        plt.show(block = True)

    def motion(self, ts):
        # To visualize the path for the coverage of multi-robots
        # Input is the polygon, a range and the coverage path, output is the graph showing the path
        start, goal = [], []
        start_time = time.time()
        xs, ys = zip(*self.polygon)
        plt.figure(2)
        px = [[] for _ in self.updated_path]
        py = [[] for _ in self.updated_path]
        qx = [[] for _ in self.updated_path]    # For drawing the coverage so far
        qy = [[] for _ in self.updated_path]    # For drawing the coverage so far
        for i in range(len(self.updated_path)):
            start.append(self.updated_path[i][0])
            goal.append(self.updated_path[i][-1])

        for i, p in enumerate(self.updated_path):
            px[i], py[i] = zip(*p)
            px[i], py[i] = list(px[i]), list(py[i])
            qx[i].append(start[i][0])
            qy[i].append(start[i][1])
        print("Starting simulation for the", len(px), "robots...")
        print("Press 'P' to pause, 'R' to resume and 'Esc' to exit simulation")
        for j in range(len(px[0])):
            plt.cla()
            plt.plot(xs, ys)
            # print('The k-inputs are,')
            for i in range(len(self.updated_path)):
                plt.axis('scaled')
                plt.plot(start[i][0], start[i][1], "-pr")  # Red pentagon is the starting location
                plt.plot(goal[i][0], goal[i][1], "-xb")  # Blue cross is the goal location

                plt.plot(px[i], py[i], "--", alpha = 0.5)  # Draw the k-paths
                # plt.plot(px[i][j], py[i][j], "-or")  # Red circles are the robots

                if j - 1 >= 0:
                    dx = round(px[i][j] - px[i][j-1], 2)
                    dy = round(py[i][j] - py[i][j-1], 2)
                    # print('dx = ', dx, 'dy=',dy)
                    # plt.arrow(px[i][j], py[i][j], dx/2, dy/2, width=0.1, head_width=0.3, head_length=0.3)
                    qx[i].append(px[i][j])
                    qy[i].append(py[i][j])
                    if dx == 0 and dy == 0:
                        plt.plot(px[i][j], py[i][j], "-og") # Check if end point is reached
                    else: # Draw the heading angle with arrow
                        plt.annotate("", xy = (px[i][j],py[i][j]), xytext=(px[i][j-1],py[i][j-1]),
                                 arrowprops=dict(arrowstyle="->", mutation_scale=20))
                    plt.plot(qx[i], qy[i])  # Draw the coverage so far
                if keyboard.is_pressed('Esc'):
                    print("Esc button is pressed, now exiting the program...")
                    raise SystemExit  # Program will terminate if Esc button is pressed
                # elif keyboard.is_pressed('P'):  # Press 'P' to pause and 'R' to resume
                #     print("Simulation is Paused. Press 'R' to resume")
                #     keyboard.wait('r')  # Program will wait until 'R' is pressed
                # u = self.updated_inputs[i][j]
                # print('The robot', i + 1, 'has input, u =', u)
            plt.pause(ts)
        end_time = time.time()
        time_taken = end_time - start_time
        print("Simulation Running Time = ", time_taken, "seconds")
        plt.show()