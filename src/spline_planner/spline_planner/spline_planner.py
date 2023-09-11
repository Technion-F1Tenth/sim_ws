from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt
class CubicSplinePlanner:
    def __init__(self, map, start, goal, M=10, N=1, INNER_POINTS=10):
        self.map = map
        self.start = start
        self.goal = goal
        self.M = M
        self.N = N
        self.INNER_POINTS = INNER_POINTS
        self.points = self.generate_random_points()

    def generate_random_points(self):
        path_length = self.goal[1] - self.start[1]
        points = []
        for i in range(self.M):
            point = []
            for j in range(self.N):
                point.append([np.random.randint(0,self.map.shape[0]-1), path_length//(self.N+1) * (j+1)])
            points.append(point)
        points = np.array(points)
        #print(points)
        # Sort points by x value
        for i in range(len(points)):
            points[i] = points[i][points[i][:,0].argsort()]
        return points

   
    def interpolate_splines(self):
        interpolated_splines = []
        costs_splines = []
        for possible_spline in self.points:
            possible_spline = np.vstack((self.start, possible_spline, self.goal))

            x, y = zip(*possible_spline)

            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.sqrt(dx**2 + dy**2)
            s = np.cumsum(ds)
            #print(s[-1])
            
            s = np.insert(s, 0, 0)

            x_spline = interpolate.CubicSpline(s, x)
            y_spline = interpolate.CubicSpline(s, y)

            new_s = np.linspace(min(s), max(s), self.INNER_POINTS)

            new_x = x_spline(new_s)
            new_y = y_spline(new_s)

            spline_points = np.column_stack((new_x, new_y))
            spline_clean = True
            for point in spline_points:
                if self.is_point_in_obstacle(point):
                    spline_clean = False
                    break
            if spline_clean:
                interpolated_splines.append(list(zip(new_x, new_y)))
                costs_splines.append(s[-1])
        if len(interpolated_splines) == 0:
            return [], [], []
        best_spline = interpolated_splines[np.argmin(costs_splines)]
        return interpolated_splines, costs_splines, best_spline

    def is_point_in_obstacle(self, point):
        x, y = point.astype(int)
        if x > self.map.shape[0] or y > self.map.shape[1]:
            return True
        if x < 0 or y < 0:
            return True
        try:
            return self.map[x][y] == 100
        except IndexError:
            return True

    def plot_splines(self):
        interpolated_splines, costs_splines,best_spline = self.interpolate_splines()
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='Greys', interpolation='nearest')
        plt.plot(self.start[1], self.start[0], 'ro')
        plt.plot(self.goal[1], self.goal[0], 'bo')
        if len(interpolated_splines) == 0:
            print("No spline found")
            return
        for idx, spline in enumerate(interpolated_splines):
            x, y = zip(*spline)
            plt.plot(y, x, 'r')
            plt.text(y[round(len(y) / 2)], x[round(len(x) / 2)], str(round(costs_splines[idx], 2)))
        
        x, y = zip(*best_spline)
        plt.plot(y, x, 'g')

        plt.show()