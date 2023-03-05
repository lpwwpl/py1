import os
os.environ['QT_API'] = 'pyside'
os.environ["FORCE_CPU"] = 'True'
from matplotlib.pyplot import figure
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import kdata_lx

class ThreeDSurfaceGraphWindow(FigureCanvas):  # Class for 3D window
    def __init__(self):
        self.plot_colorbar = None
        self.plot_figure = figure()
        FigureCanvas.__init__(self, self.plot_figure)
        self.axes = self.plot_figure.gca(projection='3d')
        self.setWindowTitle("")  # sets Window title

    def draw_graph(self, x, y, z, color, acolor):  # Function for graph plotting
        self.axes.clear()
        if self.plot_colorbar is not None:  # avoids adding one more colorbar at each draw operation
            self.plot_colorbar.remove()
        # plots the 3D surface plot
        plot_stuff = self.axes.scatter(x, y, z, c="tab:blue", linewidth=0, antialiased=True)

        for i in range(kdata_lx.axisPoints.shape[0]):
            self.axes.scatter(kdata_lx.axisPoints[i][:, 0], kdata_lx.axisPoints[i][:, 1], kdata_lx.axisPoints[i][:, 2],
                              c="tab:orange")

        self.axes.tick_params(axis='x', colors='red')
        self.axes.tick_params(axis='y', colors='green')
        self.axes.tick_params(axis='z', colors='blue')

        self.axes.zaxis.set_major_locator(LinearLocator(10))
        self.axes.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        # Add a color bar which maps values to colors.
        self.plot_colorbar = self.plot_figure.colorbar(plot_stuff, shrink=0.5, aspect=5)
        # draw plot
        self.draw()