import pyqtgraph as pg
from PyQt6.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QFileDialog, QMenu
from PyQt6.QtCore import Qt
from pyqtgraph.exporters import ImageExporter
import time

class PlotWindow(QMainWindow):
    """
    An independent plot window that handles variables of a single unit.
    """
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setGeometry(100, 100, 1000, 800)
        
        self.data_streams = {}
        self.plot_curves = {}
        self.colors = ['#ff0000', '#00ff00', '#0000ff', '#ffff00', '#ff00ff', '#00ffff']
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QVBoxLayout(central_widget)
        
        self.setup_plot()
        
    def setup_plot(self):
        self.plot_item = pg.PlotItem()
        self.plot_widget = pg.PlotWidget(plotItem=self.plot_item)
        
        self.plot_item.showGrid(x=True, y=True)
        self.plot_item.setLabel('bottom', 'Data Point Index')
        self.plot_item.addLegend()
        
        self.plot_widget.setMenuEnabled(False)
        self.plot_widget.scene().contextMenuEvent = self.on_context_menu

        self.layout.addWidget(self.plot_widget)

    def on_context_menu(self, event):
        menu = QMenu()
        auto_range_action = menu.addAction("Auto-range")
        reset_action = menu.addAction("Reset and Start from Current Point")
        save_action = menu.addAction("Save as Image")

        action = menu.exec(event.screenPos())

        if action == auto_range_action:
            self.auto_range_plot()
        elif action == reset_action:
            self.clear_and_reset()
        elif action == save_action:
            self.save_snapshot()

    def auto_range_plot(self):
        self.plot_item.autoRange()

    def clear_and_reset(self):
        self.data_streams.clear()
        self.plot_curves.clear()
        
        self.layout.removeWidget(self.plot_widget)
        self.plot_widget.deleteLater()
        
        self.setup_plot()

    def update_plot(self, new_data):
        if not new_data:
            return

        for key, value in new_data.items():
            if key not in self.plot_curves:
                self.add_new_curve(key)
            
            self.data_streams[key].append(value)
            
            x = range(len(self.data_streams[key]))
            y = self.data_streams[key]
            self.plot_curves[key].setData(x, y)
        
        self.auto_range_plot()

    def add_new_curve(self, key):
        color_index = len(self.plot_curves) % len(self.colors)
        curve = pg.PlotDataItem(name=key, pen=pg.mkPen(self.colors[color_index], width=2), skipFiniteCheck=True)
        self.plot_item.addItem(curve)
        
        self.plot_curves[key] = curve
        self.data_streams[key] = []

    def save_snapshot(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Plot", "plot_snapshot.png", "PNG Files (*.png)")
        if filename:
            exporter = ImageExporter(self.plot_item)
            exporter.export(filename)
            return True
        return False

    def closeEvent(self, event):
        self.hide()
        event.ignore()