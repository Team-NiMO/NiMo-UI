import sys
import rospkg
from PyQt5 import QtWidgets, uic

class Ui(QtWidgets.QMainWindow):
    def __init__(self, directory):
        super(Ui, self).__init__()
        uic.loadUi(directory + "/gui.ui", self)

        self.showFullScreen()

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('nimo_ui')

    app = QtWidgets.QApplication(sys.argv)
    window = Ui(package_path)
    app.exec_()