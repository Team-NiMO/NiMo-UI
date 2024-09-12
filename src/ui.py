import sys
import rospkg
from PyQt5 import QtWidgets, uic

class Ui(QtWidgets.QMainWindow):
    def __init__(self, directory):
        super(Ui, self).__init__()
        uic.loadUi(directory + "/gui.ui", self)

        self.showFullScreen()

        # Connect Buttons to Functions
        self.exitButton.clicked.connect(self.exitEvent)

        # Setup table
        self.horizontalHeader = self.table.horizontalHeader()
        self.horizontalHeader.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)

    def exitEvent(self):
        exit()

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('nimo_ui')

    app = QtWidgets.QApplication(sys.argv)
    window = Ui(package_path)
    app.exec_()