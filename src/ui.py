import sys
import csv
import cv2
import yaml
import numpy as np

import rospy
import rospkg
from std_msgs.msg import Float32

from PyQt5 import QtWidgets, uic, QtGui

# TODO: Amiga location datatype w/ Sauda + Shara
# TODO: Sampling coords datatype

class Ui(QtWidgets.QMainWindow):
    def __init__(self, directory):
        super(Ui, self).__init__()
        uic.loadUi(directory + "/gui.ui", self)

        self.showFullScreen()

        # Load config
        self.loadConfig()
        self.current_waypoint = 0

        # Setup ROS publishers and subscribers
        rospy.init_node("nimo_ui")
        rospy.Subscriber("sampleVal", Float32, self.nitrateCallback)
        # rospy.Subscriber("???", ???, self.baseUpdateCallback)
        # self.pos_pub = rospy.Publisher("???", ???, queue_size=10)

        # Connect Buttons to Functions
        self.exitButton.clicked.connect(self.exitEvent)
        self.addButton.clicked.connect(self.addButtonAction)
        self.startButton.clicked.connect(self.startButtonAction)
        self.uploadCSVButton.clicked.connect(self.uploadCSVButtonAction)
        self.saveCSVButton.clicked.connect(self.saveCSVButtonAction)

        # Setup table
        self.horizontalHeader = self.table.horizontalHeader()
        self.horizontalHeader.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)

        self.coordinates = []

        # Initialize images
        self.border = 50
        self.image = 255 * np.ones((650 + self.border * 2, 700 + self.border * 2, 3), np.uint8)

        self.corn_img = cv2.imread(directory+"/images/cornstalks.jpg")
        self.corn_img = cv2.cvtColor(self.corn_img, cv2.COLOR_RGB2BGR)
        self.corn_img = cv2.resize(self.corn_img, (0, 0), fx = 0.083, fy = 0.083)
        self.amiga_img = cv2.imread(directory+"/images/amiga.jpg")
        self.amiga_img = cv2.cvtColor(self.amiga_img, cv2.COLOR_RGB2BGR)
        self.amiga_img = cv2.resize(self.amiga_img, (0, 0), fx = 0.0605, fy = 0.0605)
        self.amiga_img = self.amiga_img[:100, :100, :]

        self.updateMapImage()

    def loadConfig(self):
        '''
        Load configuration from yaml file
        '''

        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('nimo_ui')
        config_path = self.package_path + '/config/default.yaml'
        with open(config_path) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)

        self.min_lat = config["map"]["min_lat"]
        self.max_lat = config["map"]["max_lat"]
        self.min_long = config["map"]["min_long"]
        self.max_long = config["map"]["max_long"]

    def baseUpdateCallback(self, data):
        lat, long = ... # Depends on data type

        lat, long = self.coords2Pixels(float(lat), float(long))
        if (lat > 700) or (lat < 0) or (long > 650) or (long < 0): return
        
        self.image[long:long+100,lat:lat+100,:] = self.amiga_img
        self.updateMapImage()

    def nitrateCallback(self, data):
        if self.addButton.isEnabled():
            return
        elif self.current_waypoint > self.table.rowCount():
            return

        nitVal = data.data

        self.table.setItem(self.current_waypoint, 3, QtWidgets.QTableWidgetItem(str(nitVal)))
        self.current_waypoint += 1

    def coords2Pixels(self, lat, long):
        lat_px = 700 * (lat - self.min_lat) / (self.max_lat - self.min_lat)
        long_px = 650 * (long - self.min_long) / (self.max_long - self.min_long)

        return int(lat_px), int(long_px)

    def updateMapImage(self):
        rows, cols, _ = self.image.shape
        croppedImage = self.image[self.border:rows - self.border, self.border:cols - self.border].copy()
        self.mapImage = QtGui.QImage(croppedImage, croppedImage.shape[1], croppedImage.shape[0], 3 * croppedImage.shape[1], QtGui.QImage.Format_RGB888)
        pixmap_current = QtGui.QPixmap.fromImage(self.mapImage)
        pixmap_image_current = QtGui.QPixmap(pixmap_current)
        self.map.setPixmap(pixmap_image_current)

    def exitEvent(self):
        exit()

    def deleteTableRowAction(self, row):
        # Bandaid for double delete thing
        try:
            self.table.item(row, 2).text()
        except:
            return
        
        self.image[int(self.table.item(row, 2).text()):int(self.table.item(row, 2).text())+100, int(self.table.item(row, 1).text()):int(self.table.item(row, 1).text())+100] = [255, 255, 255]
        self.updateMapImage()

        self.table.removeRow(row)
        for idx in range(self.table.rowCount()):
            self.table.setItem(idx, 0, QtWidgets.QTableWidgetItem(str(idx+1)))
            self.table.cellWidget(idx, 4).clicked.connect(lambda: self.deleteTableRowAction(idx))
        self.statusImage.setText("Removed coordinate.")
    
    def addButtonAction(self):
        lat = self.lattidudeEdit.text()
        long = self.longitudeEdit.text()

        try:
            lat, long = self.coords2Pixels(float(lat), float(long))
            if (lat > 700) or (lat < 0) or (long > 650) or (long < 0): raise Exception
            self.statusImage.setText("Valid coordinate input.")
        except:
            self.statusImage.setText("Invalid coordinate input.")
            self.lattidudeEdit.clear()
            self.longitudeEdit.clear()
            return
    
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(row+1)))
        self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(lat)))
        self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(long)))
        self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(str("")))

        btn = QtWidgets.QPushButton(self.table)
        btn.setText("X")
        btn.setStyleSheet("background-color:red")
        self.table.setCellWidget(row, 4, btn)
        self.table.cellWidget(row, 4).clicked.connect(lambda: self.deleteTableRowAction(row))

        self.image[long:long+100,lat:lat+100,:] = self.corn_img
        self.updateMapImage()

        self.lattidudeEdit.clear()
        self.longitudeEdit.clear()

    def startButtonAction(self):
        self.addButton.setDisabled(True)
        self.uploadCSVButton.setDisabled(True)
        self.saveCSVButton.setDisabled(False)
        
        row = self.table.rowCount()
        for idx in range(row):
            self.table.cellWidget(idx,4).setDisabled(True)

        self.statusImage.setText("Robot in motion.")
        self.startButton.setDisabled(True)

        columns = range(1, self.table.columnCount()-2)
        for row in range(self.table.rowCount()):
            self.coordinates.append([float(self.table.item(row, column).text()) for column in columns])

        # # Publish coordinates via ROS
        # self.coordinates
        # self.pos_pub(...)

    def uploadCSVButtonAction(self):
        fileName, ok = QtWidgets.QFileDialog.getOpenFileName(self, 'Select a CSV file:', 'C:\\', "CSV (*.csv)")
        if ok:
            validInputs = True
            with open(fileName, newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                next(reader, None) # Assume that there is a header for "Latitude, Longitude"
                for row in reader:
                    if len(row) != 2:
                        validInputs = False
                        continue

                    try:
                        lat, long = self.coords2Pixels(float(row[0]), float(row[1]))
                        if (lat > 700) or (lat < 0) or (long > 650) or (long < 0): raise Exception
                    except:
                        validInputs = False
                        continue
                    
                    rowNum = self.table.rowCount()
                    self.table.insertRow(rowNum)
                    self.table.setItem(rowNum, 0, QtWidgets.QTableWidgetItem(str(rowNum+1)))
                    self.table.setItem(rowNum, 1, QtWidgets.QTableWidgetItem(str(row[0])))
                    self.table.setItem(rowNum, 2, QtWidgets.QTableWidgetItem(str(row[1])))
                    self.table.setItem(rowNum, 3, QtWidgets.QTableWidgetItem(str("")))

                    btn = QtWidgets.QPushButton(self.table)
                    btn.setText("X")
                    btn.setStyleSheet("background-color:red")
                    self.table.setCellWidget(rowNum, 4, btn)
                    self.table.cellWidget(rowNum, 4).clicked.connect(lambda: self.deleteTableRowAction(rowNum))

                    self.image[long:long+100,lat:lat+100,:] = self.corn_img
                    self.updateMapImage()
            
            if not validInputs:
                self.statusImage.setText("Some invalid values were skipped.")
            else:
                self.statusImage.setText("Successful upload of .csv.")

    def saveCSVButtonAction(self):
        path, ok = QtWidgets.QFileDialog.getSaveFileName(self, 'Select a CSV file to save data:', 'C:\\', "CSV (*.csv)")
        if ok:
            columns = range(self.table.columnCount()-1)
            header = [self.table.horizontalHeaderItem(column).text() for column in columns]
            with open(path, 'w') as csvfile:
                writer = csv.writer(csvfile, delimiter = ",")
                writer.writerow(header)
                for row in range(self.table.rowCount()):
                    writer.writerow(self.table.item(row, column).text() for column in columns)

        self.statusImage.setText("Data save to .csv complete.")

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('nimo_ui')

    app = QtWidgets.QApplication(sys.argv)
    window = Ui(package_path)
    app.exec_()


    rospy.spin()