import sys
import csv
import cv2
import yaml
import numpy as np

import rospy
import rospkg
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from PyQt5 import QtWidgets, uic, QtGui

class popup(QtWidgets.QWidget):
    def __init__(self, main_window, parent=None):
        super(popup, self).__init__(parent)

        self.main_window = main_window

        layout = QtWidgets.QFormLayout()
        self.label1 = QtWidgets.QLabel("Lattitude")
        self.le1 = QtWidgets.QLineEdit()
        layout.addRow(self.label1,self.le1)

        self.label2 = QtWidgets.QLabel("Longitutde")
        self.le2 = QtWidgets.QLineEdit()
        layout.addRow(self.label2,self.le2)

        self.button = QtWidgets.QPushButton("Enter")
        self.button.clicked.connect(self.enterButtonAction)
        layout.addRow(self.button)

        self.setLayout(layout)
        self.setWindowTitle("Enter End of Row Waypoint")

    def enterButtonAction(self):
        lat = self.le1.text()
        long = self.le2.text()

        try:
            orig_lat, orig_long = float(lat), float(long)
            lat, long = self.main_window.coords2Pixels(float(lat), float(long))
            if (lat > 700) or (lat < 0) or (long > 650) or (long < 0): raise Exception
        except:
            self.setWindowTitle("Invalid Final Waypoint")
            self.le1.clear()
            self.le2.clear()
            return
        
        self.main_window.final_lat = orig_lat
        self.main_window.final_long = orig_long
        self.close()

class Ui(QtWidgets.QMainWindow):
    def __init__(self, directory):
        super(Ui, self).__init__()
        self.ready = False
        uic.loadUi(directory + "/gui.ui", self)

        self.show()

        # Load config
        self.loadConfig()
        self.current_waypoint = 0
        self.lat = None
        self.long = None
        self.final_lat = None
        self.final_long = None

        # Setup ROS publishers, subscribers, and parameters
        rospy.init_node("nimo_ui")
        rospy.Subscriber("sampleVal", Float32, self.nitrateCallback)
        rospy.Subscriber("/odometry/filtered/", Odometry, self.baseUpdateCallback)
        rospy.set_param('/ui_stat', False)

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

        self.showFullScreen()

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

        self.package_name = config["file"]["package_name"]
        self.barn_field = config["file"]["barn_field"]
        self.stopping = config["file"]["stopping"]
        self.pruning = config["file"]["pruning"]

        self.ready = True

    def baseUpdateCallback(self, data):        
        self.lat, self.long = data.pose.pose.position.x, data.pose.pose.position.y

        lat, long = self.coords2Pixels(float(self.lat), float(self.long))
        if (lat > 700) or (lat < 0) or (long > 650) or (long < 0): return
        
        try:
            self.image[long:long+100,lat:lat+100,:] = self.amiga_img
        except:
            return
        self.updateMapImage()

    def nitrateCallback(self, data):
        if self.addButton.isEnabled():
            return
        elif self.current_waypoint > self.table.rowCount():
            return

        nitVal = data.data

        self.table.setItem(self.current_waypoint, 3, QtWidgets.QTableWidgetItem(str(nitVal)))
        self.current_waypoint += 1

    def coords2Pixels(self, long, lat):
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
        
        lat, long = self.coords2Pixels(float(self.table.item(row, 1).text()), float(self.table.item(row, 2).text()))
        self.image[long:long+100, lat-100:lat,:] = [255, 255, 255]
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
            orig_lat, orig_long = float(lat), float(long)
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
        self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(orig_lat)))
        self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(orig_long)))
        self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(str("")))

        btn = QtWidgets.QPushButton(self.table)
        btn.setText("X")
        btn.setStyleSheet("background-color:red")
        self.table.setCellWidget(row, 4, btn)
        self.table.cellWidget(row, 4).clicked.connect(lambda: self.deleteTableRowAction(row))

        self.image[long:long+100,lat-100:lat,:] = self.corn_img
        self.updateMapImage()

        self.lattidudeEdit.clear()
        self.longitudeEdit.clear()

    def startButtonAction(self):
        if self.table.rowCount() == 0:
            self.statusImage.setText("No waypoints specified")
            return
        
        if self.lat is None or self.long is None:
            self.statusImage.setText("Base location unknown")
            return
        
        if self.final_lat is None or self.final_long is None:
            self.ex = popup(self)
            self.ex.show()
            self.statusImage.setText("Enter end of row waypoint")
            return

        # rospack = rospkg.RosPack()
        try:
            # package_path = rospack.get_path(self.package_name)
            package_path = "/home/amiga/catkin_workspaces/nimo_ws/src/MPC_Amiga"
        except:
            self.statusImage.setText("Navigation package not found")
            return
        
        barn_field = open(package_path + "/" + self.barn_field, "w")
        stopping = open(package_path + "/" + self.stopping, "w")
        pruning = open(package_path + "/" + self.pruning, "w")

        # Write coordinates to barn_field
        barn_field.write(str(self.lat) + "," + str(self.long) + ",0.0,0.0,0.0,0.0\n")
        for row in range(self.table.rowCount()):
            barn_field.write(str(float(self.table.item(row, 1).text())) + "," + str(float(self.table.item(row, 2).text())) + ",0.0,0.0,0.0,0.0\n")
        barn_field.write(str(self.final_lat) + "," + str(self.final_long) + ",0.0,0.0,0.0,0.0\n")

        # Write coordinates to stopping
        for row in range(self.table.rowCount()):
            stopping.write(str(float(self.table.item(row, 1).text())) + "," + str(float(self.table.item(row, 2).text())) + ",0.0,0.0,0.0,0.0\n")
        stopping.write(str(self.final_lat) + "," + str(self.final_long) + ",0.0,0.0,0.0,0.0\n")

        # Write coordinates to pruning
        for i in range(2):
            pruning.write(str(self.final_lat) + "," + str(self.final_long) + ",0.0,0.0,0.0,0.0\n")

        # Set UI
        self.addButton.setDisabled(True)
        self.uploadCSVButton.setDisabled(True)
        self.saveCSVButton.setDisabled(False)

        row = self.table.rowCount()
        for idx in range(row):
            self.table.cellWidget(idx,4).setDisabled(True)

        self.statusImage.setText("Robot in motion.")
        self.startButton.setDisabled(True)

        # Update navigation parameter status
        rospy.set_param('/ui_stat', True)

    def uploadCSVButtonAction(self):
        fileName, ok = QtWidgets.QFileDialog.getOpenFileName(self, 'Select a CSV file:', 'C:\\', "CSV (*.csv)")
        if ok:
            validInputs = True

            row_count = 0
            with open(fileName, newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                next(reader, None) # Assume that there is a header for "Latitude, Longitude"
                for row in reader:
                    row_count += 1

            count = 0
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

                    # Set final waypoint in file as end point
                    if count == row_count - 1:
                        self.final_lat = float(row[0])
                        self.final_long = float(row[1])
                        break
                    
                    rowNum = self.table.rowCount()
                    self.table.insertRow(rowNum)
                    self.table.setItem(rowNum, 0, QtWidgets.QTableWidgetItem(str(rowNum+1)))
                    self.table.setItem(rowNum, 1, QtWidgets.QTableWidgetItem(str(np.round(float(row[0]), 4))))
                    self.table.setItem(rowNum, 2, QtWidgets.QTableWidgetItem(str(np.round(float(row[1]), 4))))
                    self.table.setItem(rowNum, 3, QtWidgets.QTableWidgetItem(str("")))

                    btn = QtWidgets.QPushButton(self.table)
                    btn.setText("X")
                    btn.setStyleSheet("background-color:red")
                    self.table.setCellWidget(rowNum, 4, btn)
                    self.table.cellWidget(rowNum, 4).clicked.connect(lambda state, n=rowNum: self.deleteTableRowAction(n))

                    self.image[long:long+100,lat-100:lat,:] = self.corn_img
                    self.updateMapImage()

                    count += 1
            
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