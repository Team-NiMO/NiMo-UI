import sys
import csv
# import rospkg
from PyQt5 import QtWidgets, uic

class Ui(QtWidgets.QMainWindow):
    def __init__(self, directory):
        super(Ui, self).__init__()
        uic.loadUi(directory + "/gui.ui", self)

        self.showFullScreen()

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

    def exitEvent(self):
        exit()

    def deleteTableRowAction(self, row):
        self.table.removeRow(row)
        for idx in range(self.table.rowCount()):
            self.table.setItem(idx, 0, QtWidgets.QTableWidgetItem(str(idx+1)))
        self.statusImage.setText("Removed coordinate.")
    
    def addButtonAction(self):
        lat = self.lattidudeEdit.text()
        long = self.longitudeEdit.text()

        try:
            float(lat), float(long)
            self.statusImage.setText("Valid coordinate input.")
        except:
            self.statusImage.setText("Invalid coordinate input.")
            self.lattidudeEdit.clear()
            self.longitudeEdit.clear()
            return
    
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(row+1)))
        self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(lat))
        self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(long))
        self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(str("")))

        btn = QtWidgets.QPushButton(self.table)
        btn.setText("X")
        btn.setStyleSheet("background-color:red")
        self.table.setCellWidget(row, 4, btn)
        self.table.cellWidget(row, 4).clicked.connect(lambda: self.deleteTableRowAction(row))

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

        print(self.coordinates)

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
                        float(row[0]), float(row[1])
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
    # rospack = rospkg.RosPack()
    # package_path = rospack.get_path('nimo_ui')

    app = QtWidgets.QApplication(sys.argv)
    # window = Ui(package_path)
    window = Ui("/home/sruthim/NiMo/NiMo-UI")
    app.exec_()