# NiMo-UI
A user interface for the NiMo Autonomous Nitrate Monitoring project. This interface allows a user to provide sampling locations, recieve the nitrate readings at the sampling locations, and recieve information about the location of the Amiga Mobile Base

## Installation
First, clone the repository into the `src` folder of your ROS workspace and make the ui script executable.
```
git clone git@github.com:Team-NiMO/NiMo-UI.git
chmod +x NiMo-UI/src/ui.py
```

Install any required python packages as dependencies arise.

Then, update the [configuration file](config/default.yaml) if necessary. This file sets the coordinates of the map and the directories in which to write sampling locations.

## Use
To run the node, type the following command.

```
rosrun nimo_ui ui.py
```

This will launch the interface shown below which allows users to:
- Provide Sampling Locations
- Recieve Nitrate Readings
- Perceive the Amiga Location

<img src="https://github.com/Team-NiMO/NiMo-UI/blob/main/docs/ui.png" width="650">

### Provide Sampling Locations
The user can manually enter sampling locations using the lattitude and longitude fields then hitting add. In this case, before launching the user will be asked for an end of field waypoint at which the Amiga should stop. The user can also upload a CSV file containing the sampling locations. In this case, the last sampling location will be used as the end of field waypoint.

In either case the UI will reject coordinates out of range of the map (specified in [configuration file](config/default.yaml)) and provide a warning in the status box. Otherwise, the locations will appear in the table on the right and on the map on the left.

### Recieve Nitrate Readings
As the readings are taken, they will appear in the Nitrate Value column next to the corresponding sampling location in the table on the right.

### Perceive the Amiga Location
The map on the right will update the location of the amiga as it travels to each of the sampling locations.

## Acknowledgements
- Dr. Oliver Kroemer for his assistance and advice
- Dr. George Kantor for his assistance and advice
- The rest of [Team NiMo](https://github.com/Team-NiMO) for their suppport and feedback