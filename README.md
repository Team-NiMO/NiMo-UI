# NiMo-UI
A user interface for the NiMo Autonomous Nitrate Monitoring project. This interface allows a user to provide sampling locations, recieve the nitrate readings at the sampling locations, and recieve information about the locaiton of the Amiga Mobile Base

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

## Acknowledgements
- Dr. Oliver Kroemer for his assistance and advice
- Dr. George Kantor for his assistance and advice
- The rest of [Team NiMo](https://github.com/Team-NiMO) for their suppport and feedback