# Randomized Road Generation and 3D Visualization

## Installation

First: Install **Python 3** and **Gazebo 8**.

http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

```sh
sudo apt-get install python3 gazebo8 libgazebo8-dev

pip install -r requirements.txt
```

Compile the keyframe plugin for gazebo:

```
cd keyframe_plugin
mkdir build
cd build
cmake ../
make

export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
```

## Usage
Generate a random scenario in CommonRoad XML from Template XML:

```
./road-generator.py presets/driving.xml -o driving-scenario.xml
```

Render CommonRoad XML for Gazbeo:

```
./gazebo-renderer.py driving-scenario.xml -o world
```

View in Gazbeo:
(Make sure the plugin's build folder is set as environment variable `GAZEBO_PLUGIN_PATH`)

```
cd world
gazebo world.sdf
```

Road generation and rendering can also be done in a single step:

```
./road-generator.py presets/driving.xml | ./gazebo-renderer.py -o world
```
