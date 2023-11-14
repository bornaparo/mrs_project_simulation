# Controlling a Swarm of Robots in a Simulator Stage using Reynolds Rules

### Multi-Robot systems course project

### Faculty of Electrical Engineering and Computing in Zagreb

### 2023./2024.

## [Download Docker simulator](https://github.com/larics/mrs_course)

## Start Docker Container

In one terminal copy/paste :
```sh
$ sudo docker start -i mrs_project_2
```

I other terminal copy/paste
```sh
$ sudo docker exec -it mrs_project_2 bash
```

your terminal prompt should change from
```bash
<your username>@<your hostname>
```
to
```bash
developer@<your hostname>
```
This signals that you are currently "inside" the container.

## Clone and build the project

- Clone this project in: `~/catkin_ws/src`

```sh
$ git clone https://github.com/bornaparo/mrs_project_simulation.git
```

- Build: `$: ~/catkin_ws/`
```sh
$ catkin build
```

## Lunch the maps

- Copy files from : `/home/developer/catkin_ws/src/mrs_project_simulation/launch_for_sphero_simulation`

- to : `/home/developer/catkin_ws/src/sphero_simulation/sphero_stage/launch`

#### Note : you must override original `start.py` from `sphero_stage/launch`

### If everything is set up correctly you can launch and try various maps:

```sh
$ roslaunch mrs_project_simulation simple_maze_map_launch.launch
```
```sh
$ roslaunch mrs_project_simulation empty_map_launch.launch
```
```sh
$  roslaunch mrs_project_simulation hard_maze_map_launch.launch 
```
```sh
$ roslaunch mrs_project_simulation test_map_launch.launch
```
