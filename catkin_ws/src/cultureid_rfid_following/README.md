First execution for new environment/map: launch

```roslaunch cultureid_rfid_following create_feasible_goals.launch```

with the correct setting of `room`. The map of the environment and its .yaml
file should have been placed in

```
tuw_multi_robot/tuw_voronoi_graph/cfg/maps/$(arg room)
```

with naming convention similar to the other maps and .yaml files. The graph
nodes' structure can be adjusted by tuning parameters in

```tuw_multi_robot/tuw_voronoi_graph/cfg/graph/$(arg room)/graph.yaml```

IMPORTANT: MAKE SURE THAT RELIEF_DEVEL/AVANTI_LIVE.LAUNCHER IS EXECUTED WITH
```gp:=globalplanner```.
