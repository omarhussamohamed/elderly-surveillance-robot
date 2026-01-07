# Maps Directory

This directory stores the maps generated during autonomous exploration.

## Creating a Map

After running the mapping mode:

```bash
roslaunch elderly_bot mapping.launch
```

Save the map when exploration is complete:

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/house_map
```

This will create two files:
- `house_map.yaml` - Map metadata (resolution, origin, etc.)
- `house_map.pgm` - Map image (occupancy grid)

## Using a Map

To use your saved map for navigation:

```bash
roslaunch elderly_bot navigation.launch map_file:=~/catkin_ws/src/elderly_bot/maps/house_map.yaml
```

## Map Format

Maps are stored in the standard ROS map format:
- **YAML file**: Contains metadata
- **PGM file**: Grayscale image where:
  - White (255) = Free space
  - Black (0) = Occupied
  - Gray (205) = Unknown

## Multiple Maps

You can create multiple maps for different environments:

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/living_room
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/office
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/warehouse
```

Then switch between them when launching navigation.


