#!/usr/bin/env python3
"""Generate the warehouse world (SDF) and the station list (YAML).

Layout (metres, origin = warehouse centre, +x east, +y north):
    * interior 24 x 16, walls 0.2 thick
    * 3 rack rows (y = -3, 0, 3), 2 racks per row, 2 m cross-aisle at x = 0
    * 8 charge docks along the south wall (robot i spawns on dock i, facing north)
    * 4 pick stations on the west wall, 4 drop stations on the east wall
Everything is static, shadows are off, visuals are boxes
"""
import argparse
import math
import pathlib

WORLD_NAME = "warehouse"
HALF_X, HALF_Y = 12.0, 8.0
WALL_T, WALL_H = 0.2, 2.5

RACK_ROWS_Y = [-3.0, 0.0, 3.0]
RACK_CENTRES_X = [-3.5, 3.5]
RACK_L, RACK_D, RACK_H = 5.0, 0.9, 2.0

NUM_DOCKS = 8
DOCK_Y = -6.5
DOCK_X0, DOCK_DX = -8.75, 2.5

STATION_Y = [-4.5, -1.5, 1.5, 4.5]
PICK_X, DROP_X = -10.6, 10.6
TABLE_X = 11.6          # table centre |x|
PAD = 1.2               # square station pad edge

C_WALL = (0.80, 0.80, 0.82, 1)
C_RACK = (0.22, 0.27, 0.36, 1)
C_SLAB = (0.90, 0.55, 0.10, 1)
C_DOCK = (0.10, 0.70, 0.30, 1)
C_PICK = (0.15, 0.40, 0.85, 1)
C_DROP = (0.95, 0.50, 0.10, 1)
C_TABLE = (0.55, 0.58, 0.62, 1)
C_CHARGER = (0.95, 0.80, 0.10, 1)


def g(v):
    return f"{v:g}"


def vec(*v):
    return " ".join(g(x) for x in v)


def collision(size):
    return (f'      <collision name="collision"><geometry><box><size>{vec(*size)}</size>'
            f"</box></geometry></collision>\n")


def visual(name, size, rgba, offset=(0, 0, 0)):
    return (f'      <visual name="{name}"><pose>{vec(*offset)} 0 0 0</pose>'
            f"<geometry><box><size>{vec(*size)}</size></box></geometry>"
            f"<material><ambient>{vec(*rgba)}</ambient><diffuse>{vec(*rgba)}</diffuse></material></visual>\n")


def link(name, pos, body):
    return (f'    <link name="{name}">\n      <pose>{vec(*pos)} 0 0 0</pose>\n{body}    </link>\n')


def build():
    links, stations = [], {"docks": [], "pick_stations": [], "drop_stations": []}

    # --- walls -----------------------------------------------------------
    wx, wy = HALF_X + WALL_T / 2, HALF_Y + WALL_T / 2
    for name, pos, size in [
        ("wall_south", (0, -wy, WALL_H / 2), (2 * HALF_X + 2 * WALL_T, WALL_T, WALL_H)),
        ("wall_north", (0, wy, WALL_H / 2), (2 * HALF_X + 2 * WALL_T, WALL_T, WALL_H)),
        ("wall_west", (-wx, 0, WALL_H / 2), (WALL_T, 2 * HALF_Y, WALL_H)),
        ("wall_east", (wx, 0, WALL_H / 2), (WALL_T, 2 * HALF_Y, WALL_H)),
    ]:
        links.append(link(name, pos, collision(size) + visual("visual", size, C_WALL)))

    # --- racks (one collision box, decorative shelf slabs) ----------------
    for r, y in enumerate(RACK_ROWS_Y, start=1):
        for side, x in zip(("w", "e"), RACK_CENTRES_X):
            body = collision((RACK_L, RACK_D, RACK_H)) + visual("body", (RACK_L, RACK_D, RACK_H), C_RACK)
            for k, h in enumerate((0.25, 0.75, 1.25, 1.75)):
                body += visual(f"slab_{k}", (RACK_L + 0.04, RACK_D + 0.04, 0.05), C_SLAB,
                               (0, 0, h - RACK_H / 2))
            links.append(link(f"rack_r{r}_{side}", (x, y, RACK_H / 2), body))

    # --- charge docks (visual-only pad + charger marker) ------------------
    for i in range(NUM_DOCKS):
        x = DOCK_X0 + DOCK_DX * i
        n = i + 1
        links.append(link(f"dock_{n}_pad", (x, DOCK_Y, 0.006), visual("pad", (PAD, PAD, 0.01), C_DOCK)))
        links.append(link(f"dock_{n}_charger", (x, -HALF_Y + 0.45, 0.225),
                          visual("charger", (0.5, 0.12, 0.45), C_CHARGER)))
        stations["docks"].append((f"dock_{n}", x, DOCK_Y, math.pi / 2))

    # --- pick (west) and drop (east) stations -----------------------------
    for kind, px, yaw, col, key in (("pick", PICK_X, math.pi, C_PICK, "pick_stations"),
                                    ("drop", DROP_X, 0.0, C_DROP, "drop_stations")):
        sign = -1 if kind == "pick" else 1
        for j, y in enumerate(STATION_Y, start=1):
            links.append(link(f"{kind}_{j}_pad", (px, y, 0.006), visual("pad", (PAD, PAD, 0.01), col)))
            tsize = (0.5, 1.0, 0.8)
            links.append(link(f"{kind}_{j}_table", (sign * TABLE_X, y, 0.4),
                              collision(tsize) + visual("table", tsize, C_TABLE)))
            stations[key].append((f"{kind}_{j}", px, y, yaw))

    model = ('  <model name="warehouse_static">\n    <static>true</static>\n'
             + "".join(links) + "  </model>\n")

    floor = """  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
        <material><ambient>0.45 0.46 0.48 1</ambient><diffuse>0.45 0.46 0.48 1</diffuse></material>
      </visual>
    </link>
  </model>
"""

    sdf = f"""<?xml version="1.0" ?>
<!-- GENERATED by tools/generate_world.py - edit the script, not this file. -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">

    <physics name="fleet_physics" type="ignored">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.18 0.20 0.25 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.4 0.2 -1</direction>
    </light>

{floor}{model}  </world>
</sdf>
"""

    def rows(items):
        return "".join(f"  - {{id: {n}, x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.6f}}}\n"
                       for n, x, y, yaw in items)

    yaml = ("# GENERATED by tools/generate_world.py - do not edit by hand.\n"
            "# Poses are in the global 'map' frame (metres / radians).\n"
            f"world_name: {WORLD_NAME}\nframe_id: map\n"
            f"docks:\n{rows(stations['docks'])}"
            f"pick_stations:\n{rows(stations['pick_stations'])}"
            f"drop_stations:\n{rows(stations['drop_stations'])}")
    return sdf, yaml


if __name__ == "__main__":
    pkg = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-out", default=pkg / "worlds" / "warehouse.sdf")
    ap.add_argument("--stations-out", default=pkg / "config" / "stations.yaml")
    a = ap.parse_args()
    sdf_text, yaml_text = build()
    pathlib.Path(a.world_out).write_text(sdf_text)
    pathlib.Path(a.stations_out).write_text(yaml_text)
    print(f"wrote {a.world_out}\nwrote {a.stations_out}")
