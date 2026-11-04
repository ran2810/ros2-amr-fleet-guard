#!/usr/bin/env python3
"""Generate the warehouse world (SDF) and the station list (YAML).

Layout (metres, origin = warehouse centre, +x east, +y north):
    * interior 24 x 16, walls 0.2 thick
    * 3 rack rows (y = -3, 0, 3), 2 racks per row, 2 m cross-aisle at x = 0
    * 8 charge docks along the south wall (robot i spawns on dock i, facing north)
    * 4 pick stations on the west wall, 4 drop stations on the east wall
Also writes the ground-truth occupancy map for Nav2 (warehouse_navigation/maps),
rasterised from the same collision boxes the simulator uses.
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
MAP_RES = 0.05          # occupancy map resolution [m/cell]

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
    solids = []  # (cx, cy, sx, sy): every box with collision, used to rasterise the nav map

    # --- walls -----------------------------------------------------------
    wx, wy = HALF_X + WALL_T / 2, HALF_Y + WALL_T / 2
    for name, pos, size in [
        ("wall_south", (0, -wy, WALL_H / 2), (2 * HALF_X + 2 * WALL_T, WALL_T, WALL_H)),
        ("wall_north", (0, wy, WALL_H / 2), (2 * HALF_X + 2 * WALL_T, WALL_T, WALL_H)),
        ("wall_west", (-wx, 0, WALL_H / 2), (WALL_T, 2 * HALF_Y, WALL_H)),
        ("wall_east", (wx, 0, WALL_H / 2), (WALL_T, 2 * HALF_Y, WALL_H)),
    ]:
        links.append(link(name, pos, collision(size) + visual("visual", size, C_WALL)))
        solids.append((pos[0], pos[1], size[0], size[1]))

    # --- racks (one collision box, decorative shelf slabs) ----------------
    for r, y in enumerate(RACK_ROWS_Y, start=1):
        for side, x in zip(("w", "e"), RACK_CENTRES_X):
            body = collision((RACK_L, RACK_D, RACK_H)) + visual("body", (RACK_L, RACK_D, RACK_H), C_RACK)
            for k, h in enumerate((0.25, 0.75, 1.25, 1.75)):
                body += visual(f"slab_{k}", (RACK_L + 0.04, RACK_D + 0.04, 0.05), C_SLAB,
                               (0, 0, h - RACK_H / 2))
            links.append(link(f"rack_r{r}_{side}", (x, y, RACK_H / 2), body))
            solids.append((x, y, RACK_L, RACK_D))

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
            solids.append((sign * TABLE_X, y, tsize[0], tsize[1]))
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
    return sdf, yaml, solids


def render_traffic_yaml():
    """Generate the fleet-level lane graph used by the traffic manager.

    Every task endpoint (dock / pick / drop) is represented explicitly in the graph. This avoids
    free-space "connector" shortcuts from the robot pose to a nearby lane node, which can otherwise
    cut diagonally across the warehouse and bypass fleet reservations.

    Rack aisles and outer corridor segments are capacity-one resources. The south corridor is split
    at every dock so robots leaving / returning to chargers also participate in traffic arbitration.
    """
    x_w, x_c, x_e = -7.5, 0.0, 7.5
    y_s, y_h1, y_h2, y_n = -4.5, -1.5, 1.5, 4.5

    nodes = {
        'w_s': (x_w, y_s), 'c_s': (x_c, y_s), 'e_s': (x_e, y_s),
        'w_h1': (x_w, y_h1), 'c_h1': (x_c, y_h1), 'e_h1': (x_e, y_h1),
        'w_h2': (x_w, y_h2), 'c_h2': (x_c, y_h2), 'e_h2': (x_e, y_h2),
        'w_n': (x_w, y_n), 'c_n': (x_c, y_n), 'e_n': (x_e, y_n),
    }

    # Explicit station endpoints. The traffic manager can now snap exactly to these nodes rather
    # than inventing an unreserved straight-line connection from a station to a nearby graph node.
    for i, y in enumerate(STATION_Y, start=1):
        nodes[f'pick_{i}'] = (PICK_X, y)
        nodes[f'drop_{i}'] = (DROP_X, y)

    # Docks plus their merge points on the south traffic lane.
    dock_xs = []
    for i in range(NUM_DOCKS):
        x = DOCK_X0 + DOCK_DX * i
        n = i + 1
        dock_xs.append(x)
        nodes[f'dock_{n}'] = (x, DOCK_Y)
        nodes[f'dock_lane_{n}'] = (x, y_s)

    edges = []

    def add(edge_id, src, dst, resource):
        edges.append((edge_id, src, dst, resource))

    # West / east outer vertical corridors.
    add('west_s_h1', 'w_s', 'w_h1', 'lane_west_s_h1')
    add('west_h1_h2', 'w_h1', 'w_h2', 'lane_west_h1_h2')
    add('west_h2_n', 'w_h2', 'w_n', 'lane_west_h2_n')
    add('east_s_h1', 'e_s', 'e_h1', 'lane_east_s_h1')
    add('east_h1_h2', 'e_h1', 'e_h2', 'lane_east_h1_h2')
    add('east_h2_n', 'e_h2', 'e_n', 'lane_east_h2_n')

    # North corridor.
    add('north_w_c', 'w_n', 'c_n', 'lane_north_w_c')
    add('north_c_e', 'c_n', 'e_n', 'lane_north_c_e')

    # Rack aisles. Both halves of one physical horizontal aisle intentionally share a resource,
    # so only one robot can occupy that complete aisle at a time.
    add('rack_h1_w_c', 'w_h1', 'c_h1', 'aisle_h_south')
    add('rack_h1_c_e', 'c_h1', 'e_h1', 'aisle_h_south')
    add('rack_h2_w_c', 'w_h2', 'c_h2', 'aisle_h_north')
    add('rack_h2_c_e', 'c_h2', 'e_h2', 'aisle_h_north')
    add('rack_v_s_h1', 'c_s', 'c_h1', 'aisle_v_center')
    add('rack_v_h1_h2', 'c_h1', 'c_h2', 'aisle_v_center')
    add('rack_v_h2_n', 'c_h2', 'c_n', 'aisle_v_center')

    # Pick/drop station access corridors on the west/east sides.
    level_nodes = [('w_s', 'e_s'), ('w_h1', 'e_h1'), ('w_h2', 'e_h2'), ('w_n', 'e_n')]
    for i, (west, east) in enumerate(level_nodes, start=1):
        add(f'pick_{i}_access', f'pick_{i}', west, f'pick_{i}_access')
        add(f'drop_{i}_access', east, f'drop_{i}', f'drop_{i}_access')

    # Dock access segments, each joining vertically to the south lane.
    for i in range(1, NUM_DOCKS + 1):
        add(f'dock_{i}_access', f'dock_{i}', f'dock_lane_{i}', f'dock_{i}_access')

    # South lane is explicitly segmented at each dock merge point and at the three structural
    # lane nodes. This means two robots cannot "skip" arbitration while leaving adjacent docks.
    south_points = [('w_s', x_w), ('c_s', x_c), ('e_s', x_e)]
    south_points += [(f'dock_lane_{i+1}', x) for i, x in enumerate(dock_xs)]
    south_points = sorted(south_points, key=lambda item: item[1])
    # De-duplicate any coincident x coordinates (none today, but keep generation robust).
    compact = []
    for name, x in south_points:
        if compact and abs(compact[-1][1] - x) < 1e-6:
            continue
        compact.append((name, x))
    for idx, ((a, _), (b, _)) in enumerate(zip(compact, compact[1:]), start=1):
        add(f'south_seg_{idx}', a, b, f'lane_south_{idx}')

    resources = {resource: [] for _, _, _, resource in edges}
    resources['aisle_h_south'] = ['aisle_v_center']
    resources['aisle_h_north'] = ['aisle_v_center']
    resources['aisle_v_center'] = ['aisle_h_south', 'aisle_h_north']

    out = [
        '# GENERATED by tools/generate_world.py - do not edit by hand.',
        '# All task endpoints are graph nodes; graph connectors are fleet-reserved.',
        'frame_id: map',
        'routing:',
        '  waypoint_spacing: 0.30',
        '  nearest_nodes: 1',
        '  anchor_snap_dist: 0.75',
        '  lock_route_nodes: true',
        'resources:',
    ]
    for resource, conflicts in resources.items():
        out.append(f'  {resource}:')
        out.append('    capacity: 1')
        out.append('    conflicts_with: [' + ', '.join(conflicts) + ']')
    out.append('nodes:')
    for name, (x, y) in nodes.items():
        out.append(f'  {name}: {{x: {x:.3f}, y: {y:.3f}}}')
    out.append('edges:')
    for edge_id, src, dst, resource in edges:
        out += [
            f'  - id: {edge_id}',
            f'    from: {src}',
            f'    to: {dst}',
            '    bidirectional: true',
            f'    resource: {resource}',
        ]
    return '\n'.join(out) + '\n'

def render_map(solids, name=WORLD_NAME):
    """Rasterise collision boxes into a trinary PGM + map_server YAML (origin = bottom-left)."""
    ox, oy = -(HALF_X + WALL_T), -(HALF_Y + WALL_T)
    w = int(round(2 * (HALF_X + WALL_T) / MAP_RES))
    h = int(round(2 * (HALF_Y + WALL_T) / MAP_RES))
    img = bytearray(b"\xfe" * (w * h))                      # 254 = free
    for cx, cy, sx, sy in solids:
        c0 = max(0, math.floor((cx - sx / 2 - ox) / MAP_RES))
        c1 = min(w, math.ceil((cx + sx / 2 - ox) / MAP_RES))
        r0 = max(0, h - math.ceil((cy + sy / 2 - oy) / MAP_RES))   # image row 0 is the top (max y)
        r1 = min(h, h - math.floor((cy - sy / 2 - oy) / MAP_RES))
        for r in range(r0, r1):
            img[r * w + c0:r * w + c1] = b"\x00" * (c1 - c0)       # 0 = occupied
    pgm = f"P5\n# GENERATED by tools/generate_world.py\n{w} {h}\n255\n".encode() + bytes(img)
    yml = ("# GENERATED by tools/generate_world.py - do not edit by hand.\n"
           f"image: {name}.pgm\nmode: trinary\nresolution: {MAP_RES}\n"
           f"origin: [{ox:g}, {oy:g}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return pgm, yml


if __name__ == "__main__":
    pkg = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-out", default=pkg / "worlds" / "warehouse.sdf")
    ap.add_argument("--stations-out", default=pkg / "config" / "stations.yaml")
    ap.add_argument("--traffic-out", default=pkg / "config" / "traffic.yaml")
    ap.add_argument("--map-dir", default=pkg.parent / "warehouse_navigation" / "maps",
                    help="where to write warehouse.pgm / warehouse.yaml (skipped if the dir is missing)")
    a = ap.parse_args()
    sdf_text, yaml_text, solids = build()
    pathlib.Path(a.world_out).write_text(sdf_text)
    pathlib.Path(a.stations_out).write_text(yaml_text)
    pathlib.Path(a.traffic_out).write_text(render_traffic_yaml())
    print(f"wrote {a.world_out}\nwrote {a.stations_out}\nwrote {a.traffic_out}")
    map_dir = pathlib.Path(a.map_dir)
    if map_dir.is_dir():
        pgm, yml = render_map(solids)
        (map_dir / f"{WORLD_NAME}.pgm").write_bytes(pgm)
        (map_dir / f"{WORLD_NAME}.yaml").write_text(yml)
        # Planning map now contains the real rack geometry. Fleet-level capacity-one aisle rules are
        # enforced by traffic_manager.py, so the gaps between racks must remain physically routable.
        pgm_nav, yml_nav = render_map(solids, name=f"{WORLD_NAME}_nav")
        (map_dir / f"{WORLD_NAME}_nav.pgm").write_bytes(pgm_nav)
        (map_dir / f"{WORLD_NAME}_nav.yaml").write_text(yml_nav)
        print(f"wrote {map_dir}/{WORLD_NAME}.pgm/.yaml and {WORLD_NAME}_nav.pgm/.yaml")
