#!/usr/bin/env python3
"""Bench-only world: the project warehouse world plus libgazebo_ros_state (/set_entity_state, /get_entity_state).

Usage: make_state_world.py <workspace> <out_world>
Used by the perturbation test to teleport the robot (delete/respawn was abandoned: after deletions the
camera stream did not follow the respawned robot).
"""
import sys
ws, out = sys.argv[1], sys.argv[2]
s = open(ws + '/src/warehouse_gazebo/worlds/small_warehouse.world').read()
e = s.index('>', s.index('<world')) + 1
s = s[:e] + '''
    <!-- diagnostics_phase4 bench only: entity state services for teleporting the robot -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>''' + s[e:]
open(out, 'w').write(s)
