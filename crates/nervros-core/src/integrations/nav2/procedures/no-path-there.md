---
name: no-path-there
description: Use when a walk fails because Nav2 finds no path, or nav_check says no path reaches a goal.
---

# No path reaches the goal

1. Call `nav_check` with `path` to the goal: no path, or one far longer than it should be.
2. Call `geofence` with `list`: a keep-out zone across a doorway or around the goal cuts it off.
   Say which zone; only the operator removes one.
3. Call `nav_check` with `cost` at the goal: a goal inside an obstacle or in unknown space cannot be
   reached; walk to a spot beside it instead.
4. A costmap that still holds something that has gone, such as a door now open or where the arm
   was, blocks the way: `nav_admin` with `clear_costmaps` (the operator approves), then `nav_check`
   again.
5. If the map and what `look` shows disagree, the robot may be lost on the map: say so, and offer
   `nav_admin` with `set_pose` or `relocalize`.

Tell the operator what blocks the way and what would open it, in two sentences.
