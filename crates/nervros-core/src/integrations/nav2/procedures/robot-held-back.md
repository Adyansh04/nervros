---
name: robot-held-back
description: Use when the robot stops short, crawls, or will not start a walk though nothing seems in the way: the collision monitor, a slow zone or a stale costmap may hold it back. Not for a robot that fell or a navigation stack that is down.
---

# Something holds the robot back

1. Call `nav_status`: whether a walk runs, its recoveries, the speed limit now and the collision
   monitor's state.
2. Call `collision_monitor` with `status`: which polygons are on, what each does (stop or slow
   down) and their shapes. A polygon that reaches something the robot has to pass close to, such
   as a doorway or the table it walks up to, explains a stop or a crawl there.
3. Call `geofence` with `list`: a slow zone over the way explains a crawl.
4. Call `nav_check` with `cost` where the robot stands and just ahead of it: a lethal or inscribed
   cost where nothing is means the costmap still holds something that has gone. `nav_admin` with
   `clear_costmaps` clears it, once the operator approves.
5. Tell the operator what holds the robot back, in two sentences, and offer the change that would
   free it: a polygon off or smaller, a slow zone removed. Never make such a change without their
   approval, and say what protection it takes away.
