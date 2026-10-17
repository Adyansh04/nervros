---
name: arm-cannot-reach
description: Use when an arm move or a pick fails because MoveIt finds no plan or no arm pose reaches the object, or arm_check says an arm does not reach.
---

# The arm cannot reach

1. Call `arm_check` with `reach` for the object, once with each arm: which arm reaches, if either.
   If neither does, the robot stands too far or too low: walk to the object first, then check
   again.
2. Call `arm_check` with `preview`. "The goal is in a collision" or "the arm starts in a
   collision" means something in MoveIt's planning scene is in the way.
3. Call `scene` with `list`: an obstacle, or an object a hand holds, may be in the way. Voxels the
   sensors left of where the arm itself was block a reach down to a table: `scene` with
   `clear_octomap` clears them once the operator approves, then check again.
4. Call `arm_status`: the arm may be at an unexpected pose, or its hand may hold something.

Tell the operator what blocks the arm and the fix, in two sentences.
