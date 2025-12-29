Reuse-First Checklist (read this before modeling anything)

Before you write a URDF, create meshes, or tune PD gains, check the following in order.

1. Can I reuse an existing robot model?

☐ SAPIEN built-in robots

☐ ManiSkill built-in robots (already tuned and tested)

☐ Official robot description from the manufacturer (URDF / CAD-derived)

☐ Community-maintained robot descriptions

If yes → use it directly.
If no → only then model the robot yourself.

Reusing a robot saves weeks of debugging physics and control.

2. Can I reuse an existing object model?

☐ Public object datasets with clean meshes and collisions

☐ Pre-scaled assets (meters, not arbitrary units)

☐ Objects already used in benchmarks or examples

Examples:

https://sapien.ucsd.edu/browse

If yes → reuse and move on.
If no → model only what your task truly needs.

3. Can I reuse an existing scene layout?

☐ Tabletop / shelf / bin layouts already defined

☐ Known stable ground and support surfaces

☐ Fixed camera placements that already work

If yes → reuse the scene template and vary only goals.

4. Can I reuse a control interface?

☐ Joint delta-position + PD

☐ End-effector delta pose + IK

☐ Existing gripper open/close abstraction

If yes → do not invent a new action space.

Reusing control interfaces makes tasks interchangeable.

5. Can I reuse success criteria or metrics?

☐ Distance thresholds

☐ Grasp detection heuristics

☐ “Hold for K steps” logic

☐ Placement-inside-region checks

If yes → compose them instead of redefining them.

6. What is the minimum I must model myself?

Only model:

☐ what is unique to your robot

☐ what is unique to your object

☐ what is unique to your task

Everything else should be reused.

One-line rule

If you can reuse it, don’t model it.
If you must model it, keep it minimal.

This mindset is how SAPIEN-based frameworks (e.g. ManiSkill) scale from one robot to many tasks without collapsing under complexity.