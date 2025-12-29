# Sensor: minimal checklist
Create sensor (camera) and attach to a link or place in world

For RGB/Depth/Seg you usually:

scene.step() (physics)

update/render

read tensors/images

Rule of thumb: don’t debug control with cameras first—get motion stable with viewer, then add sensors.