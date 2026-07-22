# mode_options.py — Manual mode options for the vertex resampler UI.

MODE_ITEMS = [
    ('BRIDGED_OPEN_LOOP_WITH_CORNERS', "Open Loop Bridge With Corners", ""),
    ('CORNER', "Corner", ""),
    ('BRIDGED_OPEN_LOOP', "Open Loop Bridge", ""),
    ('PIPE', "Pipe", ""),
    ('SOLID_HOLE_PUNCH', "Solid Hole Punch", ""),
    ('FACE_HOLE_PUNCH', "Face Hole Punch", ""),
    ('CLOSED_LOOP', "Closed Loop", ""),
    ('OPEN_LOOP', "Open Loop", ""),
    ('JUNCTION', "Junction", ""),
    ('ANCHORED', "Anchored", ""),
    ('KISSING', "Kissing", ""),
]


MODE_LABELS = {
    mode_id: label
    for mode_id, label, _description in MODE_ITEMS
}
