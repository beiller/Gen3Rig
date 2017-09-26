__author__ = 'bill'


rigify_finger_dof_x = 15.0
rigify_thumb_dof = 10.0

template = {
    'bones': {
        "tweak_spine" : [0.30, 0.30, None, None, True],
        "tweak_spine.001" : [0.23, 0.17, None, None, True],
        "tweak_spine.002" : [0.23, 0.16, None, None, True],
        "tweak_spine.003" : [0.12, 0.20, None, None, True],
        "neck" : [0.1, 0.1, None, None, True],
        "head" : [0.25, 0.25, None, None, True],
        "thigh_fk.L" : [0.11, 0.11],
        "thigh_fk.R" : [0.11, 0.11],
        "shin_fk.L" : [0.1, 0.1],
        "shin_fk.R" : [0.1, 0.1],
        "foot_fk.L" : [0.16, 0.22, 0.16],
        "foot_fk.R" : [0.16, 0.22, 0.16],
        "DEF-shoulder.L" : [0.06, 0.06, None, None, True],
        "DEF-shoulder.R" : [0.06, 0.06, None, None, True],
        "upper_arm_fk.L" : [0.081, 0.081, None, None, True],
        "upper_arm_fk.R" : [0.081, 0.081, None, None, True],
        "forearm_fk.L" : [0.081, 0.081],
        "forearm_fk.R" : [0.081, 0.081],
        "hand_fk.L" : [0.06, 0.045, 0.05],
        "hand_fk.R" : [0.06, 0.045, 0.05],
        # HAND BONES ----------------------------------------------------
        "f_index.01.R": [0.015, 0.015],
        "f_middle.01.R": [0.015, 0.015],
        "f_ring.01.R": [0.015, 0.015],
        "f_pinky.01.R": [0.011, 0.011],
        "thumb.01.R": [0.010, 0.010],

        "f_index.02.R": [0.015, 0.015],
        "f_middle.02.R": [0.015, 0.015],
        "f_ring.02.R": [0.015, 0.015],
        "f_pinky.02.R": [0.011, 0.011],
        "thumb.02.R": [0.015, 0.015],

        "f_index.03.R": [0.015, 0.015],
        "f_middle.03.R": [0.015, 0.015],
        "f_ring.03.R": [0.015, 0.015],
        "f_pinky.03.R": [0.011, 0.011],
        "thumb.03.R": [0.015, 0.015],

        "f_index.01.L": [0.015, 0.015],
        "f_middle.01.L": [0.015, 0.015],
        "f_ring.01.L": [0.015, 0.015],
        "f_pinky.01.L": [0.011, 0.011],
        "thumb.01.L": [0.010, 0.010],

        "f_index.02.L": [0.015, 0.015],
        "f_middle.02.L": [0.015, 0.015],
        "f_ring.02.L": [0.015, 0.015],
        "f_pinky.02.L": [0.011, 0.011],
        "thumb.02.L": [0.015, 0.015],

        "f_index.03.L": [0.015, 0.015],
        "f_middle.03.L": [0.015, 0.015],
        "f_ring.03.L": [0.015, 0.015],
        "f_pinky.03.L": [0.011, 0.011],
        "thumb.03.L": [0.015, 0.015],
    },
    'constraints': [
        ['tweak_spine', 'tweak_spine.001', [-32.5, 47.5, -15, 15, -15, 15, True]],
        ['tweak_spine.001', 'tweak_spine.002', [-25, 40, -20, 20, -24, 24, True]],
        #['abdomenUpper', 'chestLower', [-25, 35, -12, 12, -20, 20, True]],
        ['tweak_spine.002', 'tweak_spine.003', [-15, 15, -10, 10, -10, 10, True]],
        ['tweak_spine.003', 'neck', [-15, 30, -22, 22, -40, 40, True]],
        # ['neck', 'head', [-17, 12, -22, 22, -10, 10, True]],
        ['neck', 'head', [-27, 25, -22, 22, -20, 20, True]],
        ['tweak_spine.003', 'DEF-shoulder.L', [-90, 50, -10, 10, -50, 50, True]],
        ['tweak_spine.003', 'DEF-shoulder.R', [-90, 50, -10, 10, -50, 50, True]],
        ['DEF-shoulder.L', 'upper_arm_fk.L', [-110, 110, -30, 30, -151, 85, True]],
        ['DEF-shoulder.R', 'upper_arm_fk.R', [-110, 110, -30, 30, -151, 85, True]],
        ['upper_arm_fk.L', 'forearm_fk.L', [-135, 5, -30, 30, 0, 0, True]],
        ['upper_arm_fk.R', 'forearm_fk.R', [-135, 5, -30, 30, 0, 0, True]],
        ['forearm_fk.L', 'hand_fk.L', [-20, 20, -60, 60, -110, 110, True]],
        ['forearm_fk.R', 'hand_fk.R', [-20, 20, -60, 60, -110, 110, True]],
        ['tweak_spine', 'thigh_fk.L', [-47.5, 140, -10, 10, -50, 50, True]],
        ['tweak_spine', 'thigh_fk.R', [-47.5, 140, -10, 10, -50, 50, True]],
        ['thigh_fk.L', 'shin_fk.L', [-140, 3, -32, 32, 0, 0, True]],
        ['thigh_fk.R', 'shin_fk.R', [-140, 3, -32, 32, 0, 0, True]],
        ['shin_fk.L', 'foot_fk.L', [-75, 40, -30, 30, -30, 30, True]],
        ['shin_fk.R', 'foot_fk.R', [-75, 40, -30, 30, -30, 30, True]],

        #  DAMMIT HANDS....

        ['hand_fk.R', 'f_index.01.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.R', 'f_middle.01.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.R', 'f_ring.01.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.R', 'f_pinky.01.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.R', 'thumb.01.R', [-rigify_thumb_dof, rigify_thumb_dof, -10, 10, -10, 10, True]],
        ['hand_fk.L', 'f_index.01.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.L', 'f_middle.01.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.L', 'f_ring.01.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.L', 'f_pinky.01.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['hand_fk.L', 'thumb.01.L', [-rigify_thumb_dof, rigify_thumb_dof, -5, 5, -5, 5, True]],

        ['f_index.01.R', 'f_index.02.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['f_middle.01.R', 'f_middle.02.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['f_ring.01.R', 'f_ring.02.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['f_pinky.01.R', 'f_pinky.02.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['thumb.01.R', 'thumb.02.R', [-rigify_thumb_dof, rigify_thumb_dof, -5, 5, -5, 5, True]],
        ['f_index.01.L', 'f_index.02.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['f_middle.01.L', 'f_middle.02.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['f_ring.01.L', 'f_ring.02.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['f_pinky.01.L', 'f_pinky.02.L', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        ['thumb.01.L', 'thumb.02.L', [-rigify_thumb_dof, rigify_thumb_dof, -5, 5, -5, 5, True]],

        ['f_index.02.R', 'f_index.03.R', [-1, 1, -1, 1, -1, 1, True]],
        ['f_middle.02.R', 'f_middle.03.R', [-1, 1, -1, 1, -1, 1, True]],
        ['f_ring.02.R', 'f_ring.03.R', [-1, 1, -1, 1, -1, 1, True]],
        ['f_pinky.02.R', 'f_pinky.03.R', [-1, 1, -1, 1, -1, 1, True]],
        ['thumb.02.R', 'thumb.03.R', [-1, 1, -1, 1, -1, 1, True]],
        ['f_index.02.L', 'f_index.03.L', [-1, 1, -1, 1, -1, 1, True]],
        ['f_middle.02.L', 'f_middle.03.L', [-1, 1, -1, 1, -1, 1, True]],
        ['f_ring.02.L', 'f_ring.03.L', [-1, 1, -1, 1, -1, 1, True]],
        ['f_pinky.02.L', 'f_pinky.03.L', [-1, 1, -1, 1, -1, 1, True]],
        ['thumb.02.L', 'thumb.03.L', [-1, 1, -1, 1, -1, 1, True]],
    ],
    'stiffness_map': [
        [0.5, ["tweak_spine", "tweak_spine.001", "tweak_spine.002", "tweak_spine.003", "neck"]],
        [0.75, []],  # this is hands?
        [0.75, ["head", "neck"]],
        [0.25, ["tweak_spine.003", "DEF-shoulder.L", "DEF-shoulder.R"]],
    ],
    'minimize_twist': [

    ],
    'collision_group_ext': [
        'DEF-shoulder.L',
        'DEF-shoulder.R'
    ]
}