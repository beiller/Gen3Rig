__author__ = 'bill'


rigify_finger_dof_x = 15.0
rigify_thumb_dof = 10.0

template = {
    'bones': {
        "pelvis" : [0.30, 0.30, None, None, True],
        "abdomenLower" : [0.23, 0.17, None, None, True],
        "abdomenUpper" : [0.23, 0.16],
        "chestLower" : [0.28, 0.3, 0.13],
        "chestUpper" : [0.12, 0.20, 0.14],
        "neckLower" : [0.1, 0.1],
        "neckUpper" : [0.1, 0.1],
        "head" : [0.20, 0.20, 0.2],
        "lThighBend" : [0.11, 0.11],
        "rThighBend" : [0.11, 0.11],
        "lThighTwist" : [0.11, 0.11],
        "rThighTwist" : [0.11, 0.11],
        "lShin" : [0.10, 0.10, 0.36],
        "rShin" : [0.10, 0.10, 0.36],
        "lCollar" : [0.1, 0.1, 0.1, (0,0.035,0)],
        "rCollar" : [0.1, 0.1, 0.1, (0,0.035,0)],
        "lShldrBend" : [0.081, 0.081],
        "rShldrBend" : [0.081, 0.081],
        "lShldrTwist" : [0.081, 0.081],
        "rShldrTwist" : [0.081, 0.081],
        "lForearmBend" : [0.051, 0.051],
        "rForearmBend" : [0.051, 0.051],
        "lForearmTwist" : [0.041, 0.041],
        "rForearmTwist" : [0.041, 0.041],
        "lHand" : [0.10, 0.085, 0.04],
        "rHand" : [0.10, 0.085, 0.04],
        "lFoot" : [0.16, 0.16, 0.09],
        "rFoot" : [0.16, 0.16, 0.09],
        "lMetatarsals" : [0.16, 0.16, 0.09],
        "rMetatarsals" : [0.16, 0.16, 0.09],

    },
    'constraints': [
        [ 'pelvis', 'abdomenLower', [-32.5,47.5,-15,15,-15,15,True] ],
        [ 'abdomenLower', 'abdomenUpper', [-25,40,-20,20,-24,24,True] ],
        [ 'abdomenUpper', 'chestLower', [-25,35,-12,12,-20,20,True] ],
        [ 'chestLower', 'chestUpper', [-15,15,-10,10,-10,10,True] ],
        [ 'chestUpper', 'neckLower', [-15,30,-22,22,-40,40,True] ],
        [ 'neckLower', 'neckUpper', [-17,12,-22,22,-10,10,True] ],
        [ 'neckUpper', 'head', [-27,25,-22,22,-20,20,True] ],
        [ 'chestUpper', 'lCollar', [-26,17,-30,30,-50,10,True] ],
        [ 'chestUpper', 'rCollar', [-17,26,-30,30,-10,50,True] ],

        [ 'lCollar', 'lShldrBend', [-110,110,-0,0,-110,110,True] ],
        [ 'rCollar', 'rShldrBend', [-110,110,-0,0,-110,110,True] ],

        [ 'pelvis', 'rThighBend', [-170,30,-1,1,-90,90,True] ],
        [ 'pelvis', 'lThighBend', [-170,30,-1,1,-90,90,True] ],

        [ 'lThighBend', 'lThighTwist', [0,0,-90,90,0,0,True] ],
        [ 'rThighBend', 'rThighTwist', [0,0,-90,90,0,0,True] ],

        [ 'lShldrBend', 'lShldrTwist', [0,0,-80,80,0,0,True] ],
        [ 'rShldrBend', 'rShldrTwist', [0,0,-80,80,0,0,True] ],

        [ 'lForearmBend', 'lForearmTwist', [0,0,-80,90,0,0,True] ],
        [ 'rForearmBend', 'rForearmTwist', [0,0,-90,80,0,0,True] ],

        [ 'lThighTwist', 'lShin', [0,0,-12,12,-140,10,True] ],
        [ 'rThighTwist', 'rShin', [0,0,-12,12,-10,140,True] ],

        [ 'lShldrTwist', 'lForearmBend', [-135,20,0,0,0,0,True] ],
        [ 'rShldrTwist', 'rForearmBend', [-135,20,0,0,0,0,True] ],

        [ 'lForearmTwist', 'lHand', [-10,10,-30,30,-80,70,True] ],
        [ 'rForearmTwist', 'rHand', [-10,10,-30,30,-70,80,True] ],

        [ 'lShin', 'lFoot', [-75,40,-12,12,-10,10,True] ],
        [ 'rShin', 'rFoot', [-75,40,-12,12,-10,10,True] ],

        ['lFoot', 'lMetatarsals', [-15, 15, -15, 15, -15, 15, True]],
        ['rFoot', 'rMetatarsals', [-15, 15, -15, 15, -15, 15, True]],

    ],
    'stiffness_map': [
        [0.25, ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower"]],
        [0.75, []],  # this is hands?
        [0.75,  ["head","neckUpper","neckLower"]],
        [0.5, ["chestUpper","rCollar","lCollar"]]
    ],
    'minimize_twist': [
        [0.9, ["rShldrBend","rShldrTwist","lShldrBend","lShldrTwist"], 25],
        [0.9, ["rThighBend","rThighTwist","lThighBend","lThighTwist"], 25],
        [0.9, ["rForearmBend","rForearmTwist","lForearmBend","lForearmTwist"], 25]
    ],
    'collision_group_ext': [
        #'DEF-shoulder.L',
        #'DEF-shoulder.R'
    ]
}

sides = ['l', 'r']
finger_names = ['Index', 'Mid', 'Ring', 'Pinky', 'Thumb']
toe_names = ['BigToe', 'SmallToe1', 'SmallToe2', 'SmallToe3', 'SmallToe4']
hand_bone_name = 'Hand'
foot_bone_name = 'Metatarsals'
finger_dof = 80.0
toe_dof = 5.0

# Create finger and toe bones
for i in [1, 2]:
    for side in sides:
        for fname in finger_names:
            name = '{}{}{}'.format(side, fname, i)
            template['bones'][name] = [0.015, 0.015]
        for tname in toe_names:
            name = '{}{}'.format(side, tname)
            template['bones'][name] = [0.010, 0.010]

# Connect first toe bone to foot
for side in sides:
    for fname in toe_names:
        finger_name = '{}{}'.format(side, fname)
        hand_name = "{}{}".format(side, foot_bone_name)
        template['constraints'].append([
            hand_name, finger_name, [-2, 2, -2, 2, -toe_dof, toe_dof, True]
        ])

# Connect first finger bone to hand
for side in sides:
    for fname in finger_names:
        finger_name = '{}{}1'.format(side, fname)
        hand_name = "{}{}".format(side, hand_bone_name)
        # eg ['hand_fk.R', 'f_index.01.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],
        if side == 'r':
            template['constraints'].append([
                hand_name, finger_name, [-2, 2, -2, 2, -finger_dof, 1.0, True]
            ])
        else:
            template['constraints'].append([
                hand_name, finger_name, [-2, 2, -2, 2, -1.0, finger_dof, True]
            ])

# Connect second finger to first finger
for side in sides:
    for fname in finger_names:
        finger_name1 = '{}{}1'.format(side, fname)
        finger_name2 = '{}{}2'.format(side, fname)

        # eg ['hand_fk.R', 'f_index.01.R', [-rigify_finger_dof_x, rigify_finger_dof_x, -2, 2, -5, 5, True]],

        if side == 'r':
            template['constraints'].append([
                finger_name1, finger_name2, [-2, 2, -2, 2, -finger_dof, 1.0, True]
            ])
        else:
            template['constraints'].append([
                finger_name1, finger_name2, [-2, 2, -2, 2, -1.0, finger_dof, True]
            ])