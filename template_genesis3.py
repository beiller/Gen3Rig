__author__ = 'bill'

template = {
    'bones': {
        "pelvis" : [0.30, 0.30, None, (0.0, -0.16, -0.02), True],
        "abdomenLower" : [0.23, 0.17, None, None, True],
        "abdomenUpper" : [0.23, 0.16],
        "chestLower" : [0.28, 0.3, 0.13],
        "chestUpper" : [0.12, 0.20, 0.14],
        "neckLower" : [0.1, 0.1],
        "neckUpper" : [0.1, 0.1],
        "head" : [0.30, 0.30, 0.2],
        "ThighBend.L" : [0.11, 0.11, None, (0, 0.07, 0) ],
        "ThighBend.R" : [0.11, 0.11, None, (0, 0.07, 0) ],
        "ThighTwist.L" : [0.11, 0.11],
        "ThighTwist.R" : [0.11, 0.11],
        "Shin.L" : [0.10, 0.10, 0.36],
        "Shin.R" : [0.10, 0.10, 0.36],
        "Collar.L" : [0.1, 0.1, 0.1, (0,0.035,0)],
        "Collar.R" : [0.1, 0.1, 0.1, (0,0.035,0)],
        "ShldrBend.L" : [0.081, 0.081],
        "ShldrBend.R" : [0.081, 0.081],
        "ShldrTwist.L" : [0.081, 0.081],
        "ShldrTwist.R" : [0.081, 0.081],
        "ForearmBend.L" : [0.051, 0.051],
        "ForearmBend.R" : [0.051, 0.051],
        "ForearmTwist.L" : [0.041, 0.041],
        "ForearmTwist.R" : [0.041, 0.041],
        "Hand.L" : [0.10, 0.085, 0.04, ( 0.030, -0.02, 0)],
        "Hand.R" : [0.10, 0.085, 0.04, (-0.030, -0.02, 0)],
        "Foot.L" : [0.16, 0.22, 0.09, (0, -0.061, 0.055)],
        "Foot.R" : [0.16, 0.22, 0.09, (0, -0.061, 0.055)],
        #HAND BONES ----------------------------------------------------
        "Index1.R" : [0.015, 0.015, None, (0, 0.012, 0) ],
        "Mid1.R" : [0.015, 0.015, None, (0, 0.012, 0) ],
        "Ring1.R" : [0.015, 0.015, None, (0, 0.012, 0) ],
        "Pinky1.R" : [0.011, 0.011, None, (0, 0.012, 0) ],
        "Thumb1.R" : [0.015, 0.015, None, (0, 0.012, 0) ],

        "Index1.L" : [0.015, 0.015, None, (0, 0.012, 0) ],
        "Mid1.L" : [0.015, 0.015, None, (0, 0.012, 0) ],
        "Ring1.L" : [0.015, 0.015, None, (0, 0.012, 0) ],
        "Pinky1.L" : [0.011, 0.011, None, (0, 0.012, 0) ],
        "Thumb1.L" : [0.015, 0.015, None, (0, 0.022, 0) ],

        "Index2.R" : [0.015, 0.015, 0.045, (0, 0.012, 0) ],
        "Mid2.R" : [0.015, 0.015, 0.045, (0, 0.012, 0) ],
        "Ring2.R" : [0.015, 0.015, 0.045, (0, 0.012, 0) ],
        "Pinky2.R" : [0.011, 0.011, 0.03, (0, 0.012, 0) ],
        "Thumb2.R" : [0.015, 0.015, 0.08, (0, 0.022, 0) ],

        "Index2.L" : [0.015, 0.015, 0.045, (0, 0.012, 0) ],
        "Mid2.L" : [0.015, 0.015, 0.045, (0, 0.012, 0) ],
        "Ring2.L" : [0.015, 0.015, 0.045, (0, 0.012, 0) ],
        "Pinky2.L" : [0.011, 0.011, 0.03, (0, 0.012, 0) ],
        "Thumb2.L" : [0.015, 0.015, 0.08, (0, 0.012, 0) ]
    },
    'constraints': [
        [ 'pelvis', 'abdomenLower', [-32.5,47.5,-15,15,-15,15,True] ],
        [ 'abdomenLower', 'abdomenUpper', [-25,40,-20,20,-24,24,True] ],
        [ 'abdomenUpper', 'chestLower', [-25,35,-12,12,-20,20,True] ],
        [ 'chestLower', 'chestUpper', [-15,15,-10,10,-10,10,True] ],
        [ 'chestUpper', 'neckLower', [-15,30,-22,22,-40,40,True] ],
        [ 'neckLower', 'neckUpper', [-17,12,-22,22,-10,10,True] ],
        [ 'neckUpper', 'head', [-27,25,-22,22,-20,20,True] ],
        [ 'chestUpper', 'Collar.L', [-26,17,-30,30,-50,10,True] ],
        [ 'chestUpper', 'Collar.R', [-17,26,-30,30,-10,50,True] ],
        [ 'Collar.L', 'ShldrBend.L', [-40,110,-0,0,-35,85,True] ],
        [ 'Collar.R', 'ShldrBend.R', [-110,40,-0,0,-85,35,True] ],
        [ 'pelvis', 'ThighBend.L', [-47.5,127.5,-0,0,-25,85,True] ],
        [ 'pelvis', 'ThighBend.R', [-47.5,127.5,-0,0,-85,25,True] ],
        [ 'ThighBend.L', 'ThighTwist.L', [0,0,-75,75,0,0,True] ],
        [ 'ThighBend.R', 'ThighTwist.R', [0,0,-75,75,0,0,True] ],
        [ 'ShldrBend.L', 'ShldrTwist.L', [0,0,-80,95,0,0,True] ],
        [ 'ShldrBend.R', 'ShldrTwist.R', [0,0,-95,80,0,0,True] ],
        [ 'ForearmBend.L', 'ForearmTwist.L', [0,0,-80,90,0,0,True] ],
        [ 'ForearmBend.R', 'ForearmTwist.R', [0,0,-90,80,0,0,True] ],
        [ 'ThighTwist.R', 'Shin.R', [-140,10,-12,12,0,0,True] ],
        [ 'ThighTwist.L', 'Shin.L', [-140,10,-12,12,0,0,True] ],
        [ 'ShldrTwist.R', 'ForearmBend.R', [-135,20,0,0,0,0,True] ],
        [ 'ShldrTwist.L', 'ForearmBend.L', [-135,20,0,0,0,0,True] ],
        [ 'ForearmTwist.L', 'Hand.L', [-10,10,-30,30,-80,70,True] ],
        [ 'ForearmTwist.R', 'Hand.R', [-10,10,-30,30,-70,80,True] ],
        [ 'Shin.L', 'Foot.L', [-75,40,-12,12,-10,10,True] ],
        [ 'Shin.R', 'Foot.R', [-75,40,-12,12,-10,10,True] ],
        #HAND BONES---------------------------------------------------------------
        [ 'Hand.R', 'Index1.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Hand.R', 'Mid1.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Hand.R', 'Ring1.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Hand.R', 'Pinky1.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Hand.R', 'Thumb1.R', [-5,5,-30,30,-30,30,True] ],
        [ 'Hand.L', 'Index1.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Hand.L', 'Mid1.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Hand.L', 'Ring1.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Hand.L', 'Pinky1.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Hand.L', 'Thumb1.L', [-5,5,-30,30,-30,30,True] ],

        [ 'Index1.R', 'Index2.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Mid1.R', 'Mid2.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Ring1.R', 'Ring2.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Pinky1.R', 'Pinky2.R', [-5,5,-2,2,-60,5,True] ],
        [ 'Thumb1.R', 'Thumb2.R', [-20,20,-20,20,-20,20,True] ],
        [ 'Index1.L', 'Index2.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Mid1.L', 'Mid2.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Ring1.L', 'Ring2.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Pinky1.L', 'Pinky2.L', [-5,5,-2,2,-5,60,True] ],
        [ 'Thumb1.L', 'Thumb2.L', [-20,20,-20,20,-20,20,True] ]
    ],
    'stiffness_map': [
        [0.05, ["pelvis","abdomenLower","abdomenUpper","chestLower","chestUpper","neckLower"]],
        [0.75, ["Hand.R", "Hand.L", "Index1.R", "Mid1.R", "Ring1.R", "Pinky1.R", "Thumb2.R", "Index1.L", "Mid1.L", "Ring1.L", "Pinky1.L", "Thumb2.L"]],
        [0.2,  ["head","neckUpper","neckLower"]],
        [1.0, ["chestUpper","Collar.R","Collar.L"]]
    ],
    'minimize_twist': [
        [0.6, ["ShldrBend.R","ShldrTwist.R","ShldrBend.L","ShldrTwist.L"], 25],
        [0.6, ["ThighBend.R","ThighTwist.R","ThighBend.L","ThighTwist.L"], 25],
        [0.6, ["ForearmBend.R","ForearmTwist.R","ForearmBend.L","ForearmTwist.L"], 25]
    ],
    'collision_group_ext': [
        'ShldrBend.L',
        'ShldrBend.R',
        'Collar.L',
        'Collar.R'
    ]
}