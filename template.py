__author__ = 'bill'
finger_dim_x = 0.005
finger_dim_z = 0.018
template = {
    'bones': {
        "root" : [0.12, 0.08, 0.25, (0, -0.1, 0), True],
        "spine04" : [0.18, 0.16, None, (0, 0.02, 0), True],
        "spine03" : [0.15, 0.11],
        "spine02" : [0.24, 0.38, 0.13],
        "spine01" : [0.05, 0.20, 0.11],
        "neck01" : [0.08, 0.08],
        "neck02" : [0.08, 0.08],
        "neck03" : [0.08, 0.08],
        "head" : [0.30, 0.30, 0.2],
        "upperleg01_L" : [0.11, 0.11],
        "upperleg01_R" : [0.11, 0.11],
        "upperleg02_L" : [0.11, 0.11],
        "upperleg02_R" : [0.11, 0.11],
        "lowerleg01_L" : [0.10, 0.10],
        "lowerleg01_R" : [0.10, 0.10],
        "lowerleg02_L" : [0.10, 0.10],
        "lowerleg02_R" : [0.10, 0.10],
        "foot_L" : [0.10, 0.10, 0.12],
        "foot_R" : [0.10, 0.10, 0.12],

        "toe1-1_L" : [0.08, 0.05, 0.06, (0.04, 0.0, 0.0)],
        "toe1-1_R" : [0.08, 0.05, 0.06, (-0.04, 0.0, 0.0)],

        "clavicle_L" : [0.05, 0.08],
        "clavicle_R" : [0.05, 0.08],
        "shoulder01_L" : [0.08, 0.08],
        "shoulder01_R" : [0.08, 0.08],
        "upperarm01_R" : [0.06, 0.06],
        "upperarm01_L" : [0.06, 0.06],
        "upperarm02_R" : [0.06, 0.06],
        "upperarm02_L" : [0.06, 0.06],
        "lowerarm01_R" : [0.06, 0.06],
        "lowerarm01_L" : [0.06, 0.06],
        "lowerarm02_R" : [0.03, 0.03],
        "lowerarm02_L" : [0.03, 0.03],
        "wrist_R" : [0.05, 0.07, 0.07, (0,0,-0.01)],
        "wrist_L" : [0.05, 0.07, 0.07, (0,0,-0.01)],

        "finger2-1_R": [finger_dim_x, finger_dim_z],
        "finger2-1_L": [finger_dim_x, finger_dim_z],
        "finger3-1_R": [finger_dim_x, finger_dim_z],
        "finger3-1_L": [finger_dim_x, finger_dim_z],
        "finger4-1_R": [finger_dim_x, finger_dim_z],
        "finger4-1_L": [finger_dim_x, finger_dim_z],
        "finger5-1_R": [finger_dim_x, finger_dim_z],
        "finger5-1_L": [finger_dim_x, finger_dim_z],

        "finger2-2_R": [finger_dim_x, finger_dim_z, 0.05],
        "finger2-2_L": [finger_dim_x, finger_dim_z, 0.05],
        "finger3-2_R": [finger_dim_x, finger_dim_z, 0.05],
        "finger3-2_L": [finger_dim_x, finger_dim_z, 0.05],
        "finger4-2_R": [finger_dim_x, finger_dim_z, 0.05],
        "finger4-2_L": [finger_dim_x, finger_dim_z, 0.05],
        "finger5-2_R": [finger_dim_x*0.75, finger_dim_z, 0.03],
        "finger5-2_L": [finger_dim_x*0.75, finger_dim_z, 0.03],
        },
    'constraints': [
        [ 'root', 'spine04', [-20,35,-15,15,-15,15,True] ],
        [ 'spine04', 'spine03', [-25,40,-20,20,-24,24,True] ],
        [ 'spine03', 'spine02', [-25,35,-12,12,-20,20,True] ],
        [ 'spine02', 'spine01', [-15,15,-10,10,-10,10,True] ],
        [ 'spine01', 'neck01', [-15,30,-22,22,-40,40,True] ],
        [ 'neck01', 'neck02', [-17,12,-22,22,-10,10,True] ],
        [ 'neck02', 'neck03', [-17,12,-22,22,-10,10,True] ],
        [ 'neck03', 'head', [-27,25,-22,22,-20,20,True] ],
        [ 'root', 'upperleg01_L', [-75,135,0,0,-60,60,True] ],
        [ 'root', 'upperleg01_R', [-75,135,0,0,-60,60,True] ],
        [ 'upperleg01_L', 'upperleg02_L', [-5,5,-45,45,0,0,True] ],
        [ 'upperleg01_R', 'upperleg02_R', [-5,5,-45,45,0,0,True] ],
        [ 'upperleg02_R', 'lowerleg01_R', [-200,1,-5,5,0,0,True] ],
        [ 'upperleg02_L', 'lowerleg01_L', [-200,1,-5,5,0,0,True] ],
        [ 'lowerleg01_R', 'lowerleg02_R', [0,0,-12,12,0,0,True] ],
        [ 'lowerleg01_L', 'lowerleg02_L', [0,0,-12,12,0,0,True] ],
        [ 'lowerleg02_R', 'foot_R', [-30,30,-10,10,-10,10,True] ],
        [ 'lowerleg02_L', 'foot_L', [-30,30,-10,10,-10,10,True] ],

        [ 'foot_R', 'toe1-1_R', [-20,20,-1,1,-5,5,True] ],
        [ 'foot_L', 'toe1-1_L', [-20,20,-1,1,-5,5,True] ],

        [ 'spine01', 'clavicle_R', [-10,10,-25,25,-5,50,True] ],
        [ 'spine01', 'clavicle_L', [-10,10,-25,25,-50,5,True] ],
        [ 'clavicle_R', 'shoulder01_R', [-10,10,-20,20,-70,10,True] ],
        [ 'clavicle_L', 'shoulder01_L', [-10,10,-20,20,-10,70,True] ],

        [ 'shoulder01_R', 'upperarm01_R', [-40,40,-1,1,-70,30,True] ],
        [ 'shoulder01_L', 'upperarm01_L', [-40,40,-1,1,-30,70,True] ],
        [ 'upperarm01_R', 'upperarm02_R', [-0,0,-110,110,-0,0,True] ],
        [ 'upperarm01_L', 'upperarm02_L', [-0,0,-110,110,-0,0,True] ],
        [ 'upperarm02_R', 'lowerarm01_R', [-130,20,-5,5,-0,0,True] ],
        [ 'upperarm02_L', 'lowerarm01_L', [-130,20,-5,5,-0,0,True] ],
        [ 'lowerarm01_R', 'lowerarm02_R', [-0,0,-100,100,-0,0,True] ],
        [ 'lowerarm01_L', 'lowerarm02_L', [-0,0,-100,100,-0,0,True] ],
        [ 'lowerarm02_R', 'wrist_R', [-30,30,-10,10,-75,75,True] ],
        [ 'lowerarm02_L', 'wrist_L', [-30,30,-10,10,-75,75,True] ],

        [ 'wrist_R', 'finger2-1_R', [-70,10,-1,1,-1,1,True] ],
        [ 'wrist_R', 'finger3-1_R', [-70,10,-1,1,-1,1,True] ],
        [ 'wrist_R', 'finger4-1_R', [-70,10,-1,1,-1,1,True] ],
        [ 'wrist_R', 'finger5-1_R', [-70,10,-1,1,-1,1,True] ],

        [ 'finger2-1_R', 'finger2-2_R', [-70,10,-1,1,-1,1,True] ],
        [ 'finger3-1_R', 'finger3-2_R', [-70,10,-1,1,-1,1,True] ],
        [ 'finger4-1_R', 'finger4-2_R', [-70,10,-1,1,-1,1,True] ],
        [ 'finger5-1_R', 'finger5-2_R', [-70,10,-1,1,-1,1,True] ],

        [ 'wrist_L', 'finger2-1_L', [-70,10,-1,1,-1,1,True] ],
        [ 'wrist_L', 'finger3-1_L', [-70,10,-1,1,-1,1,True] ],
        [ 'wrist_L', 'finger4-1_L', [-70,10,-1,1,-1,1,True] ],
        [ 'wrist_L', 'finger5-1_L', [-70,10,-1,1,-1,1,True] ],

        [ 'finger2-1_L', 'finger2-2_L', [-70,10,-1,1,-1,1,True] ],
        [ 'finger3-1_L', 'finger3-2_L', [-70,10,-1,1,-1,1,True] ],
        [ 'finger4-1_L', 'finger4-2_L', [-70,10,-1,1,-1,1,True] ],
        [ 'finger5-1_L', 'finger5-2_L', [-70,10,-1,1,-1,1,True] ],


        ],
    'stiffness_map': [
        [0.05, ["root", "spine04", "spine03", "spine02", "spine01"]],
        [0.05,  ["spine02", "spine01", "neck01", "neck02", "neck03"]],
        #[0.75, ["Hand.R", "Hand.L", "Index1.R", "Mid1.R", "Ring1.R", "Pinky1.R", "Thumb2.R", "Index1.L", "Mid1.L", "Ring1.L", "Pinky1.L", "Thumb2.L"]],
        [0.05,  ["head", "neck03"]],
        #[0.25, ["chestUpper","Collar.R","Collar.L"]]
    ],
    'minimize_twist': [
        [0.6,  ["upperarm01_L", "upperarm02_L", "upperarm01_R", "upperarm02_R", "lowerarm01_L", "lowerarm02_L", "lowerarm01_R", "lowerarm02_R"], 25],
        [0.6,  ["upperleg01_L", "upperleg02_L", "upperleg01_R", "upperleg02_R", "lowerleg01_L", "lowerleg02_L", "lowerleg01_R", "lowerleg02_R"], 25],
        #[0.6, ["ForearmBend.R","ForearmTwist.R","ForearmBend.L","ForearmTwist.L"], 25]
    ],
    'collision_group_ext': [
        'upperarm01_R',
        'upperarm01_L'
    ]
}