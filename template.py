__author__ = 'bill'

template = {
    'bones': {
        "root" : [0.29, 0.05, 0.25, (0, -0.1, 0), True],
        "spine04" : [0.18, 0.16, None, (0, 0.02, 0), True],
        "spine03" : [0.15, 0.11],
        "spine02" : [0.24, 0.38, 0.13],
        "spine01" : [0.12, 0.20, 0.11],
        "neck01" : [0.08, 0.08],
        "neck02" : [0.08, 0.08],
        "neck03" : [0.08, 0.08],
        "head" : [0.30, 0.30, 0.2],
        "upperleg01_L" : [0.11, 0.11, 0.2 ],
        "upperleg01_R" : [0.11, 0.11, 0.2 ],
        "upperleg02_L" : [0.11, 0.11, 0.22, (0, 0.1, 0)],
        "upperleg02_R" : [0.11, 0.11, 0.22, (0, 0.1, 0)],
        "lowerleg01_L" : [0.10, 0.10],
        "lowerleg01_R" : [0.10, 0.10],
        "lowerleg02_L" : [0.10, 0.10],
        "lowerleg02_R" : [0.10, 0.10],
        "foot_L" : [0.10, 0.10, 0.2],
        "foot_R" : [0.10, 0.10, 0.2]
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
        [ 'root', 'upperleg01_L', [-75,135,-5,5,-70,70,True] ],
        [ 'root', 'upperleg01_R', [-75,135,-5,5,-70,70,True] ],
        [ 'upperleg01_L', 'upperleg02_L', [0,0,-55,55,0,0,True] ],
        [ 'upperleg01_R', 'upperleg02_R', [0,0,-55,55,0,0,True] ],
        [ 'upperleg02_R', 'lowerleg01_R', [-140,1,-5,5,0,0,True] ],
        [ 'upperleg02_L', 'lowerleg01_L', [-140,1,-5,5,0,0,True] ],
        [ 'lowerleg01_R', 'lowerleg02_R', [0,0,-12,12,0,0,True] ],
        [ 'lowerleg01_L', 'lowerleg02_L', [0,0,-12,12,0,0,True] ],
        [ 'lowerleg02_R', 'foot_R', [-30,30,-10,10,-10,10,True] ],
        [ 'lowerleg02_L', 'foot_L', [-30,30,-10,10,-10,10,True] ]
    ],
    'stiffness_map': [
        [0.05, ["root", "spine04", "spine03", "spine02", "spine01"]],
        [0.25,  ["spine02", "spine01", "neck01", "neck02", "neck03"]],
        #[0.75, ["Hand.R", "Hand.L", "Index1.R", "Mid1.R", "Ring1.R", "Pinky1.R", "Thumb2.R", "Index1.L", "Mid1.L", "Ring1.L", "Pinky1.L", "Thumb2.L"]],
        [0.25,  ["head", "neck03"]],
        #[0.25, ["chestUpper","Collar.R","Collar.L"]]
    ],
    'minimize_twist': [
        #[0.6, ["ShldrBend.R","ShldrTwist.R","ShldrBend.L","ShldrTwist.L"], 25],
        [0.6,  ["upperleg01_L", "upperleg02_L", "upperleg01_R", "upperleg02_R"], 25],
        #[0.6, ["ForearmBend.R","ForearmTwist.R","ForearmBend.L","ForearmTwist.L"], 25]
    ]
}