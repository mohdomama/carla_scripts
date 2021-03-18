""" Lists custom spawn locations for NPCs. 

    Save locations of spawn points where NPCs should be generated. Used by spawn_custom.py.

    Currently, the locations add 12 cars on the left lane and 11 cars on the right lane.

    USAGE: 
        N/A
"""

CUSTOM_SPAWN_POINTS = {
    'Town01': [],
    'Town02': [ # uses SET-3 below
        {'Location': (132.718338, 110.301804, 0.821689), 'Rotation': (0.000000, 0.038567, 0.000014)},
        {'Location': (147.712631, 105.358192, 0.821703), 'Rotation': (-0.000273, -177.641846, 0.000012)},
        {'Location': (147.980911, 105.561806, 0.821674), 'Rotation': (0.000403, -179.063736, -0.000031)},
        {'Location': (116.006111, 105.042389, 0.821685), 'Rotation': (-0.001769, -179.063904, 0.000014)},
        {'Location': (67.408035, 105.324356, 0.821663), 'Rotation': (0.000321, 178.000992, 0.000020)},
        {'Location': (53.834099, 105.607529, 0.821738), 'Rotation': (0.000328, 179.241089, 0.000043)},
        {'Location': (25.004023, 105.825409, 0.821661), 'Rotation': (0.000929, 179.758377, 0.000014)},
        {'Location': (-7.567891, 133.176193, 0.821682), 'Rotation': (-0.000184, 91.572685, 0.000015)},
        {'Location': (-7.042178, 245.986435, 0.821669), 'Rotation': (0.000389, 89.642899, 0.000015)},
        {'Location': (-7.191473, 222.038269, 0.821700), 'Rotation': (0.000389, 89.642899, 0.000015)},
        {'Location': (-7.329150, 199.949646, 0.821678), 'Rotation': (0.000499, 89.642899, 0.000068)},
        {'Location': (130.716858, 108.938690, 0.821669), 'Rotation': (-0.000034, -2.352539, 0.000021)},
        {'Location': (110.098953, 109.302002, 0.821678), 'Rotation': (0.000225, -1.183807, 0.000014)},
        {'Location': (92.096581, 109.551285, 0.821695), 'Rotation': (0.000205, -1.287277, 0.000015)},
        {'Location': (71.389214, 109.628647, 0.821692), 'Rotation': (0.000676, -1.904999, 0.000014)},
        {'Location': (33.252842, 109.691193, 0.821693), 'Rotation': (-0.001680, -1.904831, 0.000014)},
        {'Location': (-4.003404, 270.255890, 0.821680), 'Rotation': (0.000396, -91.541580, 0.000016)},
        {'Location': (-4.184578, 247.613312, 0.821726), 'Rotation': (0.000396, -91.541580, 0.000016)},
        {'Location': (-4.317701, 230.974228, 0.821678), 'Rotation': (0.000505, -91.541580, -0.000031)},
        {'Location': (-3.294299, 163.348938, 0.221620), 'Rotation': (0.010259, 91.198952, 0.000015)},
        {'Location': (-3.090862, 153.628067, 0.221678), 'Rotation': (0.000505, 91.198952, 0.000015)},
        {'Location': (190.810760, 212.979767, 0.221840), 'Rotation': (0.001188, -91.203918, 0.000000)},
        {'Location': (190.532440, 199.741776, 0.221841), 'Rotation': (0.000970, -91.203918, 0.000014)},
        {'Location': (190.006943, 174.664215, 0.221841), 'Rotation': (0.000861, -91.203918, 0.000014)} 
    ]
    'Town03': [
        {'Location': (18.211578, 130.357574, 0.801680), 'Rotation': (-0.000123, -179.942932, 0.000014)},
        {'Location': (27.145554, 127.382416, 0.801681), 'Rotation': (-0.000123, 179.260529, 0.000015)},
        {'Location': (27.145554, 133.382416, 0.801679), 'Rotation': (0.000027, -179.791626, 0.000000)},
        {'Location': (87.220924, 194.343231, 1.153675), 'Rotation': (-1.277402, -179.359268, -0.017578)},
        {'Location': (93.220924, 190.343231, 1.153675), 'Rotation': (-1.277402, -179.359268, -0.017578)},
        {'Location': (93.220924, 198.343231, 1.153675), 'Rotation': (-1.277402, -179.359268, -0.017578)}
    ]
}

"""
[Town 2 SET 1]
{'Location':  (145.728455, 105.276680, 0.821698), 'Rotation': (0.001059, -177.648773, 0.000016)},
{'Location':  (-8.175824, 181.703400, 0.821697), 'Rotation': (-0.002746, 92.118103, 0.000006)},
{'Location':  (-7.301988, 278.500793, 0.821705), 'Rotation': (0.000526, 90.425209, 0.000015)},
{'Location':  (101.330948, 307.095825, 0.821678), 'Rotation': (0.000492, 0.253462, 0.000016)},
{'Location':  (78.464478, 306.990509, 0.821678), 'Rotation': (0.000601, 0.253475, 0.000016)},
{'Location':  (69.222939, 306.952240, 0.821677), 'Rotation': (0.000601, 0.253475, 0.000016)}

[Town 2 SET 2]
{'Location': (32.463158, 106.316826, 0.221734), 'Rotation': (-0.001742, -179.143448, -0.000031)},
{'Location': (61.808929, 106.748337, 0.221680), 'Rotation': (0.000055, -179.209778, 0.000014)},
{'Location': (-7.486465, 151.478683, 0.221681), 'Rotation': (-0.000123, 90.342384, 0.000011)},
{'Location': (-7.554360, 177.872528, 0.221681), 'Rotation': (-0.000061, 90.132111, 0.000015)},
{'Location': (-7.703819, 208.886414, 0.221699), 'Rotation': (-0.000273, 90.285011, 0.000015))

[Town 2 SET 3]
right lane:
{'Location': (147.712631, 105.358192, 0.821703), 'Rotation': (-0.000273, -177.641846, 0.000012)},
{'Location': (147.980911, 105.561806, 0.821674), 'Rotation': (0.000403, -179.063736, -0.000031)},
{'Location': (116.006111, 105.042389, 0.821685), 'Rotation': (-0.001769, -179.063904, 0.000014)},
{'Location': (67.408035, 105.324356, 0.821663), 'Rotation': (0.000321, 178.000992, 0.000020)},
{'Location': (53.834099, 105.607529, 0.821738), 'Rotation': (0.000328, 179.241089, 0.000043)},
{'Location': (25.004023, 105.825409, 0.821661), 'Rotation': (0.000929, 179.758377, 0.000014)},
{'Location': (-7.567891, 133.176193, 0.821682), 'Rotation': (-0.000184, 91.572685, 0.000015)},
{'Location': (-7.042178, 245.986435, 0.821669), 'Rotation': (0.000389, 89.642899, 0.000015)},
{'Location': (-7.191473, 222.038269, 0.821700), 'Rotation': (0.000389, 89.642899, 0.000015)},
{'Location': (-7.329150, 199.949646, 0.821678), 'Rotation': (0.000499, 89.642899, 0.000068)},

left lane:
{'Location': (130.716858, 108.938690, 0.821669), 'Rotation': (-0.000034, -2.352539, 0.000021)},
{'Location': (110.098953, 109.302002, 0.821678), 'Rotation': (0.000225, -1.183807, 0.000014)},
{'Location': (92.096581, 109.551285, 0.821695), 'Rotation': (0.000205, -1.287277, 0.000015)},
{'Location': (71.389214, 109.628647, 0.821692), 'Rotation': (0.000676, -1.904999, 0.000014)},
{'Location': (33.252842, 109.691193, 0.821693), 'Rotation': (-0.001680, -1.904831, 0.000014)},
{'Location': (-4.003404, 270.255890, 0.821680), 'Rotation': (0.000396, -91.541580, 0.000016)},
{'Location': (-4.184578, 247.613312, 0.821726), 'Rotation': (0.000396, -91.541580, 0.000016)},
{'Location': (-4.317701, 230.974228, 0.821678), 'Rotation': (0.000505, -91.541580, -0.000031)},
{'Location': (-3.294299, 163.348938, 0.221620), 'Rotation': (0.010259, 91.198952, 0.000015)},
{'Location': (-3.090862, 153.628067, 0.221678), 'Rotation': (0.000505, 91.198952, 0.000015)},
{'Location': (190.810760, 212.979767, 0.221840), 'Rotation': (0.001188, -91.203918, 0.000000)},
{'Location': (190.532440, 199.741776, 0.221841), 'Rotation': (0.000970, -91.203918, 0.000014)},
{'Location': (190.006943, 174.664215, 0.221841), 'Rotation': (0.000861, -91.203918, 0.000014)}

[Town 3 Set 1]
{'Location': (18.211578, 130.357574, 0.001680), 'Rotation': (-0.000123, -179.942932, 0.000014)},
{'Location': (27.145554, 127.382416, 0.001681), 'Rotation': (-0.000123, 179.260529, 0.000015)},
{'Location': (27.145554, 133.382416, 0.001679), 'Rotation': (0.000027, -179.791626, 0.000000)},
{'Location': (87.220924, 194.343231, 1.153675), 'Rotation': (-1.277402, -179.359268, -0.017578)},
{'Location': (93.220924, 190.343231, 1.153675), 'Rotation': (-1.277402, -179.359268, -0.017578)},
{'Location': (93.220924, 198.343231, 1.153675), 'Rotation': (-1.277402, -179.359268, -0.017578)}
"""