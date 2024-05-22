North = 0
NE = 1
East = 2
SE = 3
South = 4
SW = 5
West = 6
NW = 7

INVERSE = {
    North: South,
    NE: SW,
    East: West,
    SE: NW,
    South: North,
    SW: NE,
    West: East,
    NW: SE,
}

DIR_X = {
    North: 0,
    NE: 1,
    East: 1,
    SE: 1,
    South: 0,
    SW: -1,
    West: -1,
    NW: -1,
}
DIR_Y = {
    North: -1,
    NE: -1,
    East: 0,
    SE: 1,
    South: 1,
    SW: 1,
    West: 0,
    NW: -1,
}

TOTAL_DIRECTION = 8