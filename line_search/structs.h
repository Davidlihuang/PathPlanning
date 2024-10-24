
#ifndef _structs_h
#define _structs_h

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "header.h"

typedef struct LineX LineX;
typedef struct LineY LineY;
typedef struct LineZ LineZ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// X Line implementation
struct LineX
{
    LineY *parentLineY;
    LineZ *parentLineZ;
    int x, y, z;
    int xStart, xEnd;
};

// Y Line implementation
struct LineY
{
    LineX *parentLineX;
    LineZ *parentLineZ;
    int x, y, z;
    int yStart, yEnd;
};

// Z Line implementation
struct LineZ
{
    LineX *parentLineX;
    LineY *parentLineY;
    int x, y, z;
    int zStart, zEnd;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Struct to store Pointers to the set of unique lines
typedef struct
{
    // Number of unique lines in the worst case (unit len)
    bool included[HORIZON_LEN][HORIZON_LEN][HORIZON_LEN];
    LineX array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int prevSize; // for the prev level
    int size;
} LinePointersX;

// Y LineSet implementation
typedef struct
{
    bool included[HORIZON_LEN][HORIZON_LEN][HORIZON_LEN];
    LineY array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int prevSize; // for the prev level
    int size;
} LinePointersY;

// Z LineSet implementation
typedef struct
{
    bool included[HORIZON_LEN][HORIZON_LEN][HORIZON_LEN];
    LineZ array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int prevSize; // for the prev level
    int size;
} LinePointersZ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Struct for storing the set of obstacles
typedef struct
{
    Point3D array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN];
    int size;
} ObstacleArray;

// Struct for storing intersections
typedef struct
{
    Point3D p3D;
    LineX *lineX;
    LineY *lineY;
    LineZ *lineZ;
} Intersection;

// Struct for storing path to src or dst
typedef struct
{
    float array[HORIZON_LEN * HORIZON_LEN * HORIZON_LEN][3];
    int size;
} PathFromIntersection;

// Struct for storing grid limits
typedef struct
{
    int xStart;
    int xEnd;
    int yStart;
    int yEnd;
    int zStart;
    int zEnd;
} GridLimits;

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})



#endif