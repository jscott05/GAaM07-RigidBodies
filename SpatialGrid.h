#pragma once
#include "RigidBody.h"
// with 100 opbjects : 100 x 99 / 2 = 4950 checks
// with 200 objects : 200 x 199 / 2 = 19900 checks per frame
// quickly unusable

// divide the world into a grid of cells
// each object is placed into the cells it overlaps
// we only collisions between objects in the same or neighboring cells

// quad trees, octrees, kd-trees, sweep and prune, hash grids etc

class SpatialGrid
{
public:
	// too small - objeects span many cells, overhead increases
	// too big - too many objects per cell, head toward o(n^2)
	// rule of thumb - 2-3x average object size

	SpatialGrid(float worldWidth, float worldHeight, float cellSize);

	void clear();

	void insert(RigidBody* body);

	// get all pairs of bodies that could collide - broad phase
	// precise test to confirm collision - narrow phase

	std::vector<std::pair<RigidBody*, RigidBody*>> getPotentialCollisions();

	// use helper function to convert from world coords to cell coords
	int getCellX(float x) const;
	int getCellY(float y) const;
	int getCellIndex(int cellx, int celly) const;

private:
	// a single cell in the grid
	struct cell
	{
		std::vector<RigidBody*> bodies;
	};

	// grid dimensions and config
	float worldWidth, worldHeight, cellSize;
	int gridWidth, gridHeight;
	
	// why a 1d array?
	// to do with memory
	// spatial locality - accessing contiguous memory is faster - thanks copilot

	std::vector<cell> cells;
	
	// helper methods
	void insertBodyIntoCell(RigidBody* body, int cellX, int cellY);
	std::vector<int> getCellsForBody(RigidBody* body) const;

};


