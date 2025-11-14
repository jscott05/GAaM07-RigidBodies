#include "SpatialGrid.h"
#include <cmath>
#include <algorithm>
#include <unordered_set>

// init spatial grid
// if world size 1200x800 with 100 px
// grid width = 1200/100 = 12 cells
// grid height = 800/100 = 8 cells
// total cells = 12 * 8 = 96 cells

SpatialGrid::SpatialGrid(float worldWidth, float worldHeight, float cellSize)
	: worldWidth(worldWidth), worldHeight(worldHeight), cellSize(cellSize)
{
	gridWidth = static_cast<int>(std::ceil(worldWidth / cellSize));
	gridHeight = static_cast<int>(std::ceil(worldHeight / cellSize));
	cells.resize(gridWidth * gridHeight);
}

void SpatialGrid::clear()
{
	for(auto& cell : cells)
	{
		cell.bodies.clear();
	}
}
void SpatialGrid::insert(RigidBody* body)
{
	auto cellIndices = getCellsForBody(body);
	for (int index : cellIndices)
	{
		cells[index].bodies.push_back(body);
	}
}

// response for getting collision pairs from the grid
// broad phase collision detection

// 1, for each cell, check all pairs of bodies within the cell
// 2, use hash set to avoid duplicate pairs (bodies in multiple cells)
// 3, return list of unique pairs
std::vector<std::pair<RigidBody*, RigidBody*>> SpatialGrid::getPotentialCollisions()
{
	std::vector<std::pair<RigidBody*, RigidBody*>> pairs;
	std::unordered_set<size_t> processedPairs;

	for (const auto& cell : cells)
	{
		const auto& bodies = cell.bodies;

		// check all pairs within the cell using nest loop
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			for (size_t j = i + 1; j < bodies.size(); ++j)
			{
				RigidBody* body1 = bodies[i];
				RigidBody* body2 = bodies[j];

				// create a unique hash for the pair
				// use pointer addresses to create unique ids
				// order pointers to ensure consistent hashing

				size_t hash = 0;
				if (body1 < body2)
				{
					hash = reinterpret_cast<size_t>(body1) ^ (reinterpret_cast<size_t>(body2) << 1);
				}
				else
				{
					hash = reinterpret_cast<size_t>(body2) ^ (reinterpret_cast<size_t>(body1) << 1);
				}

				// only add if not already processed - prevent duplicates

				if (processedPairs.find(hash) == processedPairs.end())
				{
					processedPairs.insert(hash);
					pairs.emplace_back(body1, body2);
				}
			}
		}
	}
	return pairs;
}

int SpatialGrid::getCellX(float x) const
{
	int cellX = static_cast<int>(x / cellSize);
	return std::clamp(cellX, 0, gridWidth - 1);
}

int SpatialGrid::getCellY(float y) const
{
	int cellY = static_cast<int>(y / cellSize);
	return std::clamp(cellY, 0, gridHeight - 1);
}

int SpatialGrid::getCellIndex(int cellx, int celly) const
{
	// index = y * gridWidth + x
	return celly * gridWidth + cellx;
}

void SpatialGrid::insertBodyIntoCell(RigidBody* body, int cellX, int cellY)
{
	// bounds check - ensure cell coords are valid
	if (cellX >= 0 && cellX < gridWidth && cellY >= 0 && cellY < gridHeight)
	{
		int index = getCellIndex(cellX, cellY);
		cells[index].bodies.push_back(body);
	}
}
// determine which cells a body overlaps
std::vector<int> SpatialGrid::getCellsForBody(RigidBody* body) const
{
	std::vector<int> cellIndices;
	sf::Vector2f pos = body->getPosition();
	float radius = body->getRadius();

	// calculate bounding box of body circle

	int minCellX = getCellX(pos.x - radius);
	int maxCellX = getCellX(pos.x + radius);
	int minCellY = getCellY(pos.y - radius);
	int maxCellY = getCellY(pos.y + radius);

	for (int y = minCellY; y <= maxCellY; ++y)
	{
		for (int x = minCellX; x <= maxCellX; ++x)
		{
			cellIndices.push_back(getCellIndex(x, y));
		}
	}
	return cellIndices;
}
