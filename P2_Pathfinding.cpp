#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"
#include <iostream>


#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
	height = terrain->maxMapHeight;
	width = terrain->maxMapWidth;

	Map = std::vector<Node>(height*width);
	
	OpenList.allocate(height*width);

	Callback cb = std::bind(&AStarPather::PreProcessing, this);
	Messenger::listen_for_message(Messages::MAP_CHANGE, cb);


    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{

}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

	if (request.newRequest)
	{
		OpenList.size = 0;
		Map.clear();
		Map = std::vector<Node>(height*width, std::make_tuple(-1, -1, 200.f, 0.f, NONE));

		start = terrain->get_grid_position(request.start);
		goal = terrain->get_grid_position(request.goal);

		if (terrain->is_wall(goal) || !terrain->is_valid_grid_position(goal))
			return PathResult::IMPOSSIBLE;
		

		if (start == goal)
			return PathResult::COMPLETE;

		type = request.settings.heuristic;
		weight = request.settings.weight;

		if (request.settings.debugColoring)
		{
			terrain->set_color(start, Colors::Orange);
			terrain->set_color(goal, Colors::Red);
		}

		cost = 0.f;

		size_t mapindex = start.row*width + start.col;
		std::get<COST>(Map[mapindex]) = weight * cost;
		std::get<onList>(Map[mapindex]) = OPEN;

		OpenList.insert(&Map[mapindex]);
	}
	while (!OpenList.empty())
	{
		Node* top = OpenList.getTop();
		OpenList.pop();

		GridPos position = getGridPos(top);
		float top_gx = std::get<GIVEN>(*top);

		//if goal, found
		if (position == goal)
		{
			if(request.settings.rubberBanding)
				Rubberbanding(top, position);

			request.path.push_back(terrain->get_world_position(goal));
			pushtopath(request, *top, start);

			return PathResult::COMPLETE;
		}

		//neighbor
		for (int i = position.row -1; i <= position.row + 1; ++i)
		{
			for (int j = position.col - 1; j <= position.col + 1; ++j)
			{
				if (i < 0 || j < 0 || i >= height || j >= width)
					continue;
				if (i == position.row && j == position.col)
					continue;

				GridPos neighbor;
				neighbor.col = j;
				neighbor.row = i;

				//Check Wall
				if (terrain->is_wall(neighbor) || !terrain->is_valid_grid_position(neighbor))
					continue;

				int offset = std::abs(i - position.row ) + std::abs(j  - position.col);
				float gx = 0;
				if (offset == 1)
					gx = 1.f;
				else
				{
					GridPos checkWall1, checkWall2;
					checkWall1.col = neighbor.col;
					checkWall1.row = position.row;
					checkWall2.col = position.col;
					checkWall2.row = neighbor.row;
					if (terrain->is_wall(checkWall1) || terrain->is_wall(checkWall2))
						continue;
					else
						gx = SQRT_TWO;
				}

				cost = top_gx + gx + weight * computecost(i, j, goal, type);

				size_t mapindex = i * width + j;

				if (std::get<onList>(Map[mapindex]) == NONE)
				{
					std::get<xParent>(Map[mapindex]) = position.col;
					std::get<yParent>(Map[mapindex]) = position.row;
					std::get<COST>(Map[mapindex]) = cost;
					std::get<GIVEN>(Map[mapindex]) = top_gx + gx;
					std::get<onList>(Map[mapindex]) = OPEN;
					if (request.settings.debugColoring)
						terrain->set_color(neighbor, Colors::Blue);
					OpenList.insert(&Map[mapindex]);
				}
				else
				{
					if (cost < std::get<COST>(Map[mapindex]))
					{
						std::get<xParent>(Map[mapindex]) = position.col;
						std::get<yParent>(Map[mapindex]) = position.row;
						std::get<COST>(Map[mapindex]) = cost;
						std::get<GIVEN>(Map[mapindex]) = top_gx + gx;
						if (request.settings.debugColoring)
							terrain->set_color(neighbor, Colors::Blue);
						if (std::get<onList>(Map[mapindex]) == OPEN)
						{
							OpenList.rearrange(OpenList.find(&Map[mapindex]));
						}
						else if (std::get<onList>(Map[mapindex]) == CLOSED)
						{
							OpenList.insert(&Map[mapindex]);
							std::get<onList>(Map[mapindex]) = OPEN;
						}
					}
				}

			}
		}
		std::get<onList>(*top) = CLOSED;
		if (request.settings.debugColoring)
			terrain->set_color(position, Colors::Yellow);

		if (request.settings.singleStep)
			return PathResult::PROCESSING;
	}

	return PathResult::IMPOSSIBLE;
}

void AStarPather::PreProcessing()
{
	height = terrain->get_map_height();
	width = terrain->get_map_width();

	Map.clear();
	Map.resize(height*width, std::make_tuple(-1, -1, 200.f, 0.f, NONE));

	OpenList.resize(height*width);
}

float AStarPather::computecost(int row, int col,  GridPos goal, Heuristic type)
{
	float cost = 0.f;
	float xDiff = (float)std::abs(goal.col - col);
	float yDiff = (float)std::abs(goal.row - row);
	if (type == Heuristic::OCTILE)
	{
		float min = std::min(xDiff, yDiff);
		float max = std::max(xDiff, yDiff);
		cost = min * SQRT_TWO_ONE + max;
	}
	else if (type == Heuristic::CHEBYSHEV)
	{
		cost = std::max(xDiff, yDiff);
	}
	else if (type == Heuristic::MANHATTAN)
	{
		cost = xDiff + yDiff;
	}
	else if (type == Heuristic::EUCLIDEAN)
	{
		xDiff *= xDiff;
		yDiff *= yDiff;
		cost = std::sqrtf(xDiff + yDiff);
	}
	else
		cost = 0.f;

	return cost;
}

void AStarPather::pushtopath(PathRequest & request, Node end, GridPos start)
{
	GridPos parent;
	parent.col = std::get<xParent>(end);
	parent.row = std::get<yParent>(end);
	while (parent != start)
	{
		size_t index = parent.row*width + parent.col;
		request.path.push_front(terrain->get_world_position(parent));
		//if (request.settings.debugColoring)
		//	terrain->set_color(parent, Colors::BlueViolet);
		parent.col = std::get<xParent>(Map[index]);
		parent.row = std::get<yParent>(Map[index]);
	}
	
	request.path.push_front(terrain->get_world_position(start));

	if (request.settings.smoothing)
	{
		if (request.settings.rubberBanding)
			AddPoints(request);
		Smoothing(request);
	}
	
}

GridPos AStarPather::getGridPos(Node * node)
{
	GridPos result;
	int gap = (int)(node - &Map[0]);

	result.col = gap % (int)width;
	result.row = gap / (int)width;

	return result;
}

bool AStarPather::CanEliminate(GridPos paths[], int start, int size)
{
	GridPos max = paths[0];
	GridPos min = paths[0];

	for (int i = start; i < size; ++i)
	{
		if (paths[i].col > max.col)
			max.col = paths[i].col;
		if (paths[i].row > max.row)
			max.row = paths[i].row;
		if (paths[i].col < min.col)
			min.col = paths[i].col;
		if (paths[i].row < min.row)
			min.row = paths[i].row;
	}

	for (int i = min.row; i <= max.row; ++i)
	{
		for (int j = min.col; j <= max.col; ++j)
		{
			if (i < 0 || j < 0)
				continue;

			GridPos check;
			check.row = i;
			check.col = j;
			if (terrain->is_wall(check))
				return false;
		}
	}

	return true;
}

void AStarPather::Rubberbanding(Node* end, GridPos last)
{
	
	GridPos path[3];
	path[0] = last;
	path[1].col = std::get<xParent>(*end);
	path[1].row = std::get<yParent>(*end);
	path[2].col = std::get<xParent>(Map[path[1].row*width + path[1].col]);
	path[2].row = std::get<yParent>(Map[path[1].row*width + path[1].col]);

	while (1)
	{
		//get three node
		if (CanEliminate(path, 0, 3))
		{
			std::get<xParent>(Map[path[0].row * width + path[0].col]) = path[2].col;
			std::get<yParent>(Map[path[0].row * width + path[0].col]) = path[2].row;
		}
		else
			path[0] = path[1];
		
		path[1] = path[2];

		path[2].col = std::get<xParent>(Map[path[1].row*width + path[1].col]);
		path[2].row = std::get<yParent>(Map[path[1].row*width + path[1].col]);

		if (path[2].col < 0 || path[2].row < 0)
			break;

	}
}


void AStarPather::Smoothing(PathRequest& request)
{
	Vec3 p1, p2, p3, p4;
	WaypointList::iterator it = request.path.begin();
	p1 = *it;
	p2 = *it;
	std::advance(it, 1);
	p3 = *it;

	if (request.path.size() >= 3)
	{
		std::advance(it, 1);
		p4 = *it;
	}
	else
		p4 = p3;

	while (1)
	{
		std::vector<Vec3> toPush;
		toPush.push_back(Vec3::CatmullRom(p1, p2, p3, p4, 0.25f));
		toPush.push_back(Vec3::CatmullRom(p1, p2, p3, p4, 0.5f));
		toPush.push_back(Vec3::CatmullRom(p1, p2, p3, p4, 0.75f));

		std::advance(it, -1);
		request.path.insert(it, toPush.begin(), toPush.end());
		std::advance(it, 1);

		if (p1 != p2)
			p1 = p2;
		p2 = p3;
		p3 = p4;

		if (p2 == request.path.back())
			break;

		std::advance(it, 1);
		if (it != request.path.end())
			p4 = *it;

	}
}

void AStarPather::AddPoints(PathRequest & request)
{
	Vec3 p1, p2;
	WaypointList::iterator it = request.path.begin();

	p1 = *it;
	std::advance(it, 1);
	p2 = *it;

	while (1)
	{
		GridPos g_p1 = terrain->get_grid_position(p1);
		GridPos g_p2 = terrain->get_grid_position(p2);

		if(Vec3::Distance(p1, p2) > terrain->mapSizeInWorld / (terrain->get_map_width())*1.5f)
		{
			Vec3 push = (p1 + p2)*0.5f;
			request.path.insert(it, push);

			p2 = push;
			std::advance(it, -1);
		}
		else
		{
			p1 = p2;
			std::advance(it, 1);
			if (it == request.path.end())
				break;

			p2 = *it;
		}


	}
}



AStarPather::Heap::~Heap()
{
	delete[] heap;
}

void AStarPather::Heap::insert(Node* node)
{
	int pos = size;
	int parent_pos = (int)((pos - 1) >> 1);

	heap[pos] = node;

	while (pos > 0 &&
		std::get<COST>(*heap[pos]) <= std::get<COST>(*heap[parent_pos]))
	{
		if (std::abs(std::get<COST>(*heap[pos])-std::get<COST>(*heap[parent_pos])) < 0.001f)
		{
			if (std::get<GIVEN>(*heap[pos]) < std::get<GIVEN>(*heap[parent_pos]))
			{
				swap(pos, parent_pos);

				pos = parent_pos;
				parent_pos = (int)((pos - 1) >> 1);
			}
			else
				break;
		}
		else
		{
			swap(pos, parent_pos);

			pos = parent_pos;
			parent_pos = (int)((pos - 1) >> 1);
		}
	}


	++size;

}

void AStarPather::Heap::pop()
{
	--size;
	if (size == 0)
		return;
	
	heap[0] = heap[size];

	int parent = 0;
	int leftPos = 1;
	int rightPos = 2;

	while (leftPos < size)
	{
		int child = 0;
		if (rightPos >= size
			|| std::get<COST>(*heap[rightPos]) >= std::get<COST>(*heap[leftPos]))//no right child
			child = leftPos;
		else if (rightPos < size
			&& std::get<COST>(*heap[rightPos]) < std::get<COST>(*heap[leftPos]))//right child exist
			child = rightPos;

		if (std::get<COST>(*heap[child]) <= std::get<COST>(*heap[parent]))
		{
			if (std::abs(std::get<COST>(*heap[child]) - std::get<COST>(*heap[parent])) < 0.001f)
			{
				if (std::get<GIVEN>(*heap[child]) < std::get<GIVEN>(*heap[parent]))
				{
					swap(parent, child);
					parent = child;
				}
				else
					break;
			}
			else
			{
				swap(parent, child);
				parent = child;
			}
		}
		else
			break;

		leftPos = (parent << 1) + 1;
		rightPos = (parent + 1) << 1;
	}

}

Node * AStarPather::Heap::getTop()
{
	return heap[0];
}

void AStarPather::Heap::rearrange(int i)
{
	//parent
	int parent = (int)((i - 1) >> 1);

	while (i > 0 &&
		std::get<COST>(*heap[i]) <= std::get<COST>(*heap[parent]))
	{
		if (std::abs(std::get<COST>(*heap[i]) - std::get<COST>(*heap[parent])) < 0.001f)
		{
			if (std::get<GIVEN>(*heap[i]) < std::get<GIVEN>(*heap[parent]))
			{
				swap(i, parent);

				i = parent;
				parent = (int)((i - 1) >> 1);
			}
			else
				break;
		}
		else
		{
			swap(i, parent);

			i = parent;
			parent = (int)((i - 1) >> 1);
		}
	}

	//child
	parent = i;
	int leftPos = (parent << 1) + 1;
	int rightPos = (parent + 1) << 1;

	while (leftPos < size)
	{
		int child = 0;
		if (rightPos >= size
			|| std::get<COST>(*heap[rightPos]) >= std::get<COST>(*heap[leftPos]))//no right child
			child = leftPos;
		else if(rightPos < size 
			&& std::get<COST>(*heap[rightPos]) < std::get<COST>(*heap[leftPos]))//right child exist
			child = rightPos;

		if (std::get<COST>(*heap[child]) <= std::get<COST>(*heap[parent]))
		{
			if (std::abs(std::get<COST>(*heap[child]) - std::get<COST>(*heap[parent])) < 0.001f)
			{
				if (std::get<GIVEN>(*heap[child]) < std::get<GIVEN>(*heap[parent]))
				{
					swap(parent, child);
					parent = child;
				}
				else
					break;
			}
			else
			{
				swap(parent, child);
				parent = child;
			}
		}
		else
			break;

		leftPos = (parent << 1) + 1;
		rightPos = (parent + 1) << 1;
	}
}

int AStarPather::Heap::find(Node * node)
{
	int i;
	int l_px, l_py;
	float l_cost, l_given;
	ListType l_list;
	std::tie(l_px, l_py, l_cost, l_given, l_list) = (*node);
	for (i = 0; i < size; ++i)
	{
		int px, py;
		float cost, given;
		ListType list;
		std::tie(px, py, cost, given, list) = (*heap[i]);
		if (px == l_px && py == l_py && cost == l_cost
			&& l_given == given && l_list == list)
			break;
	}
	return i;
}


void AStarPather::Heap::resize(size_t s)
{
	capacity = s;
	Node** newheap = new Node*[capacity];
	
	delete[] heap;
	heap = newheap;
	size = 0;

}
