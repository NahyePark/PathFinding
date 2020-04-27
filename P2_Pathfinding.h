#pragma once
#include "Misc/PathfindingDetails.hpp"
#include <tuple>
#include <vector>


#define SQRT_TWO 1.4142136f
#define SQRT_TWO_ONE 0.4142136f

enum Type{
	xParent,
	yParent,
	COST,
	GIVEN,
	onList
};

enum ListType {
	NONE,
	OPEN,
	CLOSED
};

//////////////xParent, yParent, cost(fx), given(gx), onList
typedef std::tuple<int, int, float, float, ListType> Node;

class AStarPather
{
public:

	AStarPather() {}

    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
	
	class Heap
	{
	public:
		Heap() {}
		Heap(size_t s) {
			capacity = s;
			heap = new Node*[capacity];
			size = 0;	
		}
		~Heap();
		void allocate(size_t s)
		{
			capacity = s;
			heap = new Node*[capacity];
			size = 0;
		}
		void insert(Node* node);
		void pop();
		Node* getTop();
		void rearrange(int i);
		int find(Node* node);
		void resize(size_t s);
		bool empty() { return (size==0); }

		void swap(int index1, int index2)
		{
			Node* temp = heap[index1];
			heap[index1] = heap[index2];
			heap[index2] = temp;
		}

		int size;
	private:
		Node* *heap;
		size_t capacity;

	};


	void PreProcessing();
	float computecost(int row, int col, GridPos goal, Heuristic type);
	void pushtopath(PathRequest& request, Node end, GridPos start);
	GridPos getGridPos(Node* node);
	bool CanEliminate(GridPos paths[], int start, int size);
	void Rubberbanding(Node* end, GridPos last);
	void Smoothing(PathRequest& request);
	void AddPoints(PathRequest& request);

	std::vector<Node> Map;
	Heap OpenList;
	size_t height, width;
	GridPos start;
	GridPos goal;
	float weight;
	float cost;
	Heuristic type;
};