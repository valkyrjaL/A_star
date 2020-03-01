#ifndef MINHEAP_H
#define MINHEAP_H

struct cell
{
	// double f, g, h;
	double f, g;
	int x, y;
	bool closed;
	struct cell *predecessor;
};

class MinHeap
{
private:
	int capacity;
	int size;
	cell *items = nullptr;
	
	int getLeftChildIndex(int parentIndex){return (parentIndex * 2 + 1);}
	int getRightChildIndex(int parentIndex){return (parentIndex * 2 + 2);}
	int getParentIndex(int childIndex){return (childIndex - 1) / 2;}
	
	bool hasLeftChild(int index){return getLeftChildIndex(index) < size;}
	bool hasRightChild(int index){return getRightChildIndex(index) < size;}
	bool hasParent(int index){return getParentIndex(index) >= 0;}
	
	cell leftChild(int index){return items[getLeftChildIndex(index)];}
	cell rightChild(int index){return items[getRightChildIndex(index)];}
	cell parent(int index){return items[getParentIndex(index)];}
	
	void swap(int indexOne, int indexTwo);
	
	/* If the array was full, increase the array with capacity.*/
	void ensureExtraCapacity();

public:

	MinHeap();
	
	~MinHeap();
	
	int heapCount(){return size;}
	
	void heapifyUp();
	
	void heapifyDown();
	
	cell pop();
	
	void add(cell item);
	
	/* Peek the first node without removing it.*/
	cell peek();

};

#endif