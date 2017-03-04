/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

* File Name : visit.c

* Purpose : Educational

* Creation Date : 02-03-2017

* Last Modified : Σαβ 04 Μάρ 2017 09:22:58 μμ EET

* Created By : Stamatios Anoustis 

_._._._._._._._._._._._._._._._._._._._._.*/

/*------------------Includes and global definitions---------------------*/
#include <stdio.h> 
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>

int c[60000];
int EXIT_STATUS = 0;

/*-----------------Main Code-------------------------------------------*/
//A structure to represent an adjacency list node
struct AdjListNode {

  int dest;
  int weight;
  struct AdjListNode* next;

};

//A structure to represent an adjacency list
struct AdjList {

  struct AdjListNode* head; //pointer to head node of list

}; 

//A structure to represent the Graph.Its an array of 
//adjacency lists.Size of array V.
struct Graph {

  int V;  //number of vertices
  struct AdjList* array;

}; 

//A utility function to create a new adjacency list node
struct AdjListNode* newAdjListNode(int dest, int weight) {

  struct AdjListNode* newNode = (struct AdjListNode*) malloc(sizeof(struct AdjListNode));
  newNode->dest = dest;
  newNode->weight = weight;
  newNode->next = NULL;
  return newNode;

}

//A utility function to create a new Graph of V nodes
struct Graph* newGraph(int V) {

  struct Graph* newGraph = (struct Graph*) malloc(sizeof(struct Graph));
  newGraph->V = V;  
  //Create an array of adjacency lists of size V.
  newGraph->array = (struct AdjList*) malloc(V*sizeof(struct AdjList));
  for (int i = 0; i < V; i++) {

  newGraph->array[i].head = NULL;

  }

  return newGraph;

}

//A utility function to add an edge to an undirected graph
void addEdge(struct Graph* graph, int src, int dest, int weight) {

  struct AdjListNode* newNode = newAdjListNode(dest, weight);
  newNode->next = graph->array[src].head;
  graph->array[src].head = newNode;

  //The symmetric work must be done for undirected graph
  newNode = newAdjListNode(src, weight);
  newNode->next = graph->array[dest].head;
  graph->array[dest].head = newNode;

}

//structure to represent a min heap node
struct MinHeapNode {

  int vertex;
  int key;

};

//structure to represent a min heap
struct MinHeap {

  int capacity;
  int size;
  int *position;
  struct MinHeapNode** array;

};

//utility function to create a min heap node
struct MinHeapNode* newMinHeapNode(int vertex, int key) {

  struct MinHeapNode* minHeapNode = (struct MinHeapNode*) malloc(sizeof(struct MinHeapNode));
  minHeapNode->vertex = vertex;
  minHeapNode->key = key;
  return minHeapNode;

}  

//A utillity function  to create a min heap
struct MinHeap* newMinHeap(int capacity) {

  struct MinHeap* minHeap = (struct MinHeap*) malloc(sizeof(struct MinHeap));
  minHeap->capacity = capacity;
  minHeap->size = 0;
  minHeap->position = (int*) malloc(capacity*sizeof(int));
  minHeap->array = (struct MinHeapNode**) malloc(capacity*sizeof(struct MinHeapNode*));
  return minHeap;

}

//A utillity function to swap min heap nodes 
void swapMinHeapNode(struct MinHeapNode** node1, struct MinHeapNode** node2) {

  struct MinHeapNode* temp = *node1;
  *node1 = *node2;
  *node2 = temp;

}  

//A utillity function to heapify at a given index
void minHeapify(struct MinHeap* minHeap,int index) {

  int smallest = index;
  int left = 2*index + 1; 
  int right = 2*index + 2;
  if ((left < minHeap->size) && (minHeap->array[left]->key < minHeap->array[smallest]->key)) {

    smallest = left;

  } 
  
  if ((right < minHeap->size) && (minHeap->array[right]->key < minHeap->array[smallest]->key)) {

    smallest = right;

  } 

  if (smallest != index) {

    //The nodes to be swapped in min heap
    struct MinHeapNode* smallestNode = minHeap->array[smallest];
    struct MinHeapNode* indexNode = minHeap->array[index];
    //Swap positions
    minHeap->position[smallestNode->vertex] = index;
    minHeap->position[indexNode->vertex] = smallest;
    //Swap min heap nodes
    swapMinHeapNode(&minHeap->array[smallest] , &minHeap->array[index] );
    //Swap rest nodes
    minHeapify(minHeap, smallest);

  }

}  

// A utility function to check if the given minHeap is ampty or not
int isEmpty(struct MinHeap* minHeap) {

  return minHeap->size == 0;
     
}

// Standard function to extract minimum node from heap
struct MinHeapNode* extractMin(struct MinHeap* minHeap) {

  if (isEmpty(minHeap)) {

    return NULL;

  }    
  
  // The root node is the minimum I need to extract
  struct MinHeapNode* root = minHeap->array[0];
  // Replace root node with last node so as to remain a heap
  struct MinHeapNode* lastNode = minHeap->array[minHeap->size - 1];
  minHeap->array[0] = lastNode;                                    
  // Update position of last node
  minHeap->position[root->vertex] = minHeap->size-1;
  minHeap->position[lastNode->vertex] = 0;                                                 
  //Reduce heap size and heapify root
  minHeap->size--;
  minHeapify(minHeap, 0);                                                              
  return root;
                                        
}

//Standard function to decrease key of a given vertex
void decreaseKey(struct MinHeap* minHeap, int vertex, int key) {

  //Get index of vertex in heap array
  int index = minHeap->position[vertex];
  //Get the node and update its key value
  minHeap->array[index]->key = key;
  //Up Heap while not heapified
  while(index && (minHeap->array[index]->key < minHeap->array[(index - 1) / 2]->key)) {

    // Swap this node with its parent
    minHeap->position[minHeap->array[index]->vertex] = (index-1)/2;
    minHeap->position[minHeap->array[(index-1)/2]->vertex] = index;
    swapMinHeapNode(&minHeap->array[index],  &minHeap->array[(index - 1) / 2]);
    //move to parent index
    index = (index - 1)/2;
  
  }

}

// A utility function to check if a given vertex  is in min heap or not
bool isInMinHeap(struct MinHeap *minHeap, int vertex) {
  
  if (minHeap->position[vertex] < minHeap->size) {

    return true;

  }
  
  return false;

}

// A utility function to find the minimum distance value, from the set 
// of vertices not yet included in shortest path tree
int minDistance (int dist[], bool sptset[], int V) {

  // Initialise min value
  int min = INT_MAX;
  int min_index;
  for (int v = 0; v < V; v++) {

    if (sptSet[v] == false && dist[v] < min) {

      min = dist[v];
      min_index = v;

    }

  }
  
  return min_index;

}

// Function that implements Dijkstra's single source shortest path
// algorithm when graph is represented as adjacency matrix
void dijkstra ( struct Graph* graph, int V,int src) {

  int dist[V];  // The output array.dist[i] will hold the shortest 
                // distance from sorce to i.
  bool sptSet[V]; // sptSet[i] will be  true if vertex i is included
                  // in SPT tree or sorest dstance from src to i finalised
  //Initialise all shortest distances to INFIITY and sptet to false
  for ( int i = 0; i < V; i++){

    dist[i] = INT_MAX;
    sptSet[i] = false;

  }

  // Set src ditance to zero
  dist[src] = 0;
  // Find the shortest paths for all vertices
  for (int count = 0; count < V - 1; count++) {

    // Pick the minimum distance vertex from the set of vertices not yet
    // processed .u is always equal to src in first iteraton.
    int u = minDistance(dist, sptSet);
    // Mark the picked vertex as processed
    sptSet[u] = true;
    // Update the dist values of the picked vertices
    for ( int v = 0; v < V; v++) {

      // Update dist[v] only if is no in sptSet, there is an edge from u to
      // v and the total weight of the path from src to v through u is smaller
      // than the curren value of dist[v].   
  
    

int main ( int argc, char** argv) {

  int N;  //number of cities;suppose 3 <= K <= N <= 60000
  int K;  //number of presidential stations
  int M;  //number of bidirectional roads N - 1 <= M <= 300000
  int A;  //the index of teleportable planet
  int B;  //the index of the one connected planet
  int T;  //the index of the other connected planet
  int K;
  int c_temp;
  int u;
  int v;
  int d_uv;
  scanf( "%d", &N);
  scanf( "%d", &M);
  scanf( "%d", &A);
  scanf( "%d", &B);
  scanf( "%d", &T);
  scanf( "%d", &K);

  for ( int i = 0; i < K; i++) {

    scanf( "%d", &c_temp);
    c[i] = c_temp;

  }

  struct Graph* city_map = newGraph(N);
  for ( int i = 0; i < M; i++) {

    scanf( "%d", &u);	  
    scanf( "%d", &v);	  
    scanf( "%d", &d_uv);
    addEdge(city_map, u, v, d_uv);

  }

  return 0;

}

