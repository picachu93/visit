/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

* File Name : visit.c

* Purpose : Educational

* Creation Date : 02-03-2017

* Last Modified : Κυρ 05 Μάρ 2017 07:34:01 μμ EET

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

// A utility function used to print the solution
void printArr(int dist[], int n) {
    
  printf("Vertex   Distance from Source\n");
  for (int i = 0; i < n; ++i) {

    printf("%d \t\t %d\n", i, dist[i]);

  }

}

// Function that implements Dijkstra's single source shortest path
// algorithm when graph is represented as adjacency list.It is a 
// O(E*logV) function 
void dijkstra ( struct Graph* graph, int src) {

  int V = graph->V;
  int dist[V];  // The output array.dist[i] will hold the shortest 
                // distance from sorce to i.
  // min heap represents the set Q
  struct MinHeap* minHeap = newMinHeap(V);
  // Initialize min heap with all vertices. dist value of all vertices
  for ( int v = 0; v < V; v++){

    dist[v] = INT_MAX;
    minHeap->array[v] = newMinHeapNode(v, dist[v]);
    minHeap->position[v] = v;

  }
  
  // Make dist value of src vertex as 0 so that it is extracted first
  minHeap->array[src] = newMinHeapNode( src, dist[src]);
  minHeap->position[src] = src;
  dist[src] = 0;
  decreaseKey(minHeap, src, dist[src]);
  // Initially size of min heap is equal to V
  minHeap->size = V;
  // In the following loop min heap contains all vertices whose shortest distances 
  // no yet finished
  while (!isEmpty(minHeap)) {

    //Extract the vertex with minimum distance value
    struct MinHeapNode* minHeapNode = extractMin(minHeap);
    int u = minHeapNode->vertex;  //store the extracted vertex number
    //Traverse through all adjacent vertices of u and update their distance values
    struct AdjListNode* pCrawl = graph->array[u].head;
    while (pCrawl != NULL) {

      int v = pCrawl->dest;
    // If shortest distance to v is not finalized yet, and distance to v
    // through u is less than its previously calculated distance
      if ( isInMinHeap(minHeap, v) && dist[v] != INT_MAX 
	                           && dist[u] + pCrawl->weight < dist[v]) {
      
        dist[v] = dist[u] + pCrawl->weight;
        decreaseKey(minHeap, v, dist[v]);

      }

      pCrawl = pCrawl->next;

    }

  }

  printArr(dist, V);

}  
  
    

int main ( int argc, char** argv) {

/*  int N;  //number of cities;suppose 3 <= K <= N <= 60000
  int M;  //number of bidirectional roads N - 1 <= M <= 300000
  int A;  //the index of teleportable planet
  int B;  //the index of the one connected planet
  int T;  //the index of the other connected planet
  int K;  //number of presidential stations
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

  } */
  
   // create the graph given in above fugure
    int V = 9;
    struct Graph* graph = newGraph(V);
    addEdge(graph, 0, 1, 4);
    addEdge(graph, 0, 7, 8);
    addEdge(graph, 1, 2, 8);
    addEdge(graph, 1, 7, 11);
    addEdge(graph, 2, 3, 7);
    addEdge(graph, 2, 8, 2);
    addEdge(graph, 2, 5, 4);
    addEdge(graph, 3, 4, 9);
    addEdge(graph, 3, 5, 14);
    addEdge(graph, 4, 5, 10);
    addEdge(graph, 5, 6, 2);
    addEdge(graph, 6, 7, 1);
    addEdge(graph, 6, 8, 6);
    addEdge(graph, 7, 8, 7);
  dijkstra(graph, 0);
  return 0;

}

