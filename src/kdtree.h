/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node==NULL)
		{
			// std::cout<<"null Node: node-x:" << (*node)->point[0] << " node-y:" << (*node)->point[1] <<std::endl;
			*node = new Node(point,id);	
			// std::cout<<"null Node: point-x:" << point[0] << " point-y:" << point[1] <<std::endl;		
		}
		else
		{
			uint cd = depth%2;

			if(point[cd] < ((*node)->point[cd]))
			{
				// std::cout<<"left Node: node-x: node-y:"  <<std::endl;
				insertHelper(&((*node)->left), depth+1, point, id);
				// std::cout<<"left Node: point-x:" << point[0] << " point-y:" << point[1] <<std::endl;	
			}
			else
			{
				// std::cout<<"right Node: node-x:  node-y:" <<std::endl;
				insertHelper(&((*node)->right), depth+1, point, id);
				// std::cout<<"right Node: point-x:" << point[0] << " point-y:" << point[1] <<std::endl;	
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(Node *node, std::vector<float> target, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if (node!=NULL)
		{
			if ( (node->point[0]>=(target[0]-distanceTol))&& (node->point[0]<=(target[0]+distanceTol)) && 
			(node->point[1]>=(target[1]-distanceTol)) && (node->point[1]<=(target[1]+distanceTol)) && 
			(node->point[2]>=(target[2]-distanceTol))&& (node->point[2]<=(target[2]+distanceTol)) )
			{
				float distance = sqrt( ( (node->point[0]-target[0])*(node->point[0]-target[0]) ) + 
										( (node->point[1]-target[1])*(node->point[1]-target[1])) + 
										( (node->point[2]-target[2])*(node->point[2]-target[2])) );
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			if ( (target[depth%2]-distanceTol) <= node->point[depth%2])
			{
				searchHelper(node->left,target,depth+1, distanceTol, ids);
			}
			if( (target[depth%2]+distanceTol) >= node->point[depth%2])
			{
				searchHelper(node->right,target,depth+1,distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, 0, distanceTol ,ids);
		return ids;
	}
	

};




