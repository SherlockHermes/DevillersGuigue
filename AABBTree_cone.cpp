#include "AABBTree_cone.h"
#define pi 3.14159265359


//注：FLT_MIN为正值，如使用float类型最小值，写-FLT_MAX。被坑太多次，放弃了这种写法

AABB_Tree_cone::AABB_Tree_cone(const std::vector<TriangleEigen3f>& triangles)
{
	this->root = new AABB_NODE;
	this->root->numNodes = triangles.size();
	this->root->cone_vector = PointEigen3f(0.0, 0.0, 0.0);
	this->root->cone_angle = 0;
	this->root->left = NULL;
	this->root->right = NULL;
	this->root->box[0] = F_MAX;
	this->root->box[1] = F_MAX;
	this->root->box[2] = F_MAX;
	this->root->box[3] = F_MIN;
	this->root->box[4] = F_MIN;
	this->root->box[5] = F_MIN;

	this->triangles.assign(triangles.begin(), triangles.end());

	std::vector<I_T> i_triangles;

	int i = 0;
	for (auto t:triangles)
	{	
		i_triangles.push_back(I_T(i,t));
		this->root->t_tab.push_back(i);

		float box[6];
		box[0] = F_MAX;
		box[1] = F_MAX;
		box[2] = F_MAX;
		box[3] = F_MIN;
		box[4] = F_MIN;
		box[5] = F_MIN;


		for (int j = 0; j < 3; j++)
		{
			//min value
			if (t.vertex[j][0] < box[0]) box[0] = t.vertex[j][0];
			if (t.vertex[j][1] < box[1]) box[1] = t.vertex[j][1];
			if (t.vertex[j][2] < box[2]) box[2] = t.vertex[j][2];
			//max value
			if (t.vertex[j][0] > box[3]) box[3] = t.vertex[j][0];
			if (t.vertex[j][1] > box[4]) box[4] = t.vertex[j][1];
			if (t.vertex[j][2] > box[5]) box[5] = t.vertex[j][2];
		}

		//root->box
		{
			//min value
			if (box[0] < this->root->box[0]) this->root->box[0] = box[0];
			if (box[1] < this->root->box[1]) this->root->box[1] = box[1];
			if (box[2] < this->root->box[2]) this->root->box[2] = box[2];
			//max value
			if (box[3] > this->root->box[3]) this->root->box[3] = box[3];
			if (box[4] > this->root->box[4]) this->root->box[4] = box[4];
			if (box[5] > this->root->box[5]) this->root->box[5] = box[5];
		}
		i++;
	}
	createTree(this->root, i_triangles);
}

void AABB_Tree_cone::createTree(AABB_NODE* &root, const std::vector<I_T>& i_triangles)
{
	if (i_triangles.size() == 1)
	{
		std::vector<TriangleEigen3f> temp;
		temp.push_back(i_triangles[0].second);
		CONE(temp, root->cone_angle, root->cone_vector);
		return;
	}
		

	int axis = 0;
	if ((root->box[3] - root->box[0] >= root->box[4] - root->box[1]) && (root->box[3] - root->box[0] >= root->box[5] - root->box[2]))
		axis = 0;
	else if ((root->box[4] - root->box[1] >= root->box[3] - root->box[0]) && (root->box[4] - root->box[1] >= root->box[5] - root->box[2]))
		axis = 1;
	else
		axis = 2;

	float plane = ((root->box[axis + 3] + root->box[axis]) / 2);
	std::vector<I_T> half1, half2;
	std::vector<TriangleEigen3f> half1_t, half2_t;
	AABB_NODE *new_node1=new AABB_NODE;
	//new_node1->t_tab;
	new_node1->numNodes = 0;
	new_node1->cone_vector = PointEigen3f(0.0, 0.0, 0.0);
	new_node1->cone_angle = 0;
	new_node1->left = NULL;
	new_node1->right = NULL;

	new_node1->box[0] = F_MAX;
	new_node1->box[1] = F_MAX;
	new_node1->box[2] = F_MAX;
	new_node1->box[3] = F_MIN;
	new_node1->box[4] = F_MIN;
	new_node1->box[5] = F_MIN;

	AABB_NODE *new_node2 = new AABB_NODE;
	//new_node2->t_tab;
	new_node2->numNodes = 0;
	new_node2->cone_vector = PointEigen3f(0.0, 0.0, 0.0);
	new_node2->cone_angle = 0;
	new_node2->left = NULL;
	new_node2->right = NULL;

	new_node2->box[0] = F_MAX;
	new_node2->box[1] = F_MAX;
	new_node2->box[2] = F_MAX;
	new_node2->box[3] = F_MIN;
	new_node2->box[4] = F_MIN;
	new_node2->box[5] = F_MIN;


	for (auto t : i_triangles)
	{
		if (((t.second.vertex[0][axis] + t.second.vertex[1][axis] + t.second.vertex[2][axis]) / 3 )<=plane)
		{
			(new_node1->numNodes)++;
			(new_node1->t_tab).push_back(t.first);
			half1.push_back(t);
			half1_t.push_back(t.second);
		}
		else
		{
			(new_node2->numNodes)++;
			(new_node2->t_tab).push_back(t.first);
			half2.push_back(t);
			half2_t.push_back(t.second);
		}
	}

	if (half1_t.empty())
	{
		AABB_BOX(half2_t, new_node2->box);
		CONE(half2_t,new_node2->cone_angle,new_node2->cone_vector);
		root = new_node2;
		return;
	}

	if (half2_t.empty())
	{
		AABB_BOX(half1_t, new_node1->box);
		CONE(half1_t, new_node1->cone_angle, new_node1->cone_vector);
		root = new_node1;
		return;
	}


	AABB_BOX(half1_t, new_node1->box);
	AABB_BOX(half2_t, new_node2->box);
	root->left = new_node1;
	root->right = new_node2;

	createTree(root->left, half1);
	createTree(root->right, half2);


	float cosr = root->left->cone_vector.dot(root->right->cone_vector) / root->left->cone_vector.norm() / root->right->cone_vector.norm();

	root->cone_angle = std::max(root->left->cone_angle, root->right->cone_angle)+acos(cosr)*180*0.5/pi;
	root->cone_vector = PointEigen3f((root->left->cone_vector[0]+ root->right->cone_vector[0])*05,
		(root->left->cone_vector[1] + root->right->cone_vector[1]) * 0.5,
		(root->left->cone_vector[2] + root->right->cone_vector[2]) * 0.5);
	root->cone_vector = root->cone_vector.normalized();
}


AABB_Tree_cone::~AABB_Tree_cone()
{
}


void AABB_BOX(const std::vector<TriangleEigen3f>& triangles, float* box)
{
	box[0] = F_MAX;
	box[1] = F_MAX;
	box[2] = F_MAX;
	box[3] = F_MIN;
	box[4] = F_MIN;
	box[5] = F_MIN;
	for (auto t : triangles)
	{
		for (int j = 0; j < 3; j++)
		{
			//min value
			if (t.vertex[j][0] < box[0]) box[0] = t.vertex[j][0];
			if (t.vertex[j][1] < box[1]) box[1] = t.vertex[j][1];
			if (t.vertex[j][2] < box[2]) box[2] = t.vertex[j][2];
			//max value
			if (t.vertex[j][0] > box[3]) box[3] = t.vertex[j][0];
			if (t.vertex[j][1] > box[4]) box[4] = t.vertex[j][1];
			if (t.vertex[j][2] > box[5]) box[5] = t.vertex[j][2];
		}
	}
}

void CONE(const std::vector<TriangleEigen3f>& triangles, float& cone_angle, PointEigen3f& cone_vector)
{
	PointEigen3f v1, v2;
	cone_angle = 0;
	int cot = triangles.size();
	for (int i=0; i < cot-1;i++)
	{
		for (int j = i + 1; j < cot; j++)
		{
			v1 = (triangles[i].vertex[0] - triangles[i].vertex[2]).cross(triangles[i].vertex[1] - triangles[i].vertex[2]).normalized();
			v2 = (triangles[j].vertex[0] - triangles[j].vertex[2]).cross(triangles[j].vertex[1] - triangles[j].vertex[2]).normalized();
			float temp;
			ANGLE(v1, v2, temp);
			if (temp > cone_angle)
			{
				cone_vector = PointEigen3f((v1[0] + v2[0]) / 2,(v1[1] + v2[1]) / 2,(v1[2] + v2[2]) / 2);
				cone_vector = cone_vector.normalized();
				cone_angle = temp;
			}
				
		}
	}
	cone_angle = cone_angle*0.5;

	//当叶结点有多个三角形时，计算法向量角较难确定，所以默认为91度
	if (cot == 1)
	{
		cone_vector = (triangles[0].vertex[0] - triangles[0].vertex[2]).cross(triangles[0].vertex[1] - triangles[0].vertex[2]).normalized();
		cone_angle = 91;
	}
}

void ANGLE(PointEigen3f& cone_vector1, PointEigen3f& cone_vector2, float& angle)
{
	float cosr = cone_vector1.dot(cone_vector2) / cone_vector1.norm() / cone_vector2.norm();
	angle = acos(cosr) * 180 / pi;
}

void PreOrder(AABB_NODE* &root)
{
	if (root)
	{
		if (!root->left && root->numNodes>1 )
		{
			std::cout << "num:" << root->numNodes << std::endl;

			std::cout << "triangle:";
			for (auto t : root->t_tab)
				std::cout << t << "    ";
			std::cout << std::endl;

			std::cout << "box:";
			for (int i = 0; i < 6; i++)
				std::cout << root->box[i] << "    ";
			std::cout << std::endl;

			std::cout << "angle:" << root->cone_angle << std::endl;

			std::cout << "cone_victor:";
			for (int i = 0; i < 3; i++)
				std::cout << root->cone_vector[i] << "    ";
			std::cout << std::endl;
			std::cout << std::endl;
		}
		

		PreOrder(root->left);
		PreOrder(root->right);
	}
}

void root_data(AABB_NODE* &root)
{
		std::cout << "num:" << root->numNodes << std::endl;

		/*std::cout << "triangle:";
		for (auto t : root->t_tab)
			std::cout << t << "    ";
		std::cout << std::endl;*/

		std::cout << "box:";
		for (int i = 0; i < 6; i++)
			std::cout << root->box[i] << "    ";
		std::cout << std::endl;

		std::cout << "angle:" << root->cone_angle << std::endl;

		std::cout << "cone_victor:";
		for (int i = 0; i < 3; i++)
			std::cout << root->cone_vector[i] << "    ";
		std::cout << std::endl;
		std::cout << std::endl;

}

bool BOX_COLLISION(float *box1, float *box2)
{
	if (box1[3]<box2[0] || box1[0]>box2[3]) return false;
	if (box1[4]<box2[1] || box1[1]>box2[4]) return false;
	if (box1[5]<box2[2] || box1[2]>box2[5]) return false;
	return true;
}


//自碰撞
//注：由于精度问题(猜测)，判断两个三角形相交时，传出A,B和传入B,A结果不同，所以都判断一下。
void AABB_Tree_cone::SELF_COLLISION(AABB_NODE* &root, std::vector<pair_int>& collision_pair)
{
	if (!root)
		return;

	//叶结点存在多个三角形
	if (!root->left)
	{
		for (int i = 0; i < root->numNodes - 1; i++)
		{
			for (int j = i + 1; j < root->numNodes; j++)
			{
				if (has_same_point(this->triangles[root->t_tab[i]], this->triangles[root->t_tab[j]]))
					continue;

				if (judge_triangle_topologicalStructure(&(this->triangles[root->t_tab[i]]), &(this->triangles[root->t_tab[j]]))  ||
					judge_triangle_topologicalStructure(&(this->triangles[root->t_tab[j]]), &(this->triangles[root->t_tab[i]])))
				{
					pair_int temp = pair_int(root->t_tab[i], root->t_tab[j]);
					collision_pair.push_back(temp);
				}
			}
		}
		return;
	}

	/*if (root->cone_angle < 90)
		return;*/

	COLLISION(root->left, root->right, this->triangles,this->triangles,collision_pair);

	SELF_COLLISION(root->left, collision_pair);
	SELF_COLLISION(root->right, collision_pair);
}


//两个节点之间碰撞
void COLLISION(AABB_NODE* &root1, AABB_NODE* &root2, const std::vector<TriangleEigen3f> &triangles1, const std::vector<TriangleEigen3f> &triangles2, std::vector<pair_int>& collision_pair)
{
	if (root1 == NULL || root2 == NULL)
		return;


	if (!BOX_COLLISION(root1->box, root2->box))
		return;


	if (root1->left == NULL  && root2->left == NULL)
	{
		//叶结点当中含有多个三角形

		for (int i=0; i < root1->numNodes; i++)
		{
			for (int j=0; j < root2->numNodes; j++)
			{
				if (has_same_point(triangles1[root1->t_tab[i]], triangles2[root2->t_tab[j]]))
					continue;

				if ( judge_triangle_topologicalStructure(&(triangles1[root1->t_tab[i]]), &(triangles2[root2->t_tab[j]])) || 
					 judge_triangle_topologicalStructure(&(triangles2[root2->t_tab[j]]), &(triangles1[root1->t_tab[i]])))
				{
					pair_int temp = pair_int(root1->t_tab[i], root2->t_tab[j]);
					collision_pair.push_back(temp);
				}
			}
		}
		return;
	}

	if (root1->left  && root2->left == NULL)
	{
		COLLISION(root1->left, root2, triangles1, triangles2,collision_pair);
		COLLISION(root1->right, root2, triangles1, triangles2,collision_pair);
		return;
	}
	
	if (root1->left == NULL  && root2->left)
	{
		COLLISION(root1, root2->left, triangles1, triangles2, collision_pair);
		COLLISION(root1, root2->right, triangles1, triangles2, collision_pair);
		return;
	}

	COLLISION(root1->left, root2, triangles1, triangles2, collision_pair);
	COLLISION(root1->right, root2, triangles1, triangles2, collision_pair);

}


bool has_same_point(const TriangleEigen3f &t1,const TriangleEigen3f &t2)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (t1.vertex[i] == t2.vertex[j])
				return true;
		}
	}
	return false;
}


















void sort_pair(std::vector<pair_int> &pis,int flag)
{
	if (flag == 0)
	{
		for (int i = 0; i < pis.size(); i++)
		{
			if (pis[i].first>pis[i].second)
			{
				pis[i].swap(pis[i]);
			}
		}
	}
	

	for (int i = 0; i < pis.size(); i++)
	{
		for (int j = i + 1; j < pis.size(); j++)
		{
			if (pis[i].first > pis[j].first)
			{
				pair_int temp;
				temp = pis[i];
				pis[i] = pis[j];
				pis[j] = temp;
			}
		}
	}

	int s = 0;
	int e = -1;
	for (int p = 1; p < pis.size(); p++)
	{
		if (pis[p].first != pis[p - 1].first)
		{
			s = e + 1;
			e = p-1;

			for (int i = s; i < e+1; i++)
			{
				for (int j = i + 1; j < e+1 ; j++)
				{
					if (pis[i].second > pis[j].second)
					{
						pair_int temp;
						temp = pis[i];
						pis[i] = pis[j];
						pis[j] = temp;
					}
				}
			}
		}
	}
}