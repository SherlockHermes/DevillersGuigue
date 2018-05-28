#include "Types.h"
#include "DandG.h"
#define F_MAX 9999999
#define F_MIN -9999999

typedef std::pair<int, TriangleEigen3f> I_T;
typedef std::pair<int, int> pair_int;

struct AABB_NODE {
	std::vector<int> t_tab;//包含的三角形
	int numNodes;
	PointEigen3f cone_vector;
	float cone_angle;
	AABB_NODE *left;
	AABB_NODE *right;
	float box[6];
};

class AABB_Tree_cone {
public:
	AABB_NODE *root;
	std::vector<TriangleEigen3f> triangles;

public:
	AABB_Tree_cone(const std::vector<TriangleEigen3f>& triangles);
	~AABB_Tree_cone();

	void createTree(AABB_NODE* &root,const std::vector<I_T>& i_triangles);


	void SELF_COLLISION(AABB_NODE* &root, std::vector<pair_int>& collision_pair);
	
};


bool BOX_COLLISION(float *box1, float *box2);
void COLLISION(AABB_NODE* &root1, AABB_NODE* &root2,const std::vector<TriangleEigen3f> &triangles1, const std::vector<TriangleEigen3f> &triangles2,std::vector<pair_int>& collision_pair);
void AABB_BOX(const std::vector<TriangleEigen3f>& triangles, float* box);
void CONE(const std::vector<TriangleEigen3f>& triangles, float& cone_angle, PointEigen3f& cone_vector);
void ANGLE(PointEigen3f& cone_vector1, PointEigen3f& cone_vector2, float& angle);
void PreOrder(AABB_NODE* &root);

void root_data(AABB_NODE* &root);

bool has_same_point(const TriangleEigen3f &t1,const TriangleEigen3f &t2);

//自碰撞flag为0，两棵树碰撞flag为1
void sort_pair(std::vector<pair_int> &pis,int flag);