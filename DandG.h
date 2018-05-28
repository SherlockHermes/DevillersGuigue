#define DandG
#ifdef DandG
//Devillers & Guigue算法

#define CGAL_EIGEN3_ENABLED
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "Types.h"
using namespace Eigen;
//using namespace std;


struct point
{
	float x, y;
};
static void copy_point(point& p, PointEigen3f f, int flag)
{
	if (flag == 0 || flag == 1)
	{
		p.x = f[0];
		p.y = f[1];
	}
	else if (flag == 2)
	{
		p.x = f[0];
		p.y = f[2];
	}
	else
	{
		p.x = f[1];
		p.y = f[2];
	}
}


//四点行列式 
inline float get_vector4_det(PointEigen3f v1, PointEigen3f v2, PointEigen3f v3, PointEigen3f v4)
{
	Matrix4d A;
	A << v1[0], v1[1], v1[2], 1,
		 v2[0], v2[1], v2[2], 1,
		 v3[0], v3[1], v3[2], 1,
		 v4[0], v4[1], v4[2], 1;
	//cout << "Here is a matrix, A:" << endl << A << endl << endl;
	double x = A.determinant();
	//std::cout << "行列式：\n" << x<<std::endl;
	return float(x);
	//float a[3][3];
	//for (int i = 0; i != 3; ++i)
	//{
	//	a[0][i] = v1[i] - v4[i];
	//	a[1][i] = v2[i] - v4[i];
	//	a[2][i] = v3[i] - v4[i];
	//}

	//return a[0][0] * a[1][1] * a[2][2]
	//	+ a[0][1] * a[1][2] * a[2][0]
	//	+ a[0][2] * a[1][0] * a[2][1]
	//	- a[0][2] * a[1][1] * a[2][0]
	//	- a[0][1] * a[1][0] * a[2][2]
	//	- a[0][0] * a[1][2] * a[2][1];
}

inline double direction(point p1, point p2, point p) {
	return (p.x - p1.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p.y - p1.y);
}

//确定与线段p1p2共线的点p是否在线段p1p2上 
inline int on_segment(point p1, point p2, point p) {
	double max = p1.x > p2.x ? p1.x : p2.x;
	double min = p1.x < p2.x ? p1.x : p2.x;
	double max1 = p1.y > p2.y ? p1.y : p2.y;
	double min1 = p1.y < p2.y ? p1.y : p2.y;
	if (p.x >= min && p.x <= max && p.y >= min1 && p.y <= max1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
inline int on_segment_3d(PointEigen3f p1, PointEigen3f p2, PointEigen3f p) {
	float maxx = p1[0] > p2[0] ? p1[0] : p2[0];
	float minx = p1[0] < p2[0] ? p1[0] : p2[0];
	float maxy = p1[1] > p2[1] ? p1[1] : p2[1];
	float miny = p1[1] < p2[1] ? p1[1] : p2[1];
	float maxz = p1[2] > p2[2] ? p1[2] : p2[2];
	float minz = p1[2] < p2[2] ? p1[2] : p2[2];
	if (p[0] >= minx && p[0] <= maxx && p[1] >= miny && p[1] <= maxy && p[2] >= minz && p[2] <= maxz)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//判断线段p1p2与线段p3p4是否相交的主函数  
inline int segments_intersert(point p1, point p2, point p3, point p4) {
	double d1, d2, d3, d4;
	d1 = direction(p3, p4, p1);
	d2 = direction(p3, p4, p2);
	d3 = direction(p1, p2, p3);
	d4 = direction(p1, p2, p4);
	if (d1 * d2 < 0 && d3 * d4 < 0)
	{
		return 1;
	}
	else if (d1 == 0 && on_segment(p3, p4, p1) == 1)
	{
		return 1;
	}
	else if (d2 == 0 && on_segment(p3, p4, p2) == 1)
	{
		return 1;
	}
	else if (d3 == 0 && on_segment(p1, p2, p3) == 1)
	{
		return 1;
	}
	else if (d4 == 0 && on_segment(p1, p2, p4) == 1)
	{
		return 1;
	}
	return 0;
}

//判断同一平面的直线和三角形是否相交  
inline bool line_triangle_intersert_inSamePlane(const TriangleEigen3f* tri,const PointEigen3f f1, PointEigen3f f2)
{
	int pro_flag = 0;

	auto normal = (tri->vertex[0] - tri->vertex[2]).cross(tri->vertex[1] - tri->vertex[2]);

	if (normal.x() == 0 && normal.y() == 0)
		pro_flag = 1;

	if (normal.x() == 0 && normal.z() == 0)
		pro_flag = 2;

	if (normal.y() == 0 && normal.z() == 0)
		pro_flag = 3;


	point p1, p2, p3, p4;

	copy_point(p1, f1, pro_flag);

	copy_point(p2, f2, pro_flag);

	copy_point(p3, tri->vertex[0], pro_flag);

	copy_point(p4, tri->vertex[1], pro_flag);

	if (segments_intersert(p1, p2, p3, p4))
	{
		return true;
	}

	copy_point(p3, tri->vertex[1], pro_flag);

	copy_point(p4, tri->vertex[2], pro_flag);

	if (segments_intersert(p1, p2, p3, p4))
	{
		return true;
	}

	copy_point(p3, tri->vertex[0], pro_flag);

	copy_point(p4, tri->vertex[2], pro_flag);

	if (segments_intersert(p1, p2, p3, p4))
	{
		return true;
	}

	return false;
}

//向量之差  
inline void get_vector_diff(PointEigen3f& aimV, const PointEigen3f a, const PointEigen3f b)
{
	aimV[0] = b[0] - a[0];

	aimV[1] = b[1] - a[1];

	aimV[2] = b[2] - a[2];
}

//向量内积
inline float Dot(const PointEigen3f& v1, const PointEigen3f& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


inline void get_central_point(PointEigen3f centralPoint,const TriangleEigen3f* tri)
{
	centralPoint[0] = (tri->vertex[0][0] + tri->vertex[1][0] + tri->vertex[2][0]) / 3;

	centralPoint[1] = (tri->vertex[0][1] + tri->vertex[1][1] + tri->vertex[2][1]) / 3;

	centralPoint[2] = (tri->vertex[0][2] + tri->vertex[1][2] + tri->vertex[2][2]) / 3;
}

//重心法判断点是否在三角形内部  
inline bool is_point_within_triangle(const TriangleEigen3f* tri, PointEigen3f point)
{
	PointEigen3f v0;
	get_vector_diff(v0, tri->vertex[0], tri->vertex[2]);
	PointEigen3f v1;
	get_vector_diff(v1, tri->vertex[0], tri->vertex[1]);
	PointEigen3f v2;
	get_vector_diff(v2, tri->vertex[0], point);
	float dot00 = Dot(v0, v0);
	float dot01 = Dot(v0, v1);
	float dot02 = Dot(v0, v2);
	float dot11 = Dot(v1, v1);
	float dot12 = Dot(v1, v2);
	float inverDeno = 1 / (dot00* dot11 - dot01* dot01);
	float u = (dot11* dot02 - dot01* dot12) * inverDeno;
	if (u < 0 || u > 1) // if u out of range, return directly  
	{
		return false;
	}
	float v = (dot00* dot12 - dot01* dot02) * inverDeno;
	if (v < 0 || v > 1) // if v out of range, return directly  
	{
		return false;
	}
	return u + v <= 1;
}

//判断同一平面内的三角形是否相交  
inline bool triangle_intersert_inSamePlane(const TriangleEigen3f* tri1,const TriangleEigen3f* tri2)
{
	if (line_triangle_intersert_inSamePlane(tri2, tri1->vertex[0], tri1->vertex[1]))
	{
		return true;
	}
	else if (line_triangle_intersert_inSamePlane(tri2, tri1->vertex[1], tri1->vertex[2]))
	{
		return true;
	}
	else if (line_triangle_intersert_inSamePlane(tri2, tri1->vertex[0], tri1->vertex[2]))
	{
		return true;
	}
	else
	{
		PointEigen3f centralPoint1, centralPoint2;
		get_central_point(centralPoint1, tri1);
		get_central_point(centralPoint2, tri2);
		if (is_point_within_triangle(tri2, centralPoint1) || is_point_within_triangle(tri1, centralPoint2))
		{
			return true;
		}
		return false;
	}
}





//Devillers算法主函数  
inline bool judge_triangle_topologicalStructure(const TriangleEigen3f *tri1,const TriangleEigen3f *tri2)
{
	//设tri1所在的平面为p1,tri2所在的平面为p2  
	float p1_tri2_vertex1 = get_vector4_det(tri1->vertex[0], tri1->vertex[1], tri1->vertex[2], tri2->vertex[0]);

	float p1_tri2_vertex2 = get_vector4_det(tri1->vertex[0], tri1->vertex[1], tri1->vertex[2], tri2->vertex[1]);

	float p1_tri2_vertex3 = get_vector4_det(tri1->vertex[0], tri1->vertex[1], tri1->vertex[2], tri2->vertex[2]);


	if (p1_tri2_vertex1 > 0 && p1_tri2_vertex2 > 0 && p1_tri2_vertex3 > 0)
	{
		return false;
	}

	if (p1_tri2_vertex1 < 0 && p1_tri2_vertex2 < 0 && p1_tri2_vertex3 < 0)
	{
		return false;
	}


	if (p1_tri2_vertex1 == 0 && p1_tri2_vertex2 == 0 && p1_tri2_vertex3 == 0)
	{
		if (triangle_intersert_inSamePlane(tri1, tri2))
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	if (p1_tri2_vertex1 == 0 && p1_tri2_vertex2 * p1_tri2_vertex3 > 0)
	{
		if (is_point_within_triangle(tri1, tri2->vertex[0]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (p1_tri2_vertex2 == 0 && p1_tri2_vertex1 * p1_tri2_vertex3 > 0)
	{
		if (is_point_within_triangle(tri1, tri2->vertex[1]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (p1_tri2_vertex3 == 0 && p1_tri2_vertex1 * p1_tri2_vertex2 > 0)
	{
		if (is_point_within_triangle(tri1, tri2->vertex[2]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}



	float p2_tri1_vertex1 = get_vector4_det(tri2->vertex[0], tri2->vertex[1], tri2->vertex[2], tri1->vertex[0]);

	float p2_tri1_vertex2 = get_vector4_det(tri2->vertex[0], tri2->vertex[1], tri2->vertex[2], tri1->vertex[1]);

	float p2_tri1_vertex3 = get_vector4_det(tri2->vertex[0], tri2->vertex[1], tri2->vertex[2], tri1->vertex[2]);


	if (p2_tri1_vertex1 > 0 && p2_tri1_vertex2 > 0 && p2_tri1_vertex3 > 0)
	{
		return false;
	}

	if (p2_tri1_vertex1 < 0 && p2_tri1_vertex2 < 0 && p2_tri1_vertex3 < 0)
	{
		return false;
	}


	if (p2_tri1_vertex1 == 0 && p2_tri1_vertex2 * p2_tri1_vertex3 > 0)
	{
		if (is_point_within_triangle(tri2, tri1->vertex[0]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	if (p2_tri1_vertex2 == 0 && p2_tri1_vertex1 * p2_tri1_vertex3 > 0)
	{
		if (is_point_within_triangle(tri2, tri1->vertex[1]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	if (p2_tri1_vertex3 == 0 && p2_tri1_vertex1 * p2_tri1_vertex2 > 0)
	{
		if (is_point_within_triangle(tri2, tri1->vertex[2]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}



	PointEigen3f tri1_a = tri1->vertex[0], tri1_b = tri1->vertex[1], tri1_c = tri1->vertex[2]
			   , tri2_a = tri2->vertex[0], tri2_b = tri2->vertex[1], tri2_c = tri2->vertex[2];

	PointEigen3f m;

	float im;

	if (p2_tri1_vertex2 * p2_tri1_vertex3 >= 0 && p2_tri1_vertex1 != 0)
	{
		if (p2_tri1_vertex1 < 0)
		{
			m = tri2_b;
			tri2_b = tri2_c;
			tri2_c = m;

			im = p1_tri2_vertex2;
			p1_tri2_vertex2 = p1_tri2_vertex3;
			p1_tri2_vertex3 = im;

			
		}
	}
	else if (p2_tri1_vertex1 * p2_tri1_vertex3 >= 0 && p2_tri1_vertex2 != 0)
	{
		m = tri1_a;
		tri1_a = tri1_b;
		tri1_b = tri1_c;
		tri1_c = m;

		if (p2_tri1_vertex2 < 0)
		{
			m = tri2_b;
			tri2_b = tri2_c;
			tri2_c = m;

			im = p1_tri2_vertex2;
			p1_tri2_vertex2 = p1_tri2_vertex3;
			p1_tri2_vertex3 = im;
		}
	}
	else if (p2_tri1_vertex1 * p2_tri1_vertex2 >= 0 && p2_tri1_vertex3 != 0)
	{
		m = tri1_a;

		tri1_a = tri1_c;

		tri1_c = tri1_b;

		tri1_b = m;

		if (p2_tri1_vertex3 < 0)
		{
			m = tri2_b;
			tri2_b = tri2_c;
			tri2_c = m;

			im = p1_tri2_vertex2;
			p1_tri2_vertex2 = p1_tri2_vertex3;
			p1_tri2_vertex3 = im;
		}
	}

	if (p1_tri2_vertex2 * p1_tri2_vertex3 >= 0 && p1_tri2_vertex1 != 0)
	{
		if (p1_tri2_vertex1 < 0)
		{
			m = tri1_b;
			tri1_b = tri1_c;
			tri1_c = m;
		}
	}
	else if (p1_tri2_vertex1 * p1_tri2_vertex3 >= 0 && p1_tri2_vertex2 != 0)
	{
		m = tri2_a;

		tri2_a = tri2_b;

		tri2_b = tri2_c;

		tri2_c = m;

		if (p1_tri2_vertex2 < 0)
		{
			m = tri1_b;
			tri1_b = tri1_c;
			tri1_c = m;
		}
	}
	else if (p1_tri2_vertex1 * p1_tri2_vertex2 >= 0 && p1_tri2_vertex3 != 0)
	{
		m = tri2_a;

		tri2_a = tri2_c;

		tri2_c = tri2_b;

		tri2_b = m;

		if (p1_tri2_vertex3 < 0)
		{
			m = tri1_b;
			tri1_b = tri1_c;
			tri1_c = m;
		}
	}

	if (get_vector4_det(tri1_a, tri1_b, tri2_a, tri2_b) <= 0 && get_vector4_det(tri1_a, tri1_c, tri2_c, tri2_a) <= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

#endif
//int main() {
//	Triangle tri1 = { {0.0,0.0,0.0},{1.0,0.0,0.0},{0.0,1.0,0.0} };
//	Triangle tri2 = { { 0.25,0.0,1.0 },{ 0.25,0.0,-1.0 },{ 0.25,-1.0,0.0 } };
//	Triangle tri3 = { { -1.25,-1.25,1.0 },{ -1.25,-1.25,-1.0 },{ 0.25,-1.0,0.0 } };
//	Triangle tri4 = { { 0.0,0.0,0.0 },{ 1.0,0.0,0.0 },{ 0.0,0.0,1.0 } };
//	Triangle tri5 = { { 0.5,0.0,0.75 },{ 1.5,0.0,0.75 },{ 0.75,0.0,1.5 } };
//	if (judge_triangle_topologicalStructure(&tri1, &tri2))
//		std::cout << "yes" << std::endl;
//	else
//		std::cout << "no" << std::endl;
//}
