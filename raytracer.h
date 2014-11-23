#pragma once

#include "GL/gli.h"
#include "model.h"
#include <list>

struct Ray
{
	Point3d  o; //origin
	Vector3d v; //direction 
};

class RayTracer
{
public:

	//constructor
	RayTracer(list<model>& models);

	//render an image with "nray_per_pixel" rays created per pixel
	void render(unsigned int img_w, unsigned int img_h, unsigned int nray_per_pixel);

	//save rendered image to file
	bool save2file(const string& filename);

	bool rayIntersectsTriangle(Vector3d p, Vector3d d, Vector3d v0, Vector3d v1, Vector3d v2,
		Vector3d& point);

	Vector3d getBarycentricCoordinatesAt(Vector3d point, Vector3d v0, Vector3d v1, Vector3d v2, Vector3d normal);

private:

	// render an image with "nray_per_pixel" rays created per pixel
	// create this ray randomly in the given pixel (x,y)
	Ray create_a_random_ray(unsigned int x, unsigned int y);

	//returns a model and triangle that intersect ray r
	//return pair<NULL,NULL> if no intersection is found
	pair<model * ,triangle *> intersect(Ray r, Point3d& x);

	//returns a triangle in model m that intersect ray r
	//return NULL if no intersection is found
	triangle * intersect(model& m, Ray r);

	//returns a triangle in model m that make closest intersection with ray r
	//return NULL if no intersection is found
	triangle * closest_intersect(model& m, Ray r, Point3d& x);

	// determine if there is an intersection between triangle t and ray r
	// return true if there is an intersection and store the intersection in x
	// return false otherwise and x is undefined in this case
	bool intersect(model& m, triangle * t, Ray r, Point3d& x);

	//check if a point p is in shadow
	bool inshadow(const Point3d& p, Vector3d light);

	//
	// determine the color of ray r, by analizing the intersection between t and r 
	// 
	Vector3d raycolor(model& m, triangle * t, const Ray& r, Point3d pos);

	list<model>& m_models;

	vector< vector< Vector3d > > m_img; //rendered image

	//Things for gluUnProject
	//We need to know the camera position
	float const *camera;

	GLdouble *modelM;	//store the modelview matrix in this
	GLdouble* proj;		//store the projection matrix in this
	GLint* view;				//store the viewport matrix in this
	GLdouble* nearx;			//store the values of the world coordinates in these
	GLdouble* neary;
	GLdouble* nearz;
	GLdouble* farx;
	GLdouble* fary;
	GLdouble* farz;

	int ctr;	//for testing

public:

	//for debugging only
	list<Ray> all_rays;
	list<Point3d> all_intersection_points;
};

