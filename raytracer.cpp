#include "raytracer.h"
#include "GL/gliLight.h"
#include <algorithm>
#include <cfloat>
#include <math.h>

//some helper functions
inline double clamp(double x){ return x<0 ? 0 : x>1 ? 1 : x; }

inline int toInt(double x){ return int( clamp(x)*255 + .5); }


//constructor
RayTracer::RayTracer(list<model>& models) : m_models(models)
{
	//We need to know the camera position
	camera = gli::getCameraPos();
	ctr = 0;
	//Things for gluUnProject
	modelM = new GLdouble[16];	//store the modelview matrix in this
	proj = new GLdouble[16];		//store the projection matrix in this
	view = new GLint[4];				//store the viewport matrix in this
	nearx = new GLdouble;			//store the values of the world coordinates in these
	neary = new GLdouble;
	nearz = new GLdouble;
	farx = new GLdouble;
	fary = new GLdouble;
	farz = new GLdouble;

	glGetDoublev(GL_MODELVIEW_MATRIX, modelM);	//get the modelview, projection, and viewport
	glGetDoublev(GL_PROJECTION_MATRIX, proj);
	glGetIntegerv(GL_VIEWPORT, view);
}

inline void show_progress_bar(double ratio)
{
	// Show the load bar.
	static const int w = 50;
	int   c = ratio * w;

	cout << setw(3) << (int)(ratio * 100) << "% [";
	for (int x = 0; x<c; x++) cout << "=";
	for (int x = c; x<w; x++) cout << " ";
	cout << "]\r" << flush;
}

//render an image with "nray_per_pixel" rays created per pixel
void RayTracer::render(unsigned int img_w, unsigned int img_h, unsigned int nray_per_pixel)
{
	//create a black image of size img_w X img_h
	m_img = vector< vector< Vector3d > >(img_h, vector< Vector3d >(img_w, Vector3d(0,0,0) ));

	//generate rays
	for (int y = 0; y < img_h; y++)
	{

		for (int x = 0; x < img_w; x++)
		{
			Vector3d color;
			for (int n = 0; n < nray_per_pixel; n++)
			{
				Ray r=create_a_random_ray(x,y);
				Point3d interPos;
				pair<model *, triangle *> X = intersect(r, interPos);

				//determine the color of this ray 
				if (X.first != NULL && X.second != NULL)
				{
					Vector3d rc = raycolor(*X.first, X.second, r, interPos);
					color = rc + color;
				}
			}

			m_img[y][x] = color / nray_per_pixel;
		}//end of x

		show_progress_bar(y*1.0 / img_h);

	}//end of y

	cout << ctr << endl;
}

// render an image with "nray_per_pixel" rays created per pixel
// create this ray randomly in the given pixel (x,y)
Ray RayTracer::create_a_random_ray(unsigned int x, unsigned int y)
{
	Ray r;

	//TODO: implement this
	//hint: see slides on generating rays for perspective views
	srand(time(NULL));

	//offset X, Y by tiny amount
	double xf = (double)((rand() % 10) / 10) + (double)x;
	double yf = (double)((rand() % 10) / 10) + (double)(view[3] - y);

	gluUnProject(xf, yf, 0, modelM, proj, view, nearx, neary, nearz);  //set pointPos
	gluUnProject(xf, yf, 1, modelM, proj, view, farx, fary, farz);

	//Just to make sure we're using correct data structures...
	Vector3d nearish = Vector3d(*nearx, *neary, *nearz);
	Vector3d farish = Vector3d(*farx, *fary, *farz);
	Vector3d cameraVec = Vector3d(camera[0], camera[1], camera[2]);

	r.o = nearish + cameraVec;

	//ray direction is normalized far - near
	Vector3d rayDir = farish - nearish;
	r.v = rayDir.normalize();

	//show rays to debug
	all_rays.push_back(r);
	return r;
}

//returns a model and triangle that intersect ray r
//return pair<NULL,NULL> if no intersection is found
pair<model *, triangle *> RayTracer::intersect(Ray r, Point3d& x)
{
	double min_dist = FLT_MAX;
	triangle * closest_T = NULL;
	model * closest_M = NULL;
	Point3d xx;

	for (list<model>::iterator i = m_models.begin(); i != m_models.end(); i++)
	{
		triangle * t = closest_intersect(*i, r, xx);
		if (t != NULL)
		{
			//Point3d x;
			intersect(*i, t, r, xx);
			double dist = (xx - r.o).normsqr();
			if (dist < min_dist)
			{
				min_dist = dist;
				closest_T = t;
				closest_M = &*i;
				x = xx;
			}
		}
	}
	return make_pair(closest_M, closest_T);
}

//returns a triangle in model m that intersect ray r
//return NULL if no intersection is found
triangle * RayTracer::intersect(model& m, Ray r)
{
	for (int i = 0; i < m.t_size; i++)
	{
		Point3d x;
		if ( intersect(m, &m.tris[i], r, x) )
			return &m.tris[i];
	}

	return NULL;
}

//returns a triangle in model m that make closest intersection with ray r
//return NULL if no intersection is found
triangle * RayTracer::closest_intersect(model& m, Ray r, Point3d& x)
{
	double min_dist = FLT_MAX;
	triangle * closest_T=NULL;
	for (int i = 0; i < m.t_size; i++)
	{
		Point3d xx;
		if (intersect(m, &m.tris[i], r, xx))
		{
			double dist = (xx - r.o).normsqr();
			if (dist < min_dist)
			{
				closest_T = &m.tris[i];
				min_dist = dist;
				x = xx;
			}
		}//end if
	}//end for i

	return closest_T;
}

// determine if there is an intersection between triangle t and ray r
// return true if there is an intersection and store the intersection in x
//http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/

bool RayTracer::rayIntersectsTriangle(Vector3d pos, Vector3d dir,
	Vector3d v0, Vector3d v1, Vector3d v2, Vector3d& point) {

	Vector3d edge1, edge2, h, s, q;
	float a, f, u, v, t;
	edge1 = v1 - v0;
	edge2 = v2 - v0;

	h = dir % edge2;
	a = edge1 * h;

	if (a > -0.00001 && a < 0.00001) return false;  //checking for perpendicularism ?

	f = 1 / a;
	s = pos - v0;
	u = f * (s * h);

	if (u < 0.0 || u > 1.0) return false;

	q = s % edge1;
	v = f * (dir * q);

	if (v < 0.0 || u + v > 1.0) return(false);

	// at this stage we can compute t to find out where
	// the intersection point is on the line
	t = f * (edge2 * q);

	if (t > 0.00001){ // ray intersection
		point = pos + dir * t;
		return true;
	}
	else // this means that there is a line intersection
		// but not a ray intersection
		return false;

}

// determine if there is an intersection between triangle t and ray r
// return true if there is an intersection and store the intersection in x
// return false otherwise and x is undefined in this case
bool RayTracer::intersect(model& m, triangle * t, Ray r, Point3d& x)
{
	//Get vertices of Triangle
	Vector3d v0 = Vector3d(m.vertices[t->v[0]].p[0], m.vertices[t->v[0]].p[1], m.vertices[t->v[0]].p[2]);
	Vector3d v1 = Vector3d(m.vertices[t->v[1]].p[0], m.vertices[t->v[1]].p[1], m.vertices[t->v[1]].p[2]);
	Vector3d v2 = Vector3d(m.vertices[t->v[2]].p[0], m.vertices[t->v[2]].p[1], m.vertices[t->v[2]].p[2]);

	//Ray stuffs
	Vector3d rayPos = Vector3d(r.o[0], r.o[1], r.o[2]);
	Vector3d rayDir = Vector3d(r.v[0], r.v[1], r.v[2]);
	
	Vector3d point; // = new Vector3d();
	bool hits = rayIntersectsTriangle(rayPos, rayDir, v0, v1, v2, point);
	
	x[0] = point[0];
	x[1] = point[1];
	x[2] = point[2];
	return hits;
}

//
// determine the color of ray r, by analizing the intersection between t and r 
// 
Vector3d RayTracer::raycolor(model& m, triangle * t, const Ray& r, Point3d pos)
{
	Vector3d color;
	vertex v0 = m.vertices[t->v[0]];
	vertex v1 = m.vertices[t->v[1]];
	vertex v2 = m.vertices[t->v[2]];

	//TODO: implement this
	//Interpolate the normal
	Vector3d weights = getBarycentricCoordinatesAt(Vector3d(pos.get()), Vector3d(v0.p.get()), Vector3d(v1.p.get()), Vector3d(v2.p.get()), t->n);
	Vector3d interNorm = (weights[0] * v0.n) + (weights[1] * v1.n) + (weights[2] * v2.n);

	//This is the color
	color = m.mat_color;

	//Get the light
	Vector3d lightPos = Vector3d(light0_position[0], light0_position[1], light0_position[2]);

	//Get the diffusion value
	Vector3d fragToLight = (lightPos - Vector3d(pos.get())).normalize();
	float diffuse = fragToLight * interNorm;

	//Get the specular value
	camera;
	m.mat_specular;
	m.mat_shininess;

	Vector3d reflection = -2 * (fragToLight * interNorm)*interNorm - fragToLight;
	Vector3d fragToEye = (Vector3d(camera[0], camera[1], camera[2]) - Vector3d(pos.get())).normalize();
	float specular = reflection * fragToEye;
	specular = pow(specular, 100);

	//is the frag facing away from the light?
	if (diffuse <= 0){
		diffuse = 0;
		return color * diffuse;
	}
	//diffuse not less than zero
	else if (inshadow(pos, lightPos)){
		return color / 3;
	}

	/*if (specular > 0.05){
		diffuse += specular;  //if we have specular, add it to diffuse
	}*/
	return color * diffuse;
}

//check if a point p is in shadow
bool RayTracer::inshadow(const Point3d& p, Vector3d light)
{
	//TODO: implement this

	//ray from our position to our light
	Ray toLight;
	toLight.v = (light - Vector3d(p.get())).normalize();

	//dont want to accidentally check myself
	toLight.o = Point3d( (Vector3d(p.get()) + (toLight.v * .001)).get());  //Dear God
	

	//Okay here we go.  Check if this ray intersect anything on its way to the light
	Point3d interPos;
	pair<model *, triangle *> mT = intersect(toLight, interPos);

	//If there's an intersection, is it before or after the light?
	if (mT.first != NULL && mT.second != NULL)
	{
		Vector3d fromLight = light - Vector3d(p.get());
		Vector3d fromInter = Vector3d(interPos.get()) - Vector3d(p.get());

		if (fromLight.norm() < fromInter.norm()){  //nothing blocking
			return false;
		}
		return true;  //something blocking
	}

	return false;  //nothing blocking
}



//save rendered image to file
bool RayTracer::save2file(const string& filename)
{
	FILE *f = fopen(filename.c_str(), "w");         // Write image to PPM file.

	int h = m_img.size();
	if (h == 0) return true; //nothing to save...

	int w = m_img.front().size();
	
	fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);

	for (int i = 0; i < h; i++) 
		for (int j = 0; j < w; j++) 
			fprintf(f, "%d %d %d ", toInt(m_img[i][j][0]), toInt(m_img[i][j][1]), toInt(m_img[i][j][2]));

	fclose(f);
}

//Helper for barycentric normals
Vector3d RayTracer::getBarycentricCoordinatesAt(Vector3d point, Vector3d v0, Vector3d v1, Vector3d v2, Vector3d normal)
{
	Vector3d bary;

	// The area of a triangle is 
	float areaABC = normal * ((v1 - v0) % (v2 - v0));
	float areaPBC = normal * ((v1 - point) % (v2 - point));
	float areaPCA = normal * ((v2 - point) % (v0 - point));

	bary[0] = areaPBC / areaABC; // alpha
	bary[1] = areaPCA / areaABC; // beta
	bary[2] = 1.0f - bary[0] - bary[1]; // gamma
	/*Vector3d a = v1 - v0, b = v2 - v0, c = point - v0;
	float d00 = a * a;
	float d01 = a * b;
	float d11 = b * b;
	float d20 = c * a;
	float d21 = c * b;
	float denom = d00 * d11 - d01 * d01;
	bary[1] = (d11 * d20 - d01 * d21) / denom;
	bary[2] = (d00 * d21 - d01 * d20) / denom;
	bary[0] = 1.0f - bary[2] - bary[1];*/

	return bary;
}
/*
void Barycentric(Point p, Point a, Point b, Point c, float &u, float &v, float &w)
{
	Vector v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = Dot(v0, v0);
	float d01 = Dot(v0, v1);
	float d11 = Dot(v1, v1);
	float d20 = Dot(v2, v0);
	float d21 = Dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}*/
