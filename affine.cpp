#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <algorithm>

using namespace std;

#define Max_t 1e8
#define V struct vector3f
#define MAX_depth 4

// either store 3d coordinates or rgb colour value
struct vector3f
{
	float x, y, z;
	vector3f() { };
	vector3f(float gx, float gy, float gz) : x(gx), y(gy), z(gz) { };
};

// transformation matrix in 3d homogeneous coordinates format (4X4 matrix)
struct xmatrix
{
	V a, b, c, d;
	xmatrix() {};
	xmatrix(V x, V y, V z, V w) : a(x), b(y), c(z), d(w) {};
};

// 3d sphere
struct sphere
{
	V center;
	int istransformed;
	float radius, reflectivity, transparency, refractive_index;
	V color;
	struct xmatrix mat;
	sphere() { };
	sphere(V gcenter, float gradius, V gcolor) : center(gcenter), radius(gradius), color(gcolor) { };
};

// 3d plane
struct plane
{
	V normal, color;
	float d, reflectivity, transparency, refractive_index;
	plane() { };
	plane(V gnormal, V icolor,float gd) : normal(gnormal), color(icolor), d(gd) { };
};

// 3d triangle
struct triangle
{
	V u, v, w;
	V color;
	float reflectivity, transparency, refractive_index;
	triangle() { };
	triangle(V gu, V gv, V gw, V gcolor) : u(gu), v(gv), w(gw), color(gcolor) { };
};

// light source
struct light
{
	V point;
	V color;
	light() { };
	light(V gpoint, V gcolor) : point(gpoint), color(gcolor) { };
};

struct coordinatesystem
{
	V camera, vrp, vpn, up;
};

enum object
{
	none,
	sphereob,
	triangleob,
	planeob
};

vector <sphere> spheres;
vector <plane> planes;
vector <triangle> triangles;
vector <light> lights;
struct coordinatesystem vcs;

int numspheres = 0;
int numplanes = 0;
int numtriangles = 0;
int numlights = 0;

int width = 640;
int height = 480;

// normalize a 3d vector
void normalize(V & v)
{
	float norm = sqrt((v.x)*(v.x)+(v.y)*(v.y)+(v.z)*(v.z));
	if(norm > 0.0)
	{
		v.x = v.x/norm;
		v.y = v.y/norm;
		v.z = v.z/norm;
	}
}

// dot product of two 3d vectors
float dotproduct(V v1, V v2) {
	float ret = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
	return ret;
}

float determinant(V a, V b, V c) {		// @params are rows
	float x = a.x*(b.y*c.z - b.z*c.y);
	float y = -1*a.y*(b.x*c.z - b.z*c.x);
	float z = a.z*(b.x*c.y - b.y*c.x);
	return (x+y+z);
}

xmatrix inverseMatrix(struct xmatrix m) {
	V a = m.a, b = m.b, c = m.c, d = m.d;
	xmatrix mm = m;
	float det = determinant(m.a, m.b, m.c);
	mm.a.x = ((b.y*c.z - b.z*c.y))/det;			//a.x
	mm.a.y = (-1*(a.y*c.z - c.y*a.z))/det;		//b.x
	mm.a.z = ((a.y*b.z - b.y*a.z))/det;			//c.x
	mm.b.x = (-1*(b.x*c.z - c.x*b.z))/det;		//a.y
	mm.b.y = ((a.x*c.z - a.z*c.x))/det;			//b.y
	mm.b.z = (-1*(a.x*b.z - a.z*b.x))/det;		//c.y
	mm.c.x = ((b.x*c.y - c.x*b.y))/det;			//a.z
	mm.c.y = (-1*(a.x*c.y - a.y*c.x))/det;		//b.z
	mm.c.z = ((a.x*b.y - a.y*b.z))/det;			//c.z
	mm.d.x = -1*determinant(b, c, d)/det;
	mm.d.y = determinant(a, c, d)/det;
	mm.d.z = -1*determinant(a, b, d)/det;
	return mm;
}

// apply affine transformation matrix on a 3d vector
// random
V transform(V v, struct xmatrix m)
{
	v.x = v.x * m.a.x + v.y * m.b.x + v.z * m.c.x + m.d.x;
	v.y = v.x * m.a.y + v.y * m.b.y + v.z * m.c.y + m.d.y;
	v.z = v.x * m.a.z + v.y * m.b.z + v.z * m.c.z + m.d.z;
	return v;
}

// apply inverse affine transformation matrix on a 3d vector
// if transpose is one transpose of inverse is the applied transformation
// else only inverse is the applied transformation
//random
V invtransform(V v, struct xmatrix m, int transpose) {
	xmatrix im = inverseMatrix(m);
	V ret(0, 0, 0);
	if(transpose == 0) {
		ret.x = v.x * im.a.x + v.y * im.b.x + v.z * im.c.x + im.d.x;
		ret.y = v.x * im.a.y + v.y * im.b.y + v.z * im.c.y + im.d.y;
		ret.z = v.x * im.a.z + v.y * im.b.z + v.z * im.c.z + im.d.z;
	} else {
		float val = v.x * im.d.x + v.y * im.d.y + v.z * im.d.z + 1;
		ret.x = (v.x * im.a.x + v.y * im.a.y + v.z * im.a.z)/val;
		ret.y = (v.x * im.b.x + v.y * im.b.y + v.z * im.b.z)/val;
		ret.z = (v.x * im.c.x + v.y * im.c.y + v.z * im.c.z)/val;
	}
	return ret;
}
// V invtransform(V v, struct xmatrix m, int transpose)
// {
// 	float b11 = m.def.y*m.ghi.z-m.ghi.y*m.def.z;
// 	float b12 = m.abc.z*m.ghi.y-m.abc.y*m.ghi.z;
// 	float b13 = m.abc.y*m.def.z-m.abc.z*m.def.y;
// 	float b14 = 0.0, b24 = 0.0, b34 = 0.0;
// 	float b21 = m.def.z*m.ghi.x-m.def.x*m.ghi.z;
// 	float b22 = m.abc.x*m.ghi.z-m.abc.z*m.ghi.x;
// 	float b23 = m.abc.z*m.def.x-m.abc.x*m.def.z;
// 	float b31 = m.def.x*m.ghi.y-m.def.y*m.ghi.x;
// 	float b32 = m.abc.y*m.ghi.x-m.abc.x*m.ghi.y;
// 	float b33 = m.abc.x*m.def.y-m.abc.y*m.def.x;
// 	float b41 = m.def.x*m.ghi.z*m.lmn.y+m.def.x*m.ghi.x*m.lmn.z+m.def.z*m.ghi.y*m.lmn.x
// 				-m.def.x*m.ghi.y*m.lmn.z-m.def.x*m.ghi.z*m.lmn.x-m.def.z*m.ghi.x*m.lmn.y;
// 	float b42 = m.abc.x*m.ghi.y*m.lmn.z+m.abc.y*m.ghi.z*m.lmn.x+m.abc.z*m.ghi.x*m.lmn.y
// 				-m.abc.x*m.ghi.z*m.lmn.y-m.abc.y*m.ghi.x*m.lmn.z-m.abc.z*m.ghi.y*m.lmn.x;
// 	float b43 = m.abc.x*m.def.z*m.lmn.y+m.abc.y*m.def.x*m.lmn.z+m.abc.z*m.def.y*m.ghi.x
// 				-m.abc.x*m.def.y*m.lmn.z-m.abc.y*m.def.z*m.lmn.x-m.abc.z*m.def.x*m.lmn.y;
// 	float b44 = m.abc.x*m.def.y*m.ghi.z+m.abc.y*m.def.z*m.ghi.x+m.abc.z*m.def.x*m.ghi.y
// 				-m.abc.x*m.def.z*m.ghi.y-m.abc.y*m.def.x*m.ghi.z-m.abc.z*m.def.y*m.ghi.x;
// 	float det = m.abc.x*m.def.y*m.ghi.z+m.abc.y*m.def.z*m.ghi.x+m.abc.z*m.def.x*m.ghi.y
// 				-m.abc.x*m.def.z*m.ghi.y-m.abc.y*m.def.x*m.ghi.z-m.abc.z*m.ghi.x*m.def.y;
// 	if(det <= 1e-4)
// 	{
// 		return v;
// 	}
// 	if(transpose)
// 	{
// 		float homow = (v.x*b14+v.y*b24+v.z*b34+b44);
// 		v.x = (v.x*b11+v.y*b21+v.z*b31+b41)/(det*homow);
// 		v.y = (v.x*b12+v.y*b22+v.z*b32+b42)/(det*homow);
// 		v.z = (v.x*b13+v.y*b23+v.z*b33+b43)/(det*homow);
// 	}
// 	else
// 	{
// 		float homow = (v.x*b41+v.y*b42+v.z*b43+b44);
// 		v.x = (v.x*b11+v.y*b12+v.z*b13+b14)/(det*homow);
// 		v.y = (v.x*b21+v.y*b22+v.z*b23+b24)/(det*homow);
// 		v.z = (v.x*b31+v.y*b32+v.z*b33+b34)/(det*homow);
// 	}
// 	return v;
// }

// check intersection of a ray and a sphere
bool intersectsphere(V rayorigin, V raydir, struct sphere s, V & normal, float & t0, float & t1)
{
	float a = 1.0;
	float b = 2*(raydir.x*(rayorigin.x-s.center.x)+
				 raydir.y*(rayorigin.y-s.center.y)+
				 raydir.z*(rayorigin.z-s.center.z));
	float c = (rayorigin.x-s.center.x)*(rayorigin.x-s.center.x)+
			  (rayorigin.y-s.center.y)*(rayorigin.y-s.center.y)+
			  (rayorigin.z-s.center.z)*(rayorigin.z-s.center.z)-
			  (s.radius)*(s.radius);
	float b2m4ac = b*b-4*a*c;
	if(b2m4ac < 0)
	{
		return false;
	}
	t0 = (-b-sqrt(b2m4ac))/(2*a);
	t1 = (-b+sqrt(b2m4ac))/(2*a);
	if(t0 < 0 && t1 < 0)
	{
		return false;
	}
	if(t0 < 0)
	{
		t0 = t1;
	}
	float t = t0;
	if(t1 > 0)
	{
		t0 = min(t0, t1);
		t = t0;
	}
	normal.x = (rayorigin.x+raydir.x*t-s.center.x)/s.radius;
	normal.y = (rayorigin.y+raydir.y*t-s.center.y)/s.radius;
	normal.z = (rayorigin.z+raydir.z*t-s.center.z)/s.radius;
	return true;
}


// second approach(vector based) for sphere ray intersection
bool intersectspherealt(V rayorigin, V raydir, struct sphere s, V & normal, float & t0, float & t1)
{
	V L(s.center.x-rayorigin.x, s.center.y-rayorigin.y, s.center.z-rayorigin.z);
	float tca = dotproduct(L, raydir);
	if(tca < 0.0)
	{
		return false;
	}
	float d = dotproduct(L, L) - tca*tca;
	if(d > s.radius)
	{
		return false;
	}
	float thc = sqrt(s.radius*s.radius - d*d);
	t0 = tca - thc;
	t1 = tca - thc;
	if(t0 < 0 && t1 < 0)
	{
		return false;
	}
	if(t0 < 0)
	{
		t0 = t1;
	}
	float t = t0;
	if(t1 > 0)
	{
		t0 = min(t0, t1);
		t = t0;
	}
	normal.x = (rayorigin.x+raydir.x*t-s.center.x)/s.radius;
	normal.y = (rayorigin.y+raydir.y*t-s.center.y)/s.radius;
	normal.z = (rayorigin.z+raydir.z*t-s.center.z)/s.radius;
	return true;
}

// check intersection of a ray and a plane
bool intersectplane(V rayorigin, V raydir, struct plane p, float & t)
{
	normalize(p.normal);
	// float vd = p.normal.x*raydir.x+p.normal.y*raydir.y+p.normal.z*raydir.z;
	float vd = dotproduct(p.normal, raydir);
	if(vd == 0)
	{
		return false;
	}
	// t = -(p.normal.x*rayorigin.x+p.normal.y*rayorigin.y+p.normal.z*rayorigin.z+p.d)/(vd);
	t = -(dotproduct(p.normal, rayorigin)+p.d)/(vd);
	if(t < 0)
	{
		return false;
	}
	return true;
}

// check intersection of a ray and a triangle
bool intersecttriangle(V rayorigin, V raydir, struct triangle s, V & normal, float & t)
{
	V v1(s.u.x-s.v.x, s.u.y-s.v.y, s.u.z-s.v.z);
	V v2(s.w.x-s.v.x, s.w.y-s.v.y, s.w.z-s.v.z);
	V cross(v1.y*v2.z-v2.y*v1.z, v2.x*v1.z-v1.x*v2.z, v1.x*v2.y-v2.x*v1.y);
	// float area = 0.5*sqrt(cross.x*cross.x+cross.y*cross.y+cross.z*cross.z);
	float area = 0.5*sqrt(dotproduct(cross, cross));
	normalize(cross);
	// float d = -(cross.x*s.u.x+cross.y*s.u.y+cross.z*s.u.z);
	float d = -(dotproduct(cross, s.u));
	// float vd = cross.x*raydir.x+cross.y*raydir.y+cross.z*raydir.z;
	float vd = dotproduct(cross, raydir);
	if(vd == 0)
	{
		return false;
	}
	if(vd > 0)
	{
		normal.x = -cross.x; normal.y = -cross.y; normal.z = -cross.z;
	}
	else
	{
		normal = cross;
	}
	// t = -(cross.x*rayorigin.x+cross.y*rayorigin.y+cross.z*rayorigin.z+d)/(vd);
	t = -(dotproduct(cross, rayorigin)+d)/(vd);
	if(t < 0)
	{
		return false;
	}
	V point(rayorigin.x+raydir.x*t, rayorigin.y+raydir.y*t, rayorigin.z+raydir.z*t);
	V vec1(point.x-s.v.x, point.y-s.v.y, point.z-s.v.z);
	V vec2(point.x-s.w.x, point.y-s.w.y, point.z-s.w.z);
	V vec3(s.u.x-s.w.x, s.u.y-s.w.y, s.u.z-s.w.z);
	V cross1(vec1.y*v1.z-v1.y*vec1.z, v1.x*vec1.z-v1.z*vec1.x, v1.y*vec1.x-v1.x*vec1.y);
	V cross2(vec1.z*v2.y-vec1.y*v2.z, vec1.x*v2.z-vec1.z*v2.x, vec1.y*v2.x-vec1.x*v2.y);
	V cross3(vec2.z*vec3.y-vec2.y*vec3.z, vec2.x*vec3.z-vec2.z*vec3.x, vec2.y*vec3.x-vec2.x*vec3.y);
	// float areau = 0.5*sqrt(cross2.x*cross2.x+cross2.y*cross2.y+cross2.z*cross2.z);
	float areau = 0.5*sqrt(dotproduct(cross2, cross2));
	// float areav = 0.5*sqrt(cross3.x*cross3.x+cross3.y*cross3.y+cross3.z*cross3.z);
	float areav = 0.5*sqrt(dotproduct(cross3, cross3));
	// float areaw = 0.5*sqrt(cross1.x*cross1.x+cross1.y*cross1.y+cross1.z*cross1.z);
	float areaw = 0.5*sqrt(dotproduct(cross1, cross1));
	if(abs(areau+areav+areaw-area) > 0.01)
	{
		return false;
	}
	return true;
}

// trace rays and get pixel colour
vector3f raytrace(V rayorigin, V raydir, int depth)
{
	float nearest = Max_t;
	struct sphere * s = NULL;
	struct plane * p = NULL;
	struct triangle * tr = NULL;
	V pix, normal;
	bool objectfound = false, transformed = false;
	object ob = none;
	pix.x = pix.y = pix.z = 0.4;
	// find intersection with all spheres
	for(int spherei = 0; spherei < numspheres; spherei++)
	{
		float t0 = Max_t, t1 = Max_t;
		V normaltemp, trayorigin = rayorigin, traydir = raydir;
		// random
		if(spheres[spherei].istransformed)
		{
			trayorigin = invtransform(rayorigin, spheres[spherei].mat, 0);
			traydir = invtransform(raydir, spheres[spherei].mat, 0);
		}
		if(intersectsphere(trayorigin, traydir, spheres[spherei], normaltemp, t0, t1))
		{
			if(t0 < nearest)
			{
				nearest = t0;
				s = &(spheres[spherei]);
				normal = normaltemp;
				// random
				if((*s).istransformed)
				{
					normal = invtransform(normal, (*s).mat, 1);
					transformed = true;
				}
				ob = sphereob;
				objectfound = true;
			}
		}
	}
	// find intersection with all triangles
	for(int trianglei = 0; trianglei < numtriangles; trianglei++)
	{
		float t = Max_t;
		V normaltemp;
		if(intersecttriangle(rayorigin, raydir, triangles[trianglei], normaltemp, t))
		{
			if(t < nearest)
			{
				nearest = t;
				tr = &(triangles[trianglei]);
				normal = normaltemp;
				ob = triangleob;
				objectfound = true;
				transformed = false;
			}
		}
	}
	
	if(!objectfound)
	{
		return pix;
	}
	V objcolor;
	V intpoint(rayorigin.x+nearest*raydir.x, rayorigin.y+nearest*raydir.y, rayorigin.z+nearest*raydir.z);
	normalize(normal);
	pix.x = pix.y = pix.z = 0.0;
	// no recursive tracing if object is diffuse just get surface colour
	// cout << (*s).color.x << " " << (*s).color.y << " " << (*s).color.z << endl;
	float bias = 1e-4;
	bool diffuse = true;
	if(depth < MAX_depth)
	{
		if(ob == sphereob)
		{
			// random
			if(transformed == true)
			{
				intpoint = transform(intpoint, (*s).mat);
			}
			bool insphere = false;
			if(dotproduct(normal, raydir) > 0) 
			{
				normal.x *= -1; normal.y *= -1; normal.z *= -1;
				insphere = true;
			}
			if((*s).reflectivity > 0)
			{
				// float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fratio = -1*(dotproduct(raydir, normal));
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				V reflectdir(raydir.x+2*fratio*normal.x, raydir.y+2*fratio*normal.y, raydir.z+2*fratio*normal.z);
				normalize(reflectdir);
				V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
				V reflectpoint = raytrace(biasedintpoint, reflectdir, depth+1);
				pix.x += reflectpoint.x*fresnel*(*s).color.x;
				pix.y += reflectpoint.y*fresnel*(*s).color.y;
				pix.z += reflectpoint.z*fresnel*(*s).color.z;
				diffuse = false;
			}
			if((*s).transparency > 0)
			{
				float refractindex = (*s).refractive_index;
				if(insphere) 
				{
					refractindex = 1.0/refractindex;
				}
				float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				float raydotn = -1*(normal.x*raydir.x+normal.y*raydir.y+normal.z*raydir.z);
				float k = 1.0-refractindex*refractindex*(1.0-raydotn*raydotn);
				V refractdir(raydir.x*refractindex+normal.x*(refractindex*raydotn-sqrt(k)),
							 raydir.y*refractindex+normal.y*(refractindex*raydotn-sqrt(k)),
							 raydir.z*refractindex+normal.z*(refractindex*raydotn-sqrt(k)));
				normalize(refractdir);
				V biasedintpoint(intpoint.x-normal.x*bias, intpoint.y-normal.y*bias, intpoint.z-normal.z*bias);
				V refractpoint = raytrace(biasedintpoint, refractdir, depth+1);
				pix.x += refractpoint.x*(1.0-fresnel)*(*s).transparency*(*s).color.x;
				pix.y += refractpoint.y*(1.0-fresnel)*(*s).transparency*(*s).color.y;
				pix.z += refractpoint.z*(1.0-fresnel)*(*s).transparency*(*s).color.z;
				diffuse = false;
			}
		}
		else if(ob == triangleob)
		{
			if((*tr).reflectivity > 0)
			{
				// float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fratio = -1*(dotproduct(raydir, normal));
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				V reflectdir(raydir.x+2*fratio*normal.x, raydir.y+2*fratio*normal.y, raydir.z+2*fratio*normal.z);
				normalize(reflectdir);
				V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
				V reflectpoint = raytrace(biasedintpoint, reflectdir, depth+1);
				pix.x += reflectpoint.x*fresnel*(*tr).color.x;
				pix.y += reflectpoint.y*fresnel*(*tr).color.y;
				pix.z += reflectpoint.z*fresnel*(*tr).color.z;
				diffuse = false;
			}
			if((*tr).transparency > 0)
			{
				float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				float refractindex = (*tr).refractive_index;
				float raydotn = -1*(normal.x*raydir.x+normal.y*raydir.y+normal.z*raydir.z);
				float k = 1.0-refractindex*refractindex*(1.0-raydotn*raydotn);
				V refractdir(raydir.x*refractindex+normal.x*(refractindex*raydotn-sqrt(k)),
							 raydir.y*refractindex+normal.y*(refractindex*raydotn-sqrt(k)),
							 raydir.z*refractindex+normal.z*(refractindex*raydotn-sqrt(k)));
				normalize(refractdir);
				V biasedintpoint(intpoint.x-normal.x*bias, intpoint.y-normal.y*bias, intpoint.z-normal.z*bias);
				V refractpoint = raytrace(biasedintpoint, refractdir, depth+1);
				pix.x += refractpoint.x*(1.0-fresnel)*(*tr).transparency*(*tr).color.x;
				pix.y += refractpoint.y*(1.0-fresnel)*(*tr).transparency*(*tr).color.y;
				pix.z += refractpoint.z*(1.0-fresnel)*(*tr).transparency*(*tr).color.z;
				diffuse = false;
			}
		}
	}
	// if(depth >= MAX_depth || diffuse)
	// {
		for(int lighti = 0; lighti < numlights; lighti++)
		{
			V lightdir(lights[lighti].point.x-intpoint.x, lights[lighti].point.y-intpoint.y, lights[lighti].point.z-intpoint.z); 
			normalize(lightdir);
			// float dotprod = max(float(0.0), normal.x*lightdir.x+normal.y*lightdir.y+normal.z*lightdir.z);
			float dotprod = max(float(0.0), dotproduct(normal, lightdir));
			// check if point is in shadow
			bool inshadow = false;
			V normaltemp;
			float t0, t1, t;
			V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
			for(int spherei = 0; spherei < numspheres; spherei++)
			{
				// random
				V tlightdir = lightdir;
				if(spheres[spherei].istransformed)
				{
					tlightdir = invtransform(lightdir, spheres[spherei].mat, 0);
				}
				if(intersectsphere(biasedintpoint, tlightdir, spheres[spherei], normaltemp, t0, t1))
				{
					inshadow = true;
					break;
				}
			}
			if(inshadow == true)
			{
				continue;
			}
			for(int trianglei = 0; trianglei < numtriangles; trianglei++)
			{
				if(intersecttriangle(biasedintpoint, lightdir, triangles[trianglei], normaltemp, t))
				{
					inshadow = true;
					break;
				}
			}
			if(!inshadow)
			{
				if(ob == sphereob)
				{
					// float ndoti2 = 2*(lightdir.x*normal.x+lightdir.y*normal.y+lightdir.z*normal.z);
					float ndoti2 = 2*(dotproduct(lightdir, normal));
					V reflectedray(ndoti2*normal.x-lightdir.x, ndoti2*normal.y-lightdir.y, ndoti2*normal.z-lightdir.z);
					normalize(reflectedray);
					// float rdotv = -1*(raydir.x*reflectedray.x+raydir.y*reflectedray.y+raydir.z*reflectedray.z);
					float rdotv = -1*(dotproduct(raydir, reflectedray));
					pix.x += (*s).color.x*(*s).reflectivity*pow(rdotv, 10)*lights[lighti].color.x;
					pix.y += (*s).color.y*(*s).reflectivity*pow(rdotv, 10)*lights[lighti].color.y;
					pix.z += (*s).color.z*(*s).reflectivity*pow(rdotv, 10)*lights[lighti].color.z;
					pix.x += dotprod*(*s).color.x*lights[lighti].color.x;
					pix.y += dotprod*(*s).color.y*lights[lighti].color.y;
					pix.z += dotprod*(*s).color.z*lights[lighti].color.z;
				}
				else if(ob == triangleob)
				{
					// float ndoti2 = 2*(lightdir.x*normal.x+lightdir.y*normal.y+lightdir.z*normal.z);
					float ndoti2 = 2*(dotproduct(lightdir, normal));
					V reflectedray(ndoti2*normal.x-lightdir.x, ndoti2*normal.y-lightdir.y, ndoti2*normal.z-lightdir.z);
					normalize(reflectedray);
					// float rdotv = -1*(raydir.x*reflectedray.x+raydir.y*reflectedray.y+raydir.z*reflectedray.z);
					float rdotv = -1*(dotproduct(raydir, reflectedray));
					pix.x += (*tr).color.x*(*tr).reflectivity*pow(rdotv, 10)*lights[lighti].color.x;
					pix.y += (*tr).color.y*(*tr).reflectivity*pow(rdotv, 10)*lights[lighti].color.y;
					pix.z += (*tr).color.z*(*tr).reflectivity*pow(rdotv, 10)*lights[lighti].color.z;
					pix.x += dotprod*(*tr).color.x*lights[lighti].color.x;
					pix.y += dotprod*(*tr).color.y*lights[lighti].color.y;
					pix.z += dotprod*(*tr).color.z*lights[lighti].color.z;
				}
			}
		}
	// }
	return pix;
}

// evaluates pixel values of resulting image and stores in a file
void render2d()
{
	V ** image = new V * [height];
	for(int row = 0; row < height; row++)
	{
		image[row] = new vector3f[width];
	}
	V * pixeli;
	V ncrossv;
	ncrossv.x = -1*(vcs.vpn.y*vcs.up.z - vcs.vpn.z*vcs.up.y);
	ncrossv.y = -1*(vcs.up.x*vcs.vpn.z - vcs.vpn.x*vcs.up.z);
	ncrossv.z = -1*(vcs.vpn.x*vcs.up.y - vcs.vpn.y*vcs.up.x);
	normalize(ncrossv);
	// float fieldofview = 90;
	float whratio = (float)width/(float)height;
	float dwidth = 1.0/(float)width;
	float dheight = 1.0/(float)height;
	// float angleofview = tan(M_PI*0.5*fieldofview/180);
	float angleofview = 1.0/sqrt(dotproduct(vcs.camera, vcs.camera));
	// V orig(0.0, 0.0, 0.0);
	V orig, raydir;
	orig.x = vcs.camera.x*ncrossv.x+vcs.camera.y*vcs.up.x+vcs.camera.z*vcs.vpn.x+vcs.vrp.x;
	orig.y = vcs.camera.x*ncrossv.y+vcs.camera.y*vcs.up.y+vcs.camera.z*vcs.vpn.y+vcs.vrp.y;
	orig.z = vcs.camera.x*ncrossv.z+vcs.camera.y*vcs.up.z+vcs.camera.z*vcs.vpn.z+vcs.vrp.z;
	for(int yval = 0; yval < height; yval++)
	{
		pixeli = image[yval];
		for(int xval = 0; xval < width; xval++)
		{
			float imagex = (2*((xval+0.5)*dwidth)-1)*angleofview*whratio;
			float imagey = (1-2*((yval+0.5)*dheight))*angleofview;
			// V raydir(imagex, imagey, -1);
			raydir.x = (imagex-vcs.camera.x)*ncrossv.x+(imagey-vcs.camera.y)*vcs.up.x+(-vcs.camera.z)*vcs.vpn.x;
			raydir.y = (imagex-vcs.camera.x)*ncrossv.y+(imagey-vcs.camera.y)*vcs.up.y+(-vcs.camera.z)*vcs.vpn.y;
			raydir.z = (imagex-vcs.camera.x)*ncrossv.z+(imagey-vcs.camera.y)*vcs.up.z+(-vcs.camera.z)*vcs.vpn.z;
			normalize(raydir);
			*pixeli = raytrace(orig, raydir, 0);
			pixeli++;
		}
	}

	// anti aliasing stuff
	/*V ** imagenew = new V * [height];
	for(int row = 0; row < height; row++)
	{
		imagenew[row] = new vector3f[width];
	}
	
	for(int y=0; y<height; y++) 
	{
		pixeli = imagenew[y];
		for(int x=0; x<width; x++) 
		{
			float counter = 0.0;
			V temppixel(0.0, 0.0, 0.0);
			for(int yval = y-1; yval <= y+1; yval++) 
			{
				for(int xval = x-1; xval <= x+1; xval++) 
				{
					if(xval >= 0 && yval >= 0 && xval < width && yval < height) 
					{
						counter = counter + 1.0;
						float imagex = (2*((xval+0.5)*dwidth)-1)*angleofview*whratio;
						float imagey = (1-2*((yval+0.5)*dheight))*angleofview;
						V raydir(imagex, imagey, -1);
						normalize(raydir);
						temppixel = raytrace(orig, raydir, 0);
						if(yval == y && xval == x) 
							{
								pixeli->x += 2*temppixel.x, pixeli->y += 2*temppixel.y, pixeli->z += 2*temppixel.z;
							}
						else 
						{
							pixeli->x += temppixel.x, pixeli->y += temppixel.y, pixeli->z += temppixel.z;
						}
					}
				}
			}
			pixeli->x = pixeli->x/(counter+1);
			pixeli->y = pixeli->y/(counter+1);
			pixeli->z = pixeli->z/(counter+1);
			pixeli++;
		}
	}*/

	FILE *f = fopen("out.ppm", "wb");
	fprintf(f, "P6\n%i %i 255\n", width, height);
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			fputc(min(float(1.0), image[y][x].x)*255, f);
			fputc(min(float(1.0), image[y][x].y)*255, f);
			fputc(min(float(1.0), image[y][x].z)*255, f);
		}
	}
	fclose(f);
}

// parse input and populate global shape vectors
void parseinput(char * file)
{
	ifstream op(file);
	int numobjects;
	op >> numobjects;
	string name;
	op >> name >> vcs.camera.x >> vcs.camera.y >> vcs.camera.z;
	op >> name >> vcs.vrp.x >> vcs.vrp.y >> vcs.vrp.z;
	op >> name >> vcs.vpn.x >> vcs.vpn.y >> vcs.vpn.z;
	op >> name >> vcs.up.x >> vcs.up.y >> vcs.up.z;
	for(int objecti = 0; objecti < numobjects; objecti++)
	{
		string objecttype;
		op >> objecttype;
		if(objecttype == "sphere")
		{
			struct sphere s;
			op >> s.center.x >> s.center.y >> s.center.z;
			op >> s.radius;
			op >> s.color.x >> s.color.y >> s.color.z;
			op >> s.reflectivity >> s.transparency >> s.refractive_index >> s.istransformed;
			if(s.istransformed)
			{
				op >> s.mat.a.x >> s.mat.a.y >> s.mat.a.z;
				op >> s.mat.b.x >> s.mat.b.y >> s.mat.b.z;
				op >> s.mat.c.x >> s.mat.c.y >> s.mat.c.z;
				op >> s.mat.d.x >> s.mat.d.y >> s.mat.d.z;
			}
			spheres.push_back(s);
			numspheres++;
		}
		else if(objecttype == "plane")
		{
			struct plane p;
			op >> p.normal.x >> p.normal.y >> p.normal.z;
			op >> p.color.x >> p.color.y >> p.color.z;
			op >> p.d >> p.reflectivity >> p.transparency >> p.refractive_index;
			planes.push_back(p);
			numplanes++;
		}
		else if(objecttype == "triangle")
		{
			struct triangle t;
			op >> t.u.x >> t.u.y >> t.u.z;
			op >> t.v.x >> t.v.y >> t.v.z;
			op >> t.w.x >> t.w.y >> t.w.z;
			op >> t.color.x >> t.color.y >> t.color.z;
			op >> t.reflectivity >> t.transparency >> t.refractive_index;
			triangles.push_back(t);
			numtriangles++;
		}
		else if(objecttype == "light")
		{
			struct light l;
			op >> l.point.x >> l.point.y >> l.point.z;
			op >> l.color.x >> l.color.y >> l.color.z;
			lights.push_back(l);
			numlights++;
		}
	}
	op.close();
}

int main(int argc, char ** argv)
{
	if(argc < 2 || argc > 2)
	{
		cout << "Please follow the specifications!\nUnexpected arguments given.\n";
		cout << "Please run as: a.exe <inputfile>\n";
		exit(0);
	}

	parseinput(argv[1]);

	render2d();

	return 0;
}