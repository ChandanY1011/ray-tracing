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
float dotProduct(V v1, V v2) {
	float ret = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
	return ret;
}

// 3d sphere
struct sphere
{
	V center;
	float radius, reflectivity, transparency, refractive_index;
	V color;
	sphere() { };
	sphere(V gcenter, float gradius, V gcolor) : center(gcenter), radius(gradius), color(gcolor) { };
};

// 3d plane
struct plane
{
	V normal, color;
	float d, reflectivity, transparency, refractive_index;
	plane() { };
	plane(V gnormal, V icolor, float gd) : normal(gnormal), color(icolor), d(gd) { };
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

struct cuboid {
	V c1, c2, c3 ,c4, c5, c6, c7, c8;
	V color;
	float reflectivity, transparency, refractive_index;
	cuboid() {};
	cuboid(V c1, V c2, V col) : corner1(c1), corner2(c2), color(col) {};
};

// light source
struct light
{
	V point;
	V color;
	light() { };
	light(V gpoint, V gcolor) : point(gpoint), color(gcolor) { };
};

enum object
{
	none,
	sphereob,
	triangleob,
	planeob,
	cuboidob
};

vector <sphere> spheres;
vector <plane> planes;
vector <triangle> triangles;
vector <light> lights;
vector <cuboid> cuboids;

int numspheres = 0;
int numplanes = 0;
int numtriangles = 0;
int numlights = 0;
int numcuboids = 0;

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
	normal.x = (rayorigin.x + raydir.x*t - s.center.x) / s.radius;
	normal.y = (rayorigin.y + raydir.y*t - s.center.y) / s.radius;
	normal.z = (rayorigin.z + raydir.z*t - s.center.z) / s.radius;
	return true;
}


// second approach(vector based) for sphere ray intersection
bool intersectspherealt(V rayorigin, V raydir, struct sphere s, V & normal, float & t0, float & t1)
{
	return true;
}

// check intersection of a ray and a plane
bool intersectplane(V rayorigin, V raydir, struct plane p, float & t)
{
	normalize(p.normal);
	// float vd = p.normal.x*raydir.x+p.normal.y*raydir.y+p.normal.z*raydir.z;
	float vd = dotProduct(p.normal, raydir);
	if(vd == 0)
	{
		return false;
	}
	// t = -(p.normal.x*rayorigin.x+p.normal.y*rayorigin.y+p.normal.z*rayorigin.z+p.d)/(vd);
	t = -1*(dotProduct(p.normal, rayorigin) + p.d)/(vd);
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
	float area = 0.5*sqrt(cross.x*cross.x+cross.y*cross.y+cross.z*cross.z);
	normalize(cross);
	float d = -(cross.x*s.u.x+cross.y*s.u.y+cross.z*s.u.z);
	float vd = cross.x*raydir.x+cross.y*raydir.y+cross.z*raydir.z;
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
	t = -(cross.x*rayorigin.x+cross.y*rayorigin.y+cross.z*rayorigin.z+d)/(vd);
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
	float areau = 0.5*sqrt(cross2.x*cross2.x+cross2.y*cross2.y+cross2.z*cross2.z);
	float areav = 0.5*sqrt(cross3.x*cross3.x+cross3.y*cross3.y+cross3.z*cross3.z);
	float areaw = 0.5*sqrt(cross1.x*cross1.x+cross1.y*cross1.y+cross1.z*cross1.z);
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
	V pix, normal;		// normal = normal at intersection point
	bool objectfound = false;
	object ob = none;
	pix.x = pix.y = pix.z = 0.4;
	// find intersection with all spheres
	for(int spherei = 0; spherei < numspheres; spherei++)
	{
		float t0 = Max_t, t1 = Max_t;
		V normaltemp;
		if(intersectsphere(rayorigin, raydir, spheres[spherei], normaltemp, t0, t1))
		{
			if(t0 < nearest)
			{
				nearest = t0;
				s = &(spheres[spherei]);
				normal = normaltemp;
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
			}
		}
	}
	// find intersection with all planes
	for(int i=0; i < numplanes; i++) {
		float t = Max_t;
		V normaltemp;
		if(intersectplane(rayorigin, raydir, planes[i], t)) {
			if(t < nearest) {
				nearest = t;
				p = &(planes[i]);
				normal = (*p).normal;
				if(dotProduct(normal, raydir) > 0) {
					normal.x *= -1; normal.y *= -1; normal.z *= -1; 
				}
				ob = planeob;
				objectfound = true;
			}
		}
	}
	if(!objectfound)
	{
		return pix;
	}
	V objcolor;
	// INTERSECTION POINT
	V intpoint(rayorigin.x+nearest*raydir.x, rayorigin.y+nearest*raydir.y, rayorigin.z+nearest*raydir.z);
	normalize(normal);
	pix.x = pix.y = pix.z = 0.0;
	// no recursive tracing since object is diffuse just get surface colour
	// cout << (*s).color.x << " " << (*s).color.y << " " << (*s).color.z << endl;
	float bias = 1e-4;
	bool diffuse = true;

	// illumination
	for(int lighti = 0; lighti < numlights; lighti++)
	{
		V lightdir(lights[lighti].point.x-intpoint.x, lights[lighti].point.y-intpoint.y, lights[lighti].point.z-intpoint.z); 
		normalize(lightdir);
		float dotprod = max(float(0.0), normal.x*lightdir.x+normal.y*lightdir.y+normal.z*lightdir.z);
		// check if point is in shadow
		bool inshadow = false;
		V normaltemp;
		float t0, t1, t;
		V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
		for(int spherei = 0; spherei < numspheres; spherei++)
		{
			if(intersectsphere(biasedintpoint, lightdir, spheres[spherei], normaltemp, t0, t1))
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
				float ndoti2 = 2*(lightdir.x*normal.x+lightdir.y*normal.y+lightdir.z*normal.z);
				V reflectedray(ndoti2*normal.x-lightdir.x, ndoti2*normal.y-lightdir.y, ndoti2*normal.z-lightdir.z);
				normalize(reflectedray);
				float rdotv = -1*(raydir.x*reflectedray.x+raydir.y*reflectedray.y+raydir.z*reflectedray.z);
				pix.x += (*s).color.x*(*s).reflectivity*pow(rdotv, 10)*lights[lighti].color.x;
				pix.y += (*s).color.y*(*s).reflectivity*pow(rdotv, 10)*lights[lighti].color.y;
				pix.z += (*s).color.z*(*s).reflectivity*pow(rdotv, 10)*lights[lighti].color.z;
				pix.x += dotprod*(*s).color.x*lights[lighti].color.x;
				pix.y += dotprod*(*s).color.y*lights[lighti].color.y;
				pix.z += dotprod*(*s).color.z*lights[lighti].color.z;
			}
			else if(ob == triangleob)
			{
				float ndoti2 = 2*(lightdir.x*normal.x+lightdir.y*normal.y+lightdir.z*normal.z);
				V reflectedray(ndoti2*normal.x-lightdir.x, ndoti2*normal.y-lightdir.y, ndoti2*normal.z-lightdir.z);
				normalize(reflectedray);
				float rdotv = -1*(raydir.x*reflectedray.x+raydir.y*reflectedray.y+raydir.z*reflectedray.z);
				pix.x += (*tr).color.x*(*tr).reflectivity*pow(rdotv, 10)*lights[lighti].color.x;
				pix.y += (*tr).color.y*(*tr).reflectivity*pow(rdotv, 10)*lights[lighti].color.y;
				pix.z += (*tr).color.z*(*tr).reflectivity*pow(rdotv, 10)*lights[lighti].color.z;
				pix.x += dotprod*(*tr).color.x*lights[lighti].color.x;
				pix.y += dotprod*(*tr).color.y*lights[lighti].color.y;
				pix.z += dotprod*(*tr).color.z*lights[lighti].color.z;
			}
			else if(ob == planeob) {
				float ndoti2 = 2*(lightdir.x*normal.x+lightdir.y*normal.y+lightdir.z*normal.z);
				V reflectedray(ndoti2*normal.x-lightdir.x, ndoti2*normal.y-lightdir.y, ndoti2*normal.z-lightdir.z);
				normalize(reflectedray);
				float rdotv = -1*(raydir.x*reflectedray.x+raydir.y*reflectedray.y+raydir.z*reflectedray.z);
				pix.x += (*p).color.x * (*p).reflectivity * pow(rdotv, 10) * lights[lighti].color.x;
				pix.y += (*p).color.y * (*p).reflectivity * pow(rdotv, 10) * lights[lighti].color.y;
				pix.z += (*p).color.z * (*p).reflectivity * pow(rdotv, 10) * lights[lighti].color.z;
				pix.x += dotprod * (*p).color.x * lights[lighti].color.x;
				pix.y += dotprod * (*p).color.y * lights[lighti].color.y;
				pix.z += dotprod * (*p).color.z * lights[lighti].color.z;	
			}
		}
	}
	// reflection and refraction: recursive calling of raytrace gives global illumination
	if(depth < MAX_depth)
	{
		float refractindex = 1.0;
		if(ob == sphereob)
		{
			// if (raydir.dot(nhit) > 0) nhit = -nhit, inside = true; 
			bool inSphere = false;
			if(dotProduct(normal, raydir) > 0) {
				normal.x *= -1; normal.y *= -1; normal.z *= -1;
				inSphere = true;
			}
			refractindex = (*s).refractive_index;
			if((*s).reflectivity > 0)
			{
				float fratio = -1*(raydir.x*normal.x + raydir.y*normal.y + raydir.z*normal.z);
				float fresnel = 0.1*1 + 0.9*pow(1.0-fratio, 3);
				V reflectdir(raydir.x + 2*fratio*normal.x, raydir.y + 2*fratio*normal.y, raydir.z + 2*fratio*normal.z);
				normalize(reflectdir);
				intpoint.x += normal.x * bias;
				intpoint.y += normal.y * bias;
				intpoint.z += normal.z * bias;
				V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
				V reflectpoint = raytrace(biasedintpoint, reflectdir, depth+1);
				pix.x += reflectpoint.x * fresnel * (*s).color.x;
				pix.y += reflectpoint.y * fresnel * (*s).color.y;
				pix.z += reflectpoint.z * fresnel * (*s).color.z;
				diffuse = false;
			}
			if((*s).transparency > 0)
			{
				if(inSphere) {
					refractindex = 1/refractindex;
				}
				float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				float raydotn = -1*(normal.x*raydir.x+normal.y*raydir.y+normal.z*raydir.z);
				float k = 1.0-refractindex*refractindex*(1.0-raydotn*raydotn);
				V refractdir(raydir.x * refractindex + normal.x * (refractindex * raydotn-sqrt(k)),
							 raydir.y * refractindex + normal.y * (refractindex * raydotn-sqrt(k)),
							 raydir.z * refractindex + normal.z * (refractindex * raydotn-sqrt(k)));
				normalize(refractdir);
				V biasedintpoint(intpoint.x-10*normal.x*bias, intpoint.y-10*normal.y*bias, intpoint.z-10*normal.z*bias);
				V refractpoint = raytrace(biasedintpoint, refractdir, depth+1);
				pix.x += refractpoint.x * (1.0-fresnel) * (*s).transparency * (*s).color.x;
				pix.y += refractpoint.y * (1.0-fresnel) * (*s).transparency * (*s).color.y;
				pix.z += refractpoint.z * (1.0-fresnel) * (*s).transparency * (*s).color.z;
				diffuse = false;
			}
		}
		else if(ob == triangleob)
		{
			refractindex = (*tr).refractive_index;
			if((*tr).reflectivity > 0)
			{
				float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				V reflectdir(raydir.x+2*fratio*normal.x, raydir.y+2*fratio*normal.y, raydir.z+2*fratio*normal.z);
				normalize(reflectdir);
				V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
				V reflectpoint = raytrace(biasedintpoint, reflectdir, depth+1);
				pix.x += reflectpoint.x * fresnel * (*tr).color.x;
				pix.y += reflectpoint.y * fresnel * (*tr).color.y;
				pix.z += reflectpoint.z * fresnel * (*tr).color.z;
				diffuse = false;
			}
			if((*tr).transparency > 0)
			{
				float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				// float refractindex = 1.1;
				float raydotn = -1*(normal.x*raydir.x+normal.y*raydir.y+normal.z*raydir.z);
				float k = 1.0-refractindex*refractindex*(1.0-raydotn*raydotn);
				V refractdir(raydir.x*refractindex+normal.x*(refractindex*raydotn-sqrt(k)),
							 raydir.y*refractindex+normal.y*(refractindex*raydotn-sqrt(k)),
							 raydir.z*refractindex+normal.z*(refractindex*raydotn-sqrt(k)));
				normalize(refractdir);
				V biasedintpoint(intpoint.x-normal.x*bias, intpoint.y-normal.y*bias, intpoint.z-normal.z*bias);
				V refractpoint = raytrace(biasedintpoint, refractdir, depth+1);
				pix.x += refractpoint.x * (1.0-fresnel) * (*tr).transparency * (*tr).color.x;
				pix.y += refractpoint.y * (1.0-fresnel) * (*tr).transparency * (*tr).color.y;
				pix.z += refractpoint.z * (1.0-fresnel) * (*tr).transparency * (*tr).color.z;
				diffuse = false;
			}
		}
		else if(ob == planeob) {
			refractindex = (*p).refractive_index;
			if((*p).reflectivity > 0) {
				float fratio = -1*(raydir.x*normal.x+raydir.y*normal.y+raydir.z*normal.z);
				float fresnel = 0.1*1+0.9*pow(1.0-fratio, 3);
				V reflectdir(raydir.x+2*fratio*normal.x, raydir.y+2*fratio*normal.y, raydir.z+2*fratio*normal.z);
				normalize(reflectdir);
				V biasedintpoint(intpoint.x+normal.x*bias, intpoint.y+normal.y*bias, intpoint.z+normal.z*bias);
				V reflectpoint = raytrace(biasedintpoint, reflectdir, depth+1);
				pix.x += reflectpoint.x * fresnel * (*p).color.x;
				pix.y += reflectpoint.y * fresnel * (*p).color.y;
				pix.z += reflectpoint.z * fresnel * (*p).color.z;
				diffuse = false;
			}
			if((*p).transparency > 0)
			{
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
				pix.x += refractpoint.x * (1.0-fresnel) * (*p).transparency * (*p).color.x;
				pix.y += refractpoint.y * (1.0-fresnel) * (*p).transparency * (*p).color.y;
				pix.z += refractpoint.z * (1.0-fresnel) * (*p).transparency * (*p).color.z;
				diffuse = false;
			}
		}
	}
	// }
	return pix;
}

// evaluates pixel values of resulting image and stores in a file
void render2d(char* outfile)
{
	V ** image = new V * [height];
	for(int row = 0; row < height; row++)
	{
		image[row] = new vector3f[width];
	}

	V * pixeli;
	float fieldofview = 90;
	float whratio = (float)width/(float)height;
	float dwidth = 1.0/(float)width;
	float dheight = 1.0/(float)height;
	float angleofview = tan(M_PI*0.5*fieldofview/180);
	V orig(0.0, 0.0, 0.0);
	for(int yval = 0; yval < height; yval++)
	{
		pixeli = image[yval];
		for(int xval = 0; xval < width; xval++)
		{
			float imagex = (2*((xval+0.5)*dwidth)-1)*angleofview*whratio;
			float imagey = (1-2*((yval+0.5)*dheight))*angleofview;
			V raydir(imagex, imagey, -1);
			normalize(raydir);
			*pixeli = raytrace(orig, raydir, 0);
			pixeli++;
		}
	}

	// anti aliasing stuff
	// V ** imageNew = new V * [height];
	// for(int row = 0; row < height; row++)
	// {
	// 	imageNew[row] = new vector3f[width];
	// }
	
	// for(int y=0; y<height; y++) {
	// 	pixeli = imageNew[y];
	// 	for(int x=0; x<width; x++) {
	// 		float counter = 0.0;
	// 		V temppixel(0.0, 0.0, 0.0);
	// 		for(int yval = y-1; yval <= y+1; yval++) {
	// 			for(int xval = x-1; xval <= x+1; xval++) {
	// 				if(xval >= 0 && yval >= 0 && xval < width && yval < height) {
	// 					counter = counter + 1.0;
	// 					float imagex = (2*((xval+0.5)*dwidth)-1)*angleofview*whratio;
	// 					float imagey = (1-2*((yval+0.5)*dheight))*angleofview;
	// 					V raydir(imagex, imagey, -1);
	// 					normalize(raydir);
	// 					temppixel = raytrace(orig, raydir, 0);
	// 					if(yval == y && xval == x) 
	// 						pixeli->x += 2*temppixel.x, pixeli->y += 2*temppixel.y, pixeli->z += 2*temppixel.z;
	// 					else 
	// 						pixeli->x += temppixel.x, pixeli->y += temppixel.y, pixeli->z += temppixel.z;
	// 				}
	// 			}
	// 		}
	// 		pixeli->x = pixeli->x/(counter+1);
	// 		pixeli->y = pixeli->y/(counter+1);
	// 		pixeli->z = pixeli->z/(counter+1);
	// 		pixeli++;
	// 	}
	// }

	FILE *f = fopen(outfile, "wb");
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
			op >> s.reflectivity >> s.transparency >> s.refractive_index;
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
			// cout<<"tr.transparency "<<t.transparency<<endl;
			triangles.push_back(t);
			numtriangles++;
		}
		else if(objecttype == "cuboid") {
			struct cuboid c;
			op >> c.corner1.x >> c.corner1.y >> c.corner1.z;
			op >> c.corner2.x >> c.corner2.y >> c.corner2.z;
			op >> c.color.x >> c.color.y >> c.color.z;
			op >> c.reflectivity >> c.transparency >> c.refractive_index;
			cuboids.push_back(t);
			numcuboids++;
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
	if(argc < 3 || argc > 3)
	{
		cout << "Please follow the specifications!\nUnexpected arguments given.\n";
		cout << "Please run as: a.exe <inputfile>\n";
		exit(0);
	}

	parseinput(argv[1]);

	render2d(argv[2]);

	return 0;
}