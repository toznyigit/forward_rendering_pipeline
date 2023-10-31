#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;


/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/
Matrix4 modelling_transform(Mesh* mesh, Scene *scene){
	int size = mesh->transformationIds.size();
	Matrix4 transformations[size];
	for(int i=0;i<size;i++){
		char tType = mesh->transformationTypes[i];
		int tId = mesh->transformationIds[i];
		double arr[4][4];
		switch (tType)
		{
		case 's':
			{
			Scaling ss = *scene->scalings[tId-1];
			double arr[4][4] = {{ss.sx,0,0,0},
								{0,ss.sy,0,0},
								{0,0,ss.sz,0},
								{0,0,0,1}};
			transformations[i] = Matrix4 (arr);
			}
			break;
		case 't':
			{
			Translation tt = *scene->translations[tId-1];
			double arr[4][4] = {{1,0,0,tt.tx},
								{0,1,0,tt.ty},
								{0,0,1,tt.tz},
								{0,0,0,1}};
			transformations[i] = Matrix4 (arr);
			}
			break;
		case 'r':
			{
			Rotation rr = *scene->rotations[tId-1];
			Vec3 u(rr.ux, rr.uy, rr.uz, -1);
			Vec3 vv;
			double theMin = min(abs(rr.ux),min(abs(rr.uy),abs(rr.uz)));
			if(abs(rr.ux)==theMin){
				vv.x = 0; vv.y = -rr.uz; vv.z = rr.uy;
			}
			else if(abs(rr.uy)==theMin){
				vv.x = -rr.uz; vv.y = 0; vv.z = rr.ux;
			}
			else if(abs(rr.uz)==theMin){
				vv.x = -rr.uy; vv.y = rr.ux; vv.z = 0;
			}
			Vec3 ww(crossProductVec3(u,vv));

			Vec3 v(normalizeVec3(vv));
			Vec3 w(normalizeVec3(ww));

			double arr1[4][4] = {{u.x,v.x,w.x,0},
								{u.y,v.y,w.y,0},
								{u.z,v.z,w.z,0},
								{0,0,0,1}};
			Matrix4 Mneg(arr1);

			double rad = rr.angle*M_PIf32/180;
			double arr2[4][4] = {{1,0,0,0},
								{0,cos(rad),-sin(rad),0},
								{0,sin(rad),cos(rad),0},
								{0,0,0,1}};
			Matrix4 R(arr2);

			double arr3[4][4] = {{u.x,u.y,u.z,0},
								{v.x,v.y,v.z,0},
								{w.x,w.y,w.z,0},
								{0,0,0,1}};
			Matrix4 M(arr3);

			transformations[i] = multiplyMatrixWithMatrix(Mneg,multiplyMatrixWithMatrix(R,M));
			}
			break;
		default:
			break;
		}
	}
	Matrix4 mod(getIdentityMatrix());

	for(int i=0;i<size;i++){
		mod = multiplyMatrixWithMatrix(transformations[i], mod); //maybe wrong
	}
	return mod;
}

Matrix4 camera_transform(Camera *camera){
	double arrCamVec[4][4] =   {{camera->u.x, camera->u.y, camera->u.z, 0},
								{camera->v.x, camera->v.y, camera->v.z, 0},
								{camera->w.x, camera->w.y, camera->w.z, 0},
								{0,0,0,1}};
	double arrCamO[4][4] =  {{1,0,0,-camera->pos.x},
							 {0,1,0,-camera->pos.y},
							 {0,0,1,-camera->pos.z},
							 {0,0,0,1}};
	return multiplyMatrixWithMatrix(Matrix4(arrCamVec),Matrix4(arrCamO));

}

Matrix4 projection_transform(Camera *camera){
	double arrOrth[4][4] = {{2/(camera->right-camera->left),0,0,-(camera->right+camera->left)/(camera->right-camera->left)},
							{0,2/(camera->top-camera->bottom),0,-(camera->top+camera->bottom)/(camera->top-camera->bottom)},
							{0,0,-2/(camera->far-camera->near),-(camera->far+camera->near)/(camera->far-camera->near)},
							{0,0,0,1}};
	if(camera->projectionType){
		double arrPtoO[4][4] = {{camera->near,0,0,0},
								{0,camera->near,0,0},
								{0,0,camera->far+camera->near,camera->far*camera->near},
								{0,0,-1,0}};
		return multiplyMatrixWithMatrix(Matrix4(arrOrth),Matrix4(arrPtoO));

	}
	else{
		return Matrix4(arrOrth);
	}
}

Matrix4 viewport_transformation(Camera *camera){
	double arrVp[4][4] = {{camera->horRes*0.5,0,0,(camera->horRes-1)*0.5},
						  {0,camera->verRes*0.5,0,(camera->verRes-1)*0.5},
						  {0,0,0.5,0.5},
						  {0,0,0,1}};
	return Matrix4(arrVp);
}

struct Line{
	Vec4 begin;
	Vec4 end;
	bool visible;
};

int blendColor(int colorId1, int colorId2, double rO, double rN, Scene *scene){
	Color *blended = new Color();
	Color tmp1 = *(scene->colorsOfVertices[colorId1-1]);
	Color tmp2 = *(scene->colorsOfVertices[colorId2-1]);
	blended->r = floor(tmp1.r*(rO-rN)+tmp2.r*rN);
	blended->g = floor(tmp1.g*(rO-rN)+tmp2.g*rN);
	blended->b = floor(tmp1.b*(rO-rN)+tmp2.b*rN);
	scene->colorsOfVertices.push_back(blended);
	int blendedId = scene->colorsOfVertices.size();
	return blendedId;
}

bool visible(double dist, double dif, double *enter, double *leave){
	if(dist){
		double t = dif/dist;
		if(dist>0){
			if(t > *leave){ return false;}
			if(t > *enter){ *enter = t;}
		}
		else{
			if(t < *leave){ *leave = t;}
			if(t < *enter){ return false;}
		}
	}
	else{
		if(dif > 0) return false;
	}
	return true;
}

void clipper(Vec4* begin, Vec4* end, Scene* scene, vector<Line>* payload){
	Line theLine;
	theLine.begin = *begin;
	theLine.end = *end;
	theLine.visible = false;
	double te = 0;
	double tl = 1;
	double distx = end->x-begin->x;
	double disty = end->y-begin->y;
	double distz = end->z-begin->z;
	double minmax = 1;
	double x, y, z, ratioNew, ratioOrg;
	if(visible(distx, -minmax-begin->x, &te, &tl)){ // left
		if(visible(-distx, begin->x-minmax, &te, &tl)){ // right
			if(visible(disty, -minmax-begin->y, &te, &tl)){ // bottom
				if(visible(-disty, begin->y-minmax, &te, &tl)){ // top
					if(visible(distz, -minmax-begin->z, &te, &tl)){ // front
						if(visible(-distz, begin->z-minmax, &te, &tl)){ // back
							ratioOrg = sqrt(distx*distx+disty*disty+distz*distz);
							theLine.visible = true;
							if(tl<1){
								x = begin->x+distx*tl;
								y = begin->y+disty*tl;
								z = begin->z+distz*tl;
								theLine.end.x = x;
								theLine.end.y = y;
								theLine.end.z = z;
								theLine.end.t = minmax;
								ratioNew = sqrt((begin->x-x)*(begin->x-x)+(begin->y-y)*(begin->y-y)+(begin->z-z)*(begin->z-z));
								theLine.end.colorId = blendColor(begin->colorId, end->colorId, ratioOrg, ratioNew, scene);
							}
							if(te>0){
								x = begin->x+distx*te;
								y = begin->y+disty*te;
								z = begin->z+distz*te;
								theLine.begin.x = x;
								theLine.begin.y = y;
								theLine.begin.z = z;
								theLine.begin.t = minmax;
								ratioNew = sqrt((begin->x-x)*(begin->x-x)+(begin->y-y)*(begin->y-y)+(begin->z-z)*(begin->z-z));
								theLine.begin.colorId = blendColor(begin->colorId, end->colorId, ratioOrg, ratioNew, scene);
							}
							payload->push_back(theLine);
						}
					}
				}
			}
		}
	}
}

bool backfaceCulling(Vec3 ver1, Vec3 ver2, Vec3 ver3)
{
    Vec3 edge12 = subtractVec3(ver2, ver1);
    Vec3 edge13 = subtractVec3(ver3, ver1);
    Vec3 normal = normalizeVec3(crossProductVec3(edge12, edge13));
    double angle = dotProductVec3(normal, ver1);
    return (angle < 0);
}

vector <Line> lineGenerator(vector<Vec4>* rearrangedVertices, Mesh* mesh, Scene* scene, Vec3 gaze, bool culling){
	vector <Line> result;
	
	for(int i=0;i<mesh->numberOfTriangles;i++){
		int id1 = mesh->triangles[i].getFirstVertexId();
		int id2 = mesh->triangles[i].getSecondVertexId();
		int id3 = mesh->triangles[i].getThirdVertexId();
		Vec4 v1 = (*rearrangedVertices)[id1-1];
		Vec4 v2 = (*rearrangedVertices)[id2-1];
		Vec4 v3 = (*rearrangedVertices)[id3-1];
		if(!backfaceCulling(Vec3 (v1.x, v1.y, v1.z, -1), Vec3 (v2.x, v2.y, v2.z, -1), Vec3 (v3.x, v3.y, v3.z, -1)) || !culling){
			clipper(&v1, &v2, scene, &result);
			clipper(&v2, &v3, scene, &result);
			clipper(&v3, &v1, scene, &result);
		}
	}
	return result;
}

bool doubleCheck(int x, int y, int lHor, int lVer){
	if((0 <= x && x < lHor) && (0 <= y && y < lVer)) return true;
	else return false;
}

void drawLine(Line line, Camera* camera, Scene* scene){
	Vec4 begin, end;
	double slope = (line.end.y-line.begin.y)/(line.end.x-line.begin.x);
	// if((line.end.x-line.begin.x)==0 && (line.end.y-line.begin.y)>0) slope = MAXFLOAT;
	// else if((line.end.x-line.begin.x)==0 && (line.end.y-line.begin.y)<0) slope = -MAXFLOAT;
	// else slope = (line.end.y-line.begin.y)/(line.end.x-line.begin.x);
	double d, dx, dy;
	int x, y;
	Color c_curr, c_begin, c_end;

	//cout << line.begin << " | " << (*scene->colorsOfVertices[line.begin.colorId-1]) << " , " << line.end << " | " << (*scene->colorsOfVertices[line.end.colorId-1]) << " | Slope: ";
	if(abs(slope)<1 && slope>=0){ // NE & E
		// cout << slope << " NE & E " << endl;
		if(line.begin.x<line.end.x){
			begin = line.begin;
			end = line.end;
		}
		else{
			begin = line.end;
			end = line.begin;
		}
		// cout << "NE & E: " << begin << ", " << end << endl;
		y = begin.y+0.5;
		dx = end.x-begin.x;
		dy = begin.y-end.y;
		d = 2*dy+dx;
		c_begin = (*scene->colorsOfVertices[begin.colorId-1]);
		c_end = (*scene->colorsOfVertices[end.colorId-1]);
		c_curr = c_begin;
		Color c_diff((c_end-c_begin)/dx);
		for(x=(begin.x+0.5);x<end.x;x++){
			// cout << x << ", " << y << endl;
			if(!doubleCheck(x, y, camera->horRes, camera->verRes)) continue;
			scene->image[x][y] = c_curr;
			if(d<0){
				y++;
				d+=2*(dy+dx);
			}
			else{
				d+=2*dy;
			}
			c_curr+=c_diff;
		}
	}
	else if(abs(slope)<1 && slope<0){ // NW & W
		//cout << slope << " NW & W " << endl;
		if(line.begin.x>line.end.x){
			begin = line.begin;
			end = line.end;
		}
		else{
			begin = line.end;
			end = line.begin;
		}
		// cout << "NW & W: " << begin << ", " << end << endl;
		y = begin.y+0.5;
		dx = begin.x-end.x;
		dy = begin.y-end.y;
		d = 2*dy+dx;
		c_begin = (*scene->colorsOfVertices[begin.colorId-1]);
		c_end = (*scene->colorsOfVertices[end.colorId-1]);
		c_curr = c_begin;
		Color c_diff((c_begin-c_end)/dx);
		for(x=(begin.x+0.5);x>end.x;x--){
			// cout << x << ", " << y << " d: " << d << endl;
			if(!doubleCheck(x, y, camera->horRes, camera->verRes)) continue;
			scene->image[x][y] = c_curr;
			if(d<0){
				y++;
				d+=2*(dy+dx);
			}
			else{
				d+=2*dy;
			}
			c_curr-=c_diff;
		}
	}
	else if(abs(slope)>=1 && slope>=0){ // NE & N
		//cout << slope << " NE & N " << endl;
		if(line.begin.y<line.end.y){
			begin = line.begin;
			end = line.end;
		}
		else{
			begin = line.end;
			end = line.begin;
		}
		//cout << "NE & N: " << begin << ", " << end << endl;
		x = begin.x+0.5;
		dx = begin.x-end.x;
		dy = end.y-begin.y;
		d = dy+2*dx;
		c_begin = (*scene->colorsOfVertices[begin.colorId-1]);
		c_end = (*scene->colorsOfVertices[end.colorId-1]);
		c_curr = c_begin;
		Color c_diff((c_end-c_begin)/dy);
		for(y=(begin.y+0.5);y<end.y;y++){
			// cout << x << ", " << y << endl;
			if(!doubleCheck(x, y, camera->horRes, camera->verRes)) continue;
			scene->image[x][y] = c_curr;
			if(d<0){
				x++;
				d+=2*(dy+dx);
			}
			else{
				d+=2*dx;
			}
			c_curr+=c_diff;
		}
	}
	else if(abs(slope)>=1 && slope<0){ // NW & N
		//cout << slope << " NW & N " << endl;
		if(line.begin.y<line.end.y){
			begin = line.begin;
			end = line.end;
		}
		else{
			begin = line.end;
			end = line.begin;
		}
		// cout << "NW & N: " << begin << ", " << end << endl;
		x = begin.x+0.5;
		dx = end.x-begin.x;
		dy = end.y-begin.y;
		d = dy+2*dx;
		c_begin = (*scene->colorsOfVertices[begin.colorId-1]);
		c_end = (*scene->colorsOfVertices[end.colorId-1]);
		c_curr = c_begin;
		Color c_diff((c_begin-c_end)/dy);
		for(y=(begin.y+0.5);y<end.y;y++){
			// cout << x << ", " << y << endl;
			if(!doubleCheck(x, y, camera->horRes, camera->verRes)) continue;
			scene->image[x][y] = c_curr;
			if(d<0){
				x--;
				d+=2*(dy+dx);
			}
			else{
				d+=2*dx;
			}
			c_curr-=c_diff;
		}
	}
}

int minmaxThree(int i1, int i2, int i3, bool needMin){
	if(needMin){ //true -> return min(i1, i2, i3)
		int tmp = min(i1,i2);
		tmp = min(tmp,i3);
		return tmp;
	}
	else{ //false -> return max(i1, i2, i3)
		int tmp = max(i1,i2);
		tmp = max(tmp,i3);
		return tmp;
	}
}

double barycentric(double x, double y, Vec4* vA, Vec4* vB){
	return x*(vA->y-vB->y)+y*(vB->x-vA->x)+vA->x*vB->y-vA->y*vB->x;
}

void drawTriangle(vector<Vec4>* rearrangedVertices, int v1Id, int v2Id, int v3Id, int xmax, int ymax, Scene* scene, bool culling){
	int lowBorderX, highBorderX, lowBorderY, highBorderY, x, y;
	double a, b, c, f23, f31, f12;
	Vec4 v1 = (*rearrangedVertices)[v1Id-1];
	Vec4 v2 = (*rearrangedVertices)[v2Id-1];
	Vec4 v3 = (*rearrangedVertices)[v3Id-1];
	if(!backfaceCulling(Vec3 (v1.x, v1.y, v1.z, -1), Vec3 (v2.x, v2.y, v2.z, -1), Vec3 (v3.x, v3.y, v3.z, -1)) || !culling){
		Color c1 = (*scene->colorsOfVertices[v1Id-1]);
		Color c2 = (*scene->colorsOfVertices[v2Id-1]);
		Color c3 = (*scene->colorsOfVertices[v3Id-1]);
		f23 = barycentric(v1.x, v1.y, &v2, &v3);
		f31 = barycentric(v2.x, v2.y, &v3, &v1);
		f12 = barycentric(v3.x, v3.y, &v1, &v2);

		lowBorderX = minmaxThree(v1.x,v2.x,v3.x,true);
		highBorderX = minmaxThree(v1.x,v2.x,v3.x,false);
		lowBorderY = minmaxThree(v1.y,v2.y,v3.y,true);
		highBorderY = minmaxThree(v1.y,v2.y,v3.y,false);

		lowBorderX = max(lowBorderX, 0);
		highBorderX = min(highBorderX, xmax-1);
		lowBorderY = max(lowBorderY, 0);
		highBorderY = min(highBorderY, ymax-1);

		for(x=lowBorderX;x<=highBorderX;x++){
			for(y=lowBorderY;y<=highBorderY;y++){
				a = barycentric(x, y, &v2, &v3)/f23;
				b = barycentric(x, y, &v3, &v1)/f31;
				c = barycentric(x, y, &v1, &v2)/f12;
				if((a>=0)&&(b>=0)&&(c>=0)) scene->image[x][y] = c1*a+c2*b+c3*c;
			}
		}
	}
}

void Scene::forwardRenderingPipeline(Camera *camera)
{
	Matrix4 cam = camera_transform(camera);
	Matrix4 pro = projection_transform(camera);
	Matrix4 PxC = multiplyMatrixWithMatrix(pro, cam);
	Matrix4 vp = viewport_transformation(camera);
	// cout << vp << endl;

	for(int i=0;i<meshes.size();i++){
		// Matrix4 mod = getIdentityMatrix();
		Matrix4 mod = modelling_transform(meshes[i], this);
		Matrix4 PxCxM = multiplyMatrixWithMatrix(PxC, mod);
		vector< Vec4 > rearranged_vertices;
		//rearranged_vertices.resize(vertices.size()+1);

		for(int j=0;j<vertices.size();j++){
			Vec4 tmp(vertices[j]->x, vertices[j]->y, vertices[j]->z, 1.0, vertices[j]->colorId);
			tmp = multiplyMatrixWithVec4(PxCxM,tmp);
			tmp.division();
			rearranged_vertices.push_back(tmp);
			//rearranged_vertices[j]=tmp;
		}

		if(meshes[i]->type){
			for(int j=0;j<rearranged_vertices.size();j++){
				rearranged_vertices[j] = multiplyMatrixWithVec4(vp, rearranged_vertices[j]);
			}
			for(int j=0;j<meshes[i]->numberOfTriangles;j++){
				int tId1 = meshes[i]->triangles[j].getFirstVertexId();
				int tId2 = meshes[i]->triangles[j].getSecondVertexId();
				int tId3 = meshes[i]->triangles[j].getThirdVertexId();
				drawTriangle(&rearranged_vertices, tId1, tId2, tId3, camera->horRes, camera->verRes, this, cullingEnabled);
			}
		}
		else{
			vector<Line> clipped_lines = lineGenerator(&rearranged_vertices, meshes[i], this, camera->gaze, cullingEnabled);
			rearranged_vertices.clear();
			for(int j=0;j<clipped_lines.size();j++){
				// clipped_lines[j].begin.division();
				clipped_lines[j].begin = multiplyMatrixWithVec4(vp, clipped_lines[j].begin);
				// clipped_lines[j].end.division();
				clipped_lines[j].end = multiplyMatrixWithVec4(vp, clipped_lines[j].end);
			}
			for(int j=0;j<clipped_lines.size();j++){
				if(clipped_lines[j].visible){
					// cout << j << " Line: " << clipped_lines[j].begin << "," << clipped_lines[j].end << endl;
					drawLine(clipped_lines[j], camera, this);
				}
			}
			clipped_lines.clear();
		}
	}
}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL) {
		str = pElement->GetText();

		if (strcmp(str, "enabled") == 0) {
			cullingEnabled = true;
		}
		else {
			cullingEnabled = false;
		}
	}

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		// read projection type
		str = pCamera->Attribute("type");

		if (strcmp(str, "orthographic") == 0) {
			cam->projectionType = 0;
		}
		else {
			cam->projectionType = 1;
		}

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read meshes
	pElement = pRoot->FirstChildElement("Meshes");

	XMLElement *pMesh = pElement->FirstChildElement("Mesh");
	XMLElement *meshElement;
	while (pMesh != NULL)
	{
		Mesh *mesh = new Mesh();

		pMesh->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = pMesh->Attribute("type");

		if (strcmp(str, "wireframe") == 0) {
			mesh->type = 0;
		}
		else {
			mesh->type = 1;
		}

		// read mesh transformations
		XMLElement *pTransformations = pMesh->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *clone_str;
		int v1, v2, v3;
		XMLElement *pFaces = pMesh->FirstChildElement("Faces");
        str = pFaces->GetText();
		clone_str = strdup(str);

		row = strtok(clone_str, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);

			if (result != EOF) {
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		meshes.push_back(mesh);

		pMesh = pMesh->NextSiblingElement("Mesh");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;
	fout.open(camera->outputFileName.c_str());
	cout << camera->outputFileName.c_str() << endl;

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}