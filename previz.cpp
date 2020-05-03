#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <float.h>
#include "SETTINGS.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "motion.h"
#include "perlin.h"

using namespace std;

//PerlinNoise pn;

// Stick-man classes
DisplaySkeleton displayer;    
Skeleton* skeleton;
Motion* motion;

int windowWidth = 640;
int windowHeight = 480;

// scene geometry
vector<VEC3> sphereCenters;
vector<float> sphereRadii;
vector<VEC3> sphereColors;

vector<vector<VEC3> > triangleVertices;
vector<VEC3> triangleColors;

vector<int> portal;

vector<VEC3> cylinderColors;
vector<float> cylinderRadii;
vector<MATRIX4> cylinderRotation;
vector<MATRIX4> cylinderScaling;
vector<VEC4> cylinderTranslation;
vector<VEC4> cylinderRightVertex;
vector<VEC4> cylinderLeftVertex;
vector<float> cylinderLengths;

class Light {
  public:
    VEC3 position;
    VEC3 color;
    VEC3 direction;
  
  Light(VEC3 pos, VEC3 co, VEC3 direction) : position(pos), color(co), direction(direction) {}
};

bool hitLight(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal);
VEC3 shading(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal, VEC3& color);
VEC3 lambertian(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal);
VEC3 specular(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal);
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void writePPM(const string& filename, int& xRes, int& yRes, const float* values){
  int totalCells = xRes * yRes;
  unsigned char* pixels = new unsigned char[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    pixels[i] = values[i];

  FILE *fp;
  fp = fopen(filename.c_str(), "wb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for writing." << endl;
    cout << " Make sure you're not trying to write from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  fprintf(fp, "P6\n%d %d\n255\n", xRes, yRes);
  fwrite(pixels, 1, totalCells * 3, fp);
  fclose(fp);
  delete[] pixels;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

bool rayTriangleIntersection(const VEC3& dVal, const VEC3& eye, int& y, float& t, VEC3& normal){
  vector<VEC3> vertices = triangleVertices[y];
  VEC3 a = vertices[0]; VEC3 b = vertices[1]; VEC3 c = vertices[2];
  VEC3 e1 = b - a;
  VEC3 e2 = c - a;
  VEC3 h = dVal.cross(e2);
  Real parallel = e1.dot(h);
  Real smallNum = 0.0001;

  Real f = 1.0 / parallel;
  VEC3 s = (eye - a);
  //scalar dot product to find determinant
  Real u = f * s.dot(h);
  VEC3 q = s.cross(e1);
  //scalar dot product to find determinant
  Real v = f * dVal.dot(q);

  t = f * e2.dot(q);

  if(parallel >= smallNum && fabs(parallel) >= smallNum){
    if((u >= 0.0 && u <= 1.0) && (v >= 0.0 && u+v <= 1.0)){
      if(t > smallNum) {
        normal = (e1).cross(e2);
        normal = normal / normal.norm();
        return true;
      }
    }
  }
  return false;
}

bool rayTriCircleIntersection(const VEC3& dVal, const VEC3& eye, int& y, float& t){
  vector<VEC3> vertices = triangleVertices[y];

  VEC3 a = vertices[0]; VEC3 b = vertices[1]; VEC3 c = vertices[2];
  VEC3 e1 = b - a;
  VEC3 e2 = c - a;
  VEC3 h = dVal.cross(e2);
  Real parallel = e1.dot(h);
  Real smallNum = 0.00000001;

  if(parallel > -smallNum && parallel < smallNum){
    return false;
  }

  Real f = 1.0 / parallel;
  VEC3 s = (eye - a);
  //scalar dot product to find determinant
  Real u = f * s.dot(h);
  if(u < 0.0 || u > 1.0){
    return false;
  }

  VEC3 q = s.cross(e1);
  //scalar dot product to find determinant
  Real v = f * dVal.dot(q);
  if(v < 0.0 || u + v > 1.0) {
    return false;
  }

  t = f * e2.dot(q);

  if (t > smallNum) {
    return true;
  }
  return false;
}

bool raySphereIntersect(const VEC3& rayPos, 
                        const VEC3& rayDir,
                        const int& y,
                        VEC3& normal,
                        float& t) {

  VEC3 center = sphereCenters[y];
  float radius = sphereRadii[y];

  float quad1 = (rayDir).dot(rayDir);
  float quad2 = (2.0 * rayDir).dot(rayPos - center);
  float quad3 = (rayPos-center).dot(rayPos-center) - (radius*radius);
  float det = (quad2*quad2) - (4.0*quad1*quad3);

  if(det >= 0.0){
    t = (-quad2 - sqrt(det))/(2.0*quad1);
    if(t < 0.0) {
      t = (-quad2 + sqrt(det))/(2.0*quad1);
    }
    normal = (rayPos + (t * rayDir)) - center;
    normal = normal / normal.norm();
    return true;
  }

  return false;
}


bool rayCylinderIntersection(const VEC3& rayDir, 
                             const VEC3& rayPos,
                             VEC3& normal,  
                             int y,
                             float& t
                            ){
  float radius = cylinderRadii[y];
  MATRIX4 rotation = cylinderRotation[y];
  MATRIX4 scaling = cylinderScaling[y];
  VEC4 translation = cylinderTranslation[y];
  VEC4 rightVertex = cylinderRightVertex[y];
  VEC4 leftVertex = cylinderLeftVertex[y];
  float length = cylinderLengths[y];

  //transform ray into cylinder's local frame
  MATRIX4 rotate = scaling.inverse() * rotation.inverse();
  //rotate = rotate.inverse();
  VEC4 tempPos = VEC4(rayPos[0], rayPos[1], rayPos[2], 0.0);
  VEC4 tempO = rotate * (tempPos - translation);
  VEC3 oVector = VEC3(tempO[0], tempO[1], tempO[2]);

  VEC4 tempDir = VEC4(rayDir[0], rayDir[1], rayDir[2], 0.0);
  VEC4 tempD = rotate * tempDir;
  VEC3 dVector = VEC3(tempD[0], tempD[1], tempD[2]);

  Real a = (dVector[0]*dVector[0]) + (dVector[1]*dVector[1]);
  Real b = (2*dVector[0]*oVector[0]) + (2*dVector[1]*oVector[1]);
  Real c = (oVector[0]*oVector[0]) + (oVector[1]*oVector[1]) - (radius*radius);

  Real determinant = (b*b) - (4*a*c);

  Real smallNum = 0.00000001;

  if (determinant < smallNum){
    return false;
  }

  t = (- b - sqrt(determinant))/(2*a);

  if(t < smallNum){
    t = (- b + sqrt(determinant))/(2*a);
    if(t < smallNum) {
      return false;
    }
  }

  VEC3 intersection = oVector + (t*dVector);

  VEC3 left = VEC3(rightVertex[0], rightVertex[1], rightVertex[2]);
  VEC3 right = VEC3(leftVertex[0], leftVertex[1], leftVertex[2]);

  //cout << intersection[2] << length << endl;
  if(intersection[2] > length || intersection[2] < 0){
    return false;
  }

  VEC4 rotateCenter = (scaling * rotation) * (((rightVertex - leftVertex) / 2) + translation);
  VEC3 center = VEC3(rotateCenter[0], rotateCenter[1], rotateCenter[2]);
  VEC4 rotateBack = (scaling * rotation) * (VEC4(intersection[0], intersection[1], intersection[2], 0) + translation);
  intersection[0] = rotateBack[0];
  intersection[1] = rotateBack[1];
  intersection[2] = rotateBack[2];

  normal = (intersection - center) / (intersection - center).norm();

  return true;
}
//////////////////////////////////////////////////////////////////////////////////
bool intersectScene(const VEC3& rayDir, const VEC3& rayPos, int& hitID, float& t, int& shape, VEC3& normal) {
  hitID = -1;
  float tMinCylinder = FLT_MAX;
  float tMinSphere = FLT_MAX;
  float tMinTriangle = FLT_MAX;
  VEC3 cNorm;
  VEC3 tNorm;
  VEC3 sNorm;
  int cHit;
  int tHit;
  int sHit;

  for(int y = 0; y < cylinderRadii.size(); y++) 
  {
    float tMin = FLT_MAX;
    if(rayCylinderIntersection(rayDir, rayPos, cNorm, y, tMin))
    {
      if (tMin < tMinCylinder && tMin > 0.0) 
      {
        tMinCylinder = tMin;
        cHit = y;
        hitID = y;
      }
    }
  }
  for(int y = 0; y < sphereCenters.size() - 3; y++) 
  {
    float tMin = FLT_MAX;
    if(raySphereIntersect(rayPos, rayDir, y, sNorm, tMin)) 
    {
      if(tMin < tMinSphere && tMin > 0.0) 
      {
        tMinSphere = tMin;
        sHit = y;
        hitID = y;
      }
    }
  }
  for(int y = 0; y < triangleColors.size(); y++) 
  {
    float tMin = FLT_MAX;
      if(portal[y] == 3 && rayTriangleIntersection(rayDir, rayPos, y, tMin, tNorm)) {
      float sMin = FLT_MAX;
      if(raySphereIntersect(rayPos, rayDir, 4, sNorm, sMin))
      {
        continue;
      }

      else{
        if(tMin < tMinTriangle && tMin > 0.0) 
        {
          tMinTriangle = tMin;
          tHit = y;
          hitID = y;
        }
      }
    } 
    else if(portal[y] == 2 && rayTriangleIntersection(rayDir, rayPos, y, tMin, tNorm)) {
      float sMin = FLT_MAX;
      if(raySphereIntersect(rayPos, rayDir, 3, sNorm, sMin))
      {
        continue;
      }

      else{
        if(tMin < tMinTriangle && tMin > 0.0) 
        {
          tMinTriangle = tMin;
          tHit = y;
          hitID = y;
        }
      }
    } 
    else if (portal[y] == 1 && rayTriangleIntersection(rayDir, rayPos, y, tMin, tNorm)) {
      float sMin = FLT_MAX;
      if(raySphereIntersect(rayPos, rayDir, 2, sNorm, sMin))
      {
        continue;
      }

      else{
        if(tMin < tMinTriangle && tMin > 0.0) 
        {
          tMinTriangle = tMin;
          tHit = y;
          hitID = y;
        }
      }
    } 
    else {
      if(rayTriangleIntersection(rayDir, rayPos, y, tMin, tNorm))
      {
        if(tMin < tMinTriangle && tMin > 0.0) 
        {
          //three vertices
          tMinTriangle = tMin;
          tHit = y;
          hitID = y;
        }
      }
    }
  }

  if(hitID == -1) {
    return false;
  }

  if((tMinCylinder < tMinTriangle) && (tMinCylinder < tMinSphere)) 
  {
    hitID = cHit;
    shape = 1;
    t = tMinCylinder;
    normal = cNorm;
  }
  else if ((tMinSphere < tMinCylinder) && (tMinSphere < tMinTriangle)) 
  {
    hitID = sHit;
    shape = 2;
    t = tMinSphere;
    normal = sNorm;
  }
  else if ((tMinTriangle < tMinSphere) && (tMinTriangle < tMinCylinder)) 
  {
    hitID = tHit;
    shape = 3;
    t = tMinTriangle;
    normal = tNorm;
  }

  return true;
}

Real clampColor(Real value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
}
//////////////////////////////////////////////////////////////////////////////////
void rayColor(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor) 
{
  pixelColor = VEC3(0.381, 0.358, 0.393);

  vector<Light> lights;
  Light light1 = Light(VEC3(0, 5.5, 0), VEC3(0.988, 0.965, 1), VEC3(0, 0, 0));
  lights.push_back(light1);
  // Light light2 = Light(VEC3(0, 200.5, 0), VEC3(1, 1, 1), VEC3(0, 0, 0));
  // lights.push_back(light2);
  //   Light light3 = Light(VEC3(0, 12.5, 0), VEC3(0.371, 0.348, 0.383), VEC3(0, 0, 0));
  // lights.push_back(light3);

  int hitID;
  float t;
  int shape;
  VEC3 normal;
  bool intersect = intersectScene(rayDir, rayPos, hitID, t, shape, normal);

  if(intersect == false) {
    return;
  }

  VEC3 colorSum = VEC3(0, 0, 0); 
  if(shape == 3)  //triangle
  {
    VEC3 intersection = rayPos + (t * rayDir);

    for( int i = 0; i < lights.size(); i = i+1){
      if(hitLight(lights[i], rayDir, rayPos, t, normal)) {
        //cout << "in shadow" << endl;
        VEC3 color = shading(lights[i], rayDir, rayPos, t, normal, triangleColors[hitID]);
        //cout << color[0] << " " << color[1] << " " << color[2] << endl;
        
        colorSum = colorSum + color;
        //cout << colorSum[0] << " " << colorSum[1] << " " << colorSum[2] << endl;
      }
    }
    // colorSum[0] = clampColor(colorSum[0] + noise);
    // colorSum[1] = clampColor(colorSum[1] + noise);
    // colorSum[2] = clampColor(colorSum[2] + noise);./
    pixelColor = colorSum;
    
    //pixelColor = triangleColors[hitID];
  }
  else if (shape == 1) //cylinder
  {
    //cout << cylinderColors[hitID][0] << endl;
    for( int i = 0; i < lights.size(); i = i+1){
      if(hitLight(lights[i], rayDir, rayPos, t, normal)) {
        //cout << "in shadow" << endl;
        VEC3 color = shading(lights[i], rayDir, rayPos, t, normal, cylinderColors[hitID]);
        //cout << color[0] << " " << color[1] << " " << color[2] << endl;
        colorSum = colorSum + color;
        //cout << colorSum[0] << " " << colorSum[1] << " " << colorSum[2] << endl;
      }
    }
    pixelColor = colorSum;
  }
  else if (shape == 2) //sphere
  {
    //cout << cylinderColors[hitID][0] << endl;
    for( int i = 0; i < lights.size(); i = i+1){
      if(hitLight(lights[i], rayDir, rayPos, t, normal)) {
        //cout << "in shadow" << endl;
        VEC3 color = shading(lights[i], rayDir, rayPos, t, normal, sphereColors[hitID]);
        //cout << color[0] << " " << color[1] << " " << color[2] << endl;
        colorSum = colorSum + color;
        //cout << colorSum[0] << " " << colorSum[1] << " " << colorSum[2] << endl;
      }
    }
    pixelColor = colorSum;
    //cout << pixelColor[0] << " " << pixelColor[1] << " " << pixelColor[2] << endl;
  }
}

bool hitLight(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal){
    VEC3 shadowPos = rayPos + t * rayDir;
    shadowPos = shadowPos + (normal*0.01);
    VEC3 shadowDir = (singleLight.position - shadowPos).normalized();
    
    VEC3 norm = normal;
    int hitID;
    int shape;
    float tempT;
    bool inter = intersectScene(shadowDir, shadowPos, hitID, tempT, shape, norm);
    if(inter == true){
      return false;
    }
    return true;
}

VEC3 shading(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal, VEC3& color){
  VEC3 newColor;
  VEC3 l = lambertian(singleLight, rayDir, rayPos, t, normal);
  VEC3 s = specular(singleLight, rayDir, rayPos, t, normal);
  newColor = color.cwiseProduct(l);
  return newColor;
}

VEC3 lambertian(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal){
  //the ray is the intersection point
  VEC3 ray = rayPos + (t * rayDir);
  //the light direction is the light ray emitted from the sphere
  singleLight.direction = singleLight.position - ray;
  //normalizing both values
  VEC3 normalizedLight = singleLight.direction / singleLight.direction.norm();
  //project light onto the normal
  normal = normal.normalized();
  Real nDotl = normal.dot(normalizedLight);
  VEC3 lambertian = singleLight.color * std::max(0.0, nDotl);
  return lambertian;
}

VEC3 specular(Light singleLight, const VEC3& rayDir, const VEC3& rayPos, float& t, VEC3& normal){
    //the ray is the intersection point
  VEC3 ray = rayPos + t * rayDir;
  //the light direction is the light ray emitted from the sphere
  singleLight.direction = singleLight.position - ray;
  //normalizing both values
  VEC3 normalizedLight = singleLight.direction.normalized();

  Real phong = 10.0;
  // //v is the eye ray emitted from the sphere
  VEC3 v = rayPos - ray;
  VEC3 normalizedV = v / v.norm();
  //might want to normalize light first
  VEC3 r = -normalizedLight + 2*(normalizedLight.dot(normal))*normal;
  VEC3 normalizedR = r / r.norm();
  Real nDoth = normalizedR.dot(normalizedV);

  VEC3 specular = singleLight.color * pow(std::max(0.0, nDoth), phong);
  return specular;
}

void buildCylinders()
{
  cylinderRadii.clear();
  cylinderColors.clear();
  cylinderRotation.clear();
  cylinderTranslation.clear();
  cylinderRightVertex.clear();
  cylinderLeftVertex.clear();
  cylinderLengths.clear();

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // build a sphere list, but skip the first bone, 
  // it's just the origin
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];


    cylinderLengths.push_back(lengths[x]);
    cylinderRotation.push_back(rotation);
    cylinderTranslation.push_back(translation);
    cylinderScaling.push_back(scaling);

    // get the endpoints of the cylinder
    // left is top, right is bottom
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;
    cylinderLeftVertex.push_back(leftVertex);
    cylinderRightVertex.push_back(rightVertex);

    //store the spheres
    cylinderRadii.push_back(0.35);
    cylinderColors.push_back(VEC3(1,1,1));
    
  }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
float clamp(float value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void renderImage(int& xRes, int& yRes, const string& filename) {

  VEC3 eye(-6, 0.5, 1);
  vector<VEC4>& translations = displayer.translations();
  VEC4 pelvisTranslation = translations[1];
  VEC3 lookingAt = VEC3(pelvisTranslation[0], pelvisTranslation[1], pelvisTranslation[2]);
  //cout << pelvisTranslation[0] << " " << pelvisTranslation[1] << " " << pelvisTranslation[2] << " " << endl;
  VEC3 up(0,1,0); 
  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // compute image plane
  const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
  const float halfX = halfY * 4.0f / 3.0f;

  const VEC3 cameraZ = (lookingAt - eye).normalized();
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  for (int y = 0; y < yRes; y++) 
    for (int x = 0; x < xRes; x++) 
    {
      // generate the ray, making x-axis go left to right
      const float ratioX = 1.0f - ((xRes - 1) - x) / float(xRes) * 2.0f;
      const float ratioY = 1.0f - y / float(yRes) * 2.0f;
      const VEC3 rayHitImage = lookingAt + 
                               ratioX * halfX * cameraX +
                               ratioY * halfY * cameraY;
      const VEC3 rayDir = (rayHitImage - eye).normalized();

      // if(x == 318 && y == 37) {
      //   cout << "hit" << endl;
      // }

      // get the color
      VEC3 color;
      rayColor(eye, rayDir, color);

      // set, in final image
      ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  writePPM(filename, xRes, yRes, ppmOut);

  delete[] ppmOut;
}

//////////////////////////////////////////////////////////////////////////////////
// Load up a new motion captured frame
//////////////////////////////////////////////////////////////////////////////////
void setSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  if (displayer.GetSkeletonMotion(0) != NULL)
  {
    int postureID;
    if (frameIndex >= displayer.GetSkeletonMotion(0)->GetNumFrames())
    {
      cout << " We hit the last frame! You might want to pick a different sequence. " << endl;
      postureID = displayer.GetSkeletonMotion(0)->GetNumFrames() - 1;
    }
    else 
      postureID = frameIndex;
    displayer.GetSkeleton(0)->setPosture(* (displayer.GetSkeletonMotion(0)->GetPosture(postureID)));
  }
}
//////////////////////////////////////////////////////////////////////////////////
// Build a list of spheres in the scene
//////////////////////////////////////////////////////////////////////////////////

void buildTriangles(){
  triangleVertices.clear();
  triangleColors.clear();
  portal.clear();

  //floor
  vector<VEC3> vertices1;
  vertices1.push_back(VEC3(20.0, 0.0, -20.0));
  vertices1.push_back(VEC3(-20.0, 0.0, 20.0));
  vertices1.push_back(VEC3(20.0, 0.0, 20.0));
  triangleVertices.push_back(vertices1);
  portal.push_back(0);

  vector<VEC3> vertices2;
  vertices2.push_back(VEC3(20.0, 0.0, -20.0));
  vertices2.push_back(VEC3(-20.0, 0.0, -20.0));
  vertices2.push_back(VEC3(-20.0, 0.0, 20.0));
  triangleVertices.push_back(vertices2);
  portal.push_back(0);

  triangleColors.push_back(VEC3(0.371, 0.348, 0.383));
  triangleColors.push_back(VEC3(0.371, 0.354, 0.383));

  //left wall 
  vector<VEC3> vertices3;
  vertices3.push_back(VEC3(-4.0, 3.0, 5.0));
  vertices3.push_back(VEC3(5.0, 5.5, 5.0));
  vertices3.push_back(VEC3(5.0, 0.0, 5.0));
  triangleVertices.push_back(vertices3);
  portal.push_back(0);
  
  vector<VEC3> vertices4;
  vertices4.push_back(VEC3(5.0, 0.0, 5.0));
  vertices4.push_back(VEC3(-4.0, 0.0, 5.0));
  vertices4.push_back(VEC3(-4.0, 3.0, 5.0));
  triangleVertices.push_back(vertices4);
  portal.push_back(0);

  triangleColors.push_back(VEC3(0.588, 0.533, 0.552));
  triangleColors.push_back(VEC3(0.588, 0.533, 0.552));

  //right wall
  vector<VEC3> vertices5;
  vertices5.push_back(VEC3(5.0, 0.0, -5.0));
  vertices5.push_back(VEC3(5.0, 5.5, -5.0));
  vertices5.push_back(VEC3(-4.0, 3.0, -5.0));
  triangleVertices.push_back(vertices5);
  portal.push_back(0);
  
  vector<VEC3> vertices6;
  vertices6.push_back(VEC3(-4.0, 3.0, -5.0));
  vertices6.push_back(VEC3(-4.0, 0.0, -5.0));
  vertices6.push_back(VEC3(5.0, 0.0, -5.0));
  triangleVertices.push_back(vertices6);
  portal.push_back(0);

  triangleColors.push_back(VEC3(0.588, 0.533, 0.552));
  triangleColors.push_back(VEC3(0.588, 0.533, 0.552));


  // //back wall
  vector<VEC3> vertices7;
  vertices7.push_back(VEC3(5.0, 0.0, -5.0));
  vertices7.push_back(VEC3(5.0, 0.0, 5.0));
  vertices7.push_back(VEC3(5.0, 5.5, -5.0));
  triangleVertices.push_back(vertices7);
  portal.push_back(1);

  vector<VEC3> vertices8;
  vertices8.push_back(VEC3(5.0, 5.5, 5.0));
  vertices8.push_back(VEC3(5.0, 5.5, -5.0));
  vertices8.push_back(VEC3(5.0, 0.0, 5.0));
  triangleVertices.push_back(vertices8);
  portal.push_back(1);

  triangleColors.push_back(VEC3(0.949, 0.894, 0.737));
  triangleColors.push_back(VEC3(0.949, 0.894, 0.737));

  //left2
  VEC3 center = VEC3(8.0, 3.15, 1.875);
  Real radians = (-45 * M_PI) / 180;
  MATRIX3 rotation;
  rotation(0, 0) = cos(radians);
  rotation(0, 1) = 0.0;
  rotation(0, 2) = -sin(radians);
  rotation(1, 0) = 0.0;
  rotation(1, 1) = 1.0;
  rotation(1, 2) = 0.0;
  rotation(2, 0) = sin(radians);
  rotation(2, 1) = 0.0;
  rotation(2, 2) = cos(radians);

  vector<VEC3> vertices9;
  vertices9.push_back(VEC3(4.55, 6.3, 3.75));
  vertices9.push_back(VEC3(12.0, 6.3, 3.75));
  vertices9.push_back(VEC3(12.0, 0.0, 3.75));
  portal.push_back(0);
  for(int i = 0; i < vertices9.size(); i = i+1){
    vertices9[i] = vertices9[i] - center;
  }
  for(int i = 0; i < vertices9.size(); i=i+1){
    vertices9[i] = rotation * vertices9[i];
  }
  for(int i = 0; i < vertices9.size(); i=i+1){
    vertices9[i] = vertices9[i] + center;
  }
  triangleVertices.push_back(vertices9);

  vector<VEC3> vertices10;
  vertices10.push_back(VEC3(12.0, 0.0, 3.75));
  vertices10.push_back(VEC3(4.55, 0.0, 3.75));
  vertices10.push_back(VEC3(4.55, 6.3, 3.75));
  portal.push_back(0);
  for(int i = 0; i < vertices10.size(); i = i+1){
    vertices10[i] = vertices10[i] - center;
  }
  for(int i = 0; i < vertices10.size(); i=i+1){
    vertices10[i] = rotation * vertices10[i];
  }
  for(int i = 0; i < vertices10.size(); i=i+1){
    vertices10[i] = vertices10[i] + center;
  }
  triangleVertices.push_back(vertices10);

  triangleColors.push_back(VEC3(0.688, 0.633, 0.652));
  triangleColors.push_back(VEC3(0.688, 0.633, 0.652));

  // right 2
  center = VEC3(8.0, 3.0, -1.875);
  radians = (45 * M_PI) / 180;
  rotation(0, 0) = cos(radians);
  rotation(0, 2) = -sin(radians);
  rotation(2, 0) = sin(radians);
  rotation(2, 2) = cos(radians);

  vector<VEC3> vertices11;
  vertices11.push_back(VEC3(13.0, 0.0, -3.75));
  vertices11.push_back(VEC3(13.0, 6.5, -3.75));
  vertices11.push_back(VEC3(5.0, 6.0, -3.75));
  portal.push_back(2);
  for(int i = 0; i < vertices11.size(); i = i+1){
    vertices11[i] = vertices11[i] - center;
  }
  for(int i = 0; i < vertices11.size(); i=i+1){
    vertices11[i] = rotation * vertices11[i];
  }
  for(int i = 0; i < vertices11.size(); i=i+1){
    vertices11[i] = vertices11[i] + center;
  }
  triangleVertices.push_back(vertices11);

  vector<VEC3> vertices12;
  vertices12.push_back(VEC3(5.0, 6.0, -3.75));
  vertices12.push_back(VEC3(5.0, 0.0, -3.75));
  vertices12.push_back(VEC3(13.0, 0.0, -3.75));
  portal.push_back(2);
  for(int i = 0; i < vertices12.size(); i = i+1){
    vertices12[i] = vertices12[i] - center;
  }
  for(int i = 0; i < vertices12.size(); i=i+1){
    vertices12[i] = rotation * vertices12[i];
  }
  for(int i = 0; i < vertices12.size(); i=i+1){
    vertices12[i] = vertices12[i] + center;
  }
  triangleVertices.push_back(vertices12);

  triangleColors.push_back(VEC3(0.688, 0.633, 0.652));
  triangleColors.push_back(VEC3(0.688, 0.633, 0.652));


// level 3
  center = VEC3(16, 3.3, -0.9);
  radians = (-55 * M_PI) / 180;
  rotation(0, 0) = cos(radians);
  rotation(0, 2) = -sin(radians);
  rotation(2, 0) = sin(radians);
  rotation(2, 2) = cos(radians);
  
  vector<VEC3> vertices13;
  vertices13.push_back(VEC3(13.8, 6.8, -1.8));
  vertices13.push_back(VEC3(20, 6.8, -1.8));
  vertices13.push_back(VEC3(20, 0.0, -1.8));
  portal.push_back(3);
  for(int i = 0; i < vertices13.size(); i = i+1){
    vertices13[i] = vertices13[i] - center;
  }
  for(int i = 0; i < vertices13.size(); i=i+1){
    vertices13[i] = rotation * vertices13[i];
  }
  for(int i = 0; i < vertices13.size(); i=i+1){
    vertices13[i] = vertices13[i] + center;
  }
  triangleVertices.push_back(vertices13);

  vector<VEC3> vertices14;
  vertices14.push_back(VEC3(20, 0.0, -1.8));
  vertices14.push_back(VEC3(13.8, 0.0, -1.8));
  vertices14.push_back(VEC3(13.8, 6.8, -1.8));
  portal.push_back(3);
  for(int i = 0; i < vertices14.size(); i = i+1){
    vertices14[i] = vertices14[i] - center;
  }
  for(int i = 0; i < vertices14.size(); i=i+1){
    vertices14[i] = rotation * vertices14[i];
  }
  for(int i = 0; i < vertices14.size(); i=i+1){
    vertices14[i] = vertices14[i] + center;
  }
  triangleVertices.push_back(vertices14);

  triangleColors.push_back(VEC3(0.949, 0.894, 0.737));
  triangleColors.push_back(VEC3(0.949, 0.894, 0.737));


  center = VEC3(17, 3.0, -3);
  radians = (55 * M_PI) / 180;
  rotation(0, 0) = cos(radians);
  rotation(0, 2) = -sin(radians);
  rotation(2, 0) = sin(radians);
  rotation(2, 2) = cos(radians);

  vector<VEC3> vertices15;
  vertices15.push_back(VEC3(16.6, 0.0, -6));
  vertices15.push_back(VEC3(16.6, 7.2, -6));
  vertices15.push_back(VEC3(13.0, 7.2, -6));
  portal.push_back(0);
  for(int i = 0; i < vertices15.size(); i = i+1){
    vertices15[i] = vertices15[i] - center;
  }
  for(int i = 0; i < vertices15.size(); i=i+1){
    vertices15[i] = rotation * vertices15[i];
  }
  for(int i = 0; i < vertices15.size(); i=i+1){
    vertices15[i] = vertices15[i] + center;
  }
  triangleVertices.push_back(vertices15);

  vector<VEC3> vertices16;
  vertices16.push_back(VEC3(16.6, 7.2, -6));
  vertices16.push_back(VEC3(16.6, 0.0, -6));
  vertices16.push_back(VEC3(16.6, 0.0, -6));
  portal.push_back(0);
  for(int i = 0; i < vertices16.size(); i = i+1){
    vertices16[i] = vertices16[i] - center;
  }
  for(int i = 0; i < vertices16.size(); i=i+1){
    vertices16[i] = rotation * vertices16[i];
  }
  for(int i = 0; i < vertices16.size(); i=i+1){
    vertices16[i] = vertices16[i] + center;
  }
  triangleVertices.push_back(vertices16);

  triangleColors.push_back(VEC3(0.949, 0.894, 0.737));
  triangleColors.push_back(VEC3(0.949, 0.894, 0.737));


  //level 4
  center = VEC3(16, 3.3, -0.9);
  radians = (-55 * M_PI) / 180;
  rotation(0, 0) = cos(radians);
  rotation(0, 2) = -sin(radians);
  rotation(2, 0) = sin(radians);
  rotation(2, 2) = cos(radians);
  
  vector<VEC3> vertices17;
  vertices17.push_back(VEC3(13.8, 6.8, -1.8));
  vertices17.push_back(VEC3(20, 6.8, -1.8));
  vertices17.push_back(VEC3(20, 0.0, -1.8));
  portal.push_back(0);
  for(int i = 0; i < vertices17.size(); i = i+1){
    vertices17[i] = vertices17[i] - center;
  }
  for(int i = 0; i < vertices17.size(); i=i+1){
    vertices17[i] = rotation * vertices17[i];
  }
  for(int i = 0; i < vertices17.size(); i=i+1){
    vertices17[i] = vertices17[i] + center;
  }
  triangleVertices.push_back(vertices17);

  vector<VEC3> vertices18;
  vertices18.push_back(VEC3(20, 0.0, -1.8));
  vertices18.push_back(VEC3(13.8, 0.0, -1.8));
  vertices18.push_back(VEC3(13.8, 6.8, -1.8));
  portal.push_back(0);
  for(int i = 0; i < vertices18.size(); i = i+1){
    vertices18[i] = vertices18[i] - center;
  }
  for(int i = 0; i < vertices18.size(); i=i+1){
    vertices18[i] = rotation * vertices18[i];
  }
  for(int i = 0; i < vertices18.size(); i=i+1){
    vertices18[i] = vertices18[i] + center;
  }
  triangleVertices.push_back(vertices18);

  triangleColors.push_back(VEC3(0.949, 0.949, 0.949));
  triangleColors.push_back(VEC3(0.949, 0.949, 0.949));


  // center = VEC3(21, 4, -3);
  // radians = (55 * M_PI) / 180;
  // rotation(0, 0) = cos(radians);
  // rotation(0, 2) = -sin(radians);
  // rotation(2, 0) = sin(radians);

  // vector<VEC3> vertices19;
  // vertices19.push_back(VEC3(23, 0.0, -6));
  // vertices19.push_back(VEC3(23, 8, -6));
  // vertices19.push_back(VEC3(19, 8, -6));
  // portal.push_back(0);
  // for(int i = 0; i < vertices19.size(); i = i+1){
  //   vertices19[i] = vertices19[i] - center;
  // }
  // for(int i = 0; i < vertices19.size(); i=i+1){
  //   vertices19[i] = rotation * vertices19[i];
  // }
  // for(int i = 0; i < vertices19.size(); i=i+1){
  //   vertices19[i] = vertices19[i] + center;
  // }
  // triangleVertices.push_back(vertices19);

  // vector<VEC3> vertices20;
  // vertices20.push_back(VEC3(19, 8, -6));
  // vertices20.push_back(VEC3(19, 0.0, -6));
  // vertices20.push_back(VEC3(23, 0.0, -6));
  // portal.push_back(0);
  // for(int i = 0; i < vertices20.size(); i = i+1){
  //   vertices20[i] = vertices20[i] - center;
  // }
  // for(int i = 0; i < vertices20.size(); i=i+1){
  //   vertices20[i] = rotation * vertices20[i];
  // }
  // for(int i = 0; i < vertices20.size(); i=i+1){
  //   vertices20[i] = vertices20[i] + center;
  // }
  // triangleVertices.push_back(vertices20);

  // triangleColors.push_back(VEC3(0.929, 0.929, 0.929));
  // triangleColors.push_back(VEC3(0.929, 0.929, 0.929));

}

void buildScene()
{
  sphereCenters.clear();
  sphereRadii.clear();
  sphereColors.clear();

  sphereCenters.push_back(VEC3(2, 0, -3.0));
  sphereRadii.push_back(1.0);
  sphereColors.push_back(VEC3(1, 1, 1));

  sphereCenters.push_back(VEC3(2, 2, -3.0));
  sphereRadii.push_back(1.0);
  sphereColors.push_back(VEC3(1, 1, 1));

  sphereCenters.push_back(VEC3(4.5, 2.6, 0.0));
  sphereRadii.push_back(2.3);
  sphereColors.push_back(VEC3(1, 0, 0));

  sphereCenters.push_back(VEC3(8.0, 2.6, -2.2));
  sphereRadii.push_back(2.2);
  sphereColors.push_back(VEC3(0, 1, 0));

  sphereCenters.push_back(VEC3(15, 3.3, -1.5));
  sphereRadii.push_back(2.2);
  sphereColors.push_back(VEC3(0, 1, 0));
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  string skeletonFilename("91.asf");
  string motionFilename("91_01.amc");
  
  // load up skeleton stuff
  skeleton = new Skeleton(skeletonFilename.c_str(), MOCAP_SCALE);
  skeleton->setBasePosture();
  displayer.LoadSkeleton(skeleton);

  // load up the motion
  motion = new Motion(motionFilename.c_str(), MOCAP_SCALE, skeleton);
  displayer.LoadMotion(motion);
  skeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));

  // Note we're going 8 frames at a time, otherwise the animation
  // is really slow.
  for (int x = 0; x < 2400; x += 8)
  {
    setSkeletonsToSpecifiedFrame(x);
    buildScene();
    buildCylinders();
    buildTriangles();

    char buffer[256];
    sprintf(buffer, "./frames/frame.%04i.ppm", x / 8);
    renderImage(windowWidth, windowHeight, buffer);
    cout << "Rendered " + to_string(x / 8) + " frames" << endl;
    break;
  }

  return 0;
}
