#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <float.h>
#include "SETTINGS.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "motion.h"

using namespace std;

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

vector<VEC3> cylinderColors;
vector<float> cylinderRadii;
vector<MATRIX4> cylinderRotation;
vector<MATRIX4> cylinderScaling;
vector<VEC4> cylinderTranslation;
vector<VEC4> cylinderRightVertex;
vector<VEC4> cylinderLeftVertex;
vector<float> cylinderLengths;

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

bool rayTriangleIntersection(const VEC3& dVal, const VEC3& eye, vector<VEC3> vertices, float& t){
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

bool raySphereIntersect(const VEC3& center, 
                        const float radius, 
                        const VEC3& rayPos, 
                        const VEC3& rayDir,
                        float& t) {
  const VEC3 op = center - rayPos;
  const float eps = 1e-8;
  const float b = op.dot(rayDir);
  float det = b * b - op.dot(op) + radius * radius;

  // determinant check
  if (det < 0) 
    return false; 
  
  det = sqrt(det);
  t = b - det;
  if (t <= eps)
  {
    t = b + det;
    if (t <= eps)
      t = -1;
  }

  if (t < 0) return false;
  return true;
}


bool rayCylinderIntersection(const VEC3& rayDir, 
                              const VEC3& rayPos,  
                              float& radius, 
                              MATRIX4& rotation,
                              MATRIX4& scaling,
                              VEC4& translation,
                              VEC4& cylinderRightVertex,
                              VEC4& cylinderLeftVertex,
                              float& length,
                              float& t
                            ){
  
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

  VEC3 left = VEC3(cylinderRightVertex[0], cylinderRightVertex[1], cylinderRightVertex[2]);
  VEC3 right = VEC3(cylinderLeftVertex[0], cylinderLeftVertex[1], cylinderLeftVertex[2]);

  //cout << intersection[2] << length << endl;
  if(intersection[2] > length || intersection[2] < 0){
    return false;
  }

  return true;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void rayColor(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor) 
{
  pixelColor = VEC3(1,1,1);
  
  int hitID = -1;
  float tMinCylinder = FLT_MAX;
  float tMinSphere = FLT_MAX;
  float tMinTriangle = FLT_MAX;

  // VEC3 tMinArray = VEC3(FLT_MAX, FLT_MAX, FLT_MAX);
  // VEC3 index = VEC3(-1, -1, -1);
  // vector<VEC3> indexColor;
  // indexColor.push_back(VEC3(-1, -1, -1));
  // indexColor.push_back(VEC3(-1, -1, -1));
  // indexColor.push_back(VEC3(-1, -1, -1));
  for(int y = 0; y < cylinderRadii.size(); y++) 
  {
    float tMin = FLT_MAX;
    if(rayCylinderIntersection(rayDir, 
                              rayPos, 
                              cylinderRadii[y], 
                              cylinderRotation[y], 
                              cylinderScaling[y],
                              cylinderTranslation[y],
                              cylinderRightVertex[y],
                              cylinderLeftVertex[y],
                              cylinderLengths[y],
                              tMin))
    {
      if (tMin < tMinCylinder && tMin > 0.0) 
      {
        tMinCylinder = tMin;
        hitID = y;
      }
    }
  }

  for(int y = 0; y < sphereCenters.size(); y++) 
  {
    float tMin = FLT_MAX;
    if(raySphereIntersect(sphereCenters[y], sphereRadii[y], rayPos, rayDir, tMin)) 
    {
      if(tMin < tMinSphere && tMin > 0.0) 
      {
        tMinSphere = tMin;
        hitID = y;
      }
    }
  }
  
  for(int y = 0; y < triangleColors.size(); y++) 
  {
    float tMin = FLT_MAX;
    if(rayTriangleIntersection(rayDir, rayPos, triangleVertices[y], tMin))
    {
      if(tMin < tMinTriangle && tMin > 0.0) 
      {
        tMinTriangle = tMin;
        hitID = y;
      }
    }
  }

  if(hitID == -1) {
    return;
  }

  if(tMinCylinder < tMinTriangle && tMinCylinder < tMinSphere) 
  {
    pixelColor = cylinderColors[hitID];
  }
  else if (tMinTriangle < tMinSphere && tMinTriangle < tMinCylinder) 
  {
    pixelColor = triangleColors[hitID];
    //pixelColor = VEC3(0, 1, 1);
  }
  else if (tMinSphere < tMinTriangle && tMinSphere < tMinCylinder) 
  {
    pixelColor = sphereColors[hitID];
    //pixelColor = VEC3(0, 1, 0);
  }
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

    // get the direction vector
    // VEC3 direction = (rightVertex - leftVertex).head<3>();
    // const float magnitude = direction.norm();
    // direction *= 1.0 / magnitude;

    // how many spheres?
    // const float sphereRadius = 0.05;
    // const int totalSpheres = magnitude / (2.0 * sphereRadius);
    // const float rayIncrement = magnitude / (float)totalSpheres;

    //store the spheres
    cylinderRadii.push_back(0.35);
    cylinderColors.push_back(VEC3(1,0,0));
    
    // cylinderRadii.push_back(0.05);
    // cylinderColors.push_back(VEC3(1,0,0));
    // for (int y = 0; y < totalSpheres; y++)
    // {
    //   VEC3 center = ((float)y + 0.5) * rayIncrement * direction + leftVertex.head<3>();
    //   cylinderRadii.push_back(0.05);
    //   cylinderColors.push_back(VEC3(1, 0, 0));
    // }
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

      // if(x == 323 && y == 256) {
      //   continue;
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

void transformVertices(){
  triangleVertices.clear();
  triangleColors.clear();

  
  vector<VEC3> vertices1;
  vertices1.push_back(VEC3(5.0, 0.0, -5.0));
  vertices1.push_back(VEC3(-5.0, 0.0, 5.0));
  vertices1.push_back(VEC3(5.0, 0.0, 5.0));
  triangleVertices.push_back(vertices1);

  vector<VEC3> vertices2;
  vertices2.push_back(VEC3(5.0, 0.0, -5.0));
  vertices2.push_back(VEC3(-5.0, 0.0, -5.0));
  vertices2.push_back(VEC3(-5.0, 0.0, 5.0));
  triangleVertices.push_back(vertices2);

  vector<VEC3> vertices3;
  vertices3.push_back(VEC3(5.0, 0.0, 5.0));
  vertices3.push_back(VEC3(5.0, 6.0, 5.0));
  vertices3.push_back(VEC3(-4.0, 3.0, 5.0));
  triangleVertices.push_back(vertices3);
  
  vector<VEC3> vertices4;
  vertices4.push_back(VEC3(5.0, 0.0, 5.0));
  vertices4.push_back(VEC3(-4.0, 0.0, 5.0));
  vertices4.push_back(VEC3(-4.0, 3.0, 5.0));
  triangleVertices.push_back(vertices4);

  vector<VEC3> vertices5;
  vertices5.push_back(VEC3(5.0, 0.0, -5.0));
  vertices5.push_back(VEC3(5.0, 6.0, -5.0));
  vertices5.push_back(VEC3(-4.0, 3.0, -5.0));
  triangleVertices.push_back(vertices5);
  
  vector<VEC3> vertices6;
  vertices6.push_back(VEC3(5.0, 0.0, -5.0));
  vertices6.push_back(VEC3(-4.0, 0.0, -5.0));
  vertices6.push_back(VEC3(-4.0, 3.0, -5.0));
  triangleVertices.push_back(vertices6);

  // vector<VEC3> vertices7;
  // vertices7.push_back(VEC3(5.0, 0.0, -5.0));
  // vertices7.push_back(VEC3(5.0, 0.0, 5.0));
  // vertices7.push_back(VEC3(5.0, 6.0, -5.0));
  // triangleVertices.push_back(vertices7);

  // vector<VEC3> vertices8;
  // vertices8.push_back(VEC3(5.0, 6.0, 5.0));
  // vertices8.push_back(VEC3(5.0, 6.0, -5.0));
  // vertices8.push_back(VEC3(5.0, 0.0, 5.0));
  // triangleVertices.push_back(vertices8);

  // vector<VEC3> vertices3;
  // vertices3.push_back(VEC3(5.0, 0.0, -5.0));
  // vertices3.push_back(VEC3(-5.0, 0.0, 0.0));
  // vertices3.push_back(VEC3(5.0, 0.0, 0.0));
  // triangleVertices.push_back(vertices3);

  // vector<VEC3> vertices4;
  // vertices4.push_back(VEC3(5.0, 0.0, -5.0));
  // vertices4.push_back(VEC3(5.0, 0.0, -10.0));
  // vertices4.push_back(VEC3(-5.0, 0.0, -5.0));
  // triangleVertices.push_back(vertices4);
  // vector<VEC3> vertices5;
  // vertices5.push_back(VEC3(5.0, 0.0, -10.0));
  // vertices5.push_back(VEC3(-5.0, 0.0, -10.0));
  // vertices5.push_back(VEC3(-5.0, 0.0, -5.0));
  // triangleVertices.push_back(vertices5);

  // vector<VEC3> vertices4;
  // vertices4.push_back(VEC3(5.0, 0.0, -4.0));
  // vertices4.push_back(VEC3(-4.0, 0.0, 0.0));
  // vertices4.push_back(VEC3(5.0, 0.0, 0.0));
  // triangleVertices.push_back(vertices4);
  //triangleVertices.push_back(vertices);
  triangleColors.push_back(VEC3(0, 1, 1));
  triangleColors.push_back(VEC3(0, 0, 1));
  triangleColors.push_back(VEC3(0, 1, 1));
  triangleColors.push_back(VEC3(0, 0, 1));
  // triangleColors.push_back(VEC3(0, 1, 1));
  // triangleColors.push_back(VEC3(0, 1, 1));
  // triangleColors.push_back(VEC3(0, 1, 1));
  // triangleColors.push_back(VEC3(0, 1, 1));

  //triangleColors.push_back(VEC3(0, 0, 1));
  //triangleColors.push_back(VEC3(0, 0, 1));
  //triangleColors.push_back(VEC3(0, 0, 1));
  // triangleColors.push_back(VEC3(0, 0, 1));
}

void buildScene()
{
  sphereCenters.clear();
  sphereRadii.clear();
  sphereColors.clear();

  sphereCenters.push_back(VEC3(2, 0, -3.0));
  sphereRadii.push_back(1.0);
  sphereColors.push_back(VEC3(0, 1, 0));

  sphereCenters.push_back(VEC3(2, 2, -3.0));
  sphereRadii.push_back(1.0);
  sphereColors.push_back(VEC3(0, 1, 0));

  // sphereCenters.push_back(VEC3(-3.0, 0.0, 3.0));
  // sphereRadii.push_back(1.0);
  // sphereColors.push_back(VEC3(0, 1, 0));

  // sphereCenters.push_back(VEC3(-3.0, 1.9, 3.0));
  // sphereRadii.push_back(1.0);
  // sphereColors.push_back(VEC3(0, 1, 0));

  sphereCenters.push_back(VEC3(4.5, 3.2, 0.0));
  sphereRadii.push_back(2.8);
  sphereColors.push_back(VEC3(0, 1, 1));
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
    transformVertices();

    char buffer[256];
    sprintf(buffer, "./frames/frame.%04i.ppm", x / 8);
    renderImage(windowWidth, windowHeight, buffer);
    cout << "Rendered " + to_string(x / 8) + " frames" << endl;
  }

  return 0;
}
