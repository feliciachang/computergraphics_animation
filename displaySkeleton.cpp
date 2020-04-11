/*
Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include "types.h"

#include "skeleton.h"
#include "motion.h"
#include "displaySkeleton.h"

////////////////SOFTWARE GL BEGIN///////////////////////
#include <stack>
stack<MATRIX4> matrixStack;
static MATRIX4 currentMatrix = MATRIX4::Identity();

static MATRIX4 toMatrix4(const MATRIX3& A)
{
  MATRIX4 result = MATRIX4::Identity();
  for (int y = 0; y < 3; y++)
    for (int x = 0; x < 3; x++)
      result(x,y) = A(x,y);
  return result;
}

static void myPushMatrix()
{
  matrixStack.push(currentMatrix);  
}
static void myPopMatrix()
{
  assert(matrixStack.size() != 0);
  currentMatrix = matrixStack.top();
  matrixStack.pop();
}

static void myTranslatef(const float x, const float y, const float z)
{
  MATRIX4 translation = MATRIX4::Identity();
  translation(3,0) = x;
  translation(3,1) = y;
  translation(3,2) = z;

  currentMatrix = translation * currentMatrix;
}
static void myRotatef(const float degrees, const float x, const float y, const float z)
{
  VEC3 axis(x,y,z);
  axis.normalize();
  MATRIX3 rotation;

  float radians = (degrees / 360.0) * 2.0 * M_PI;
  rotation = AngleAxisd(radians, axis);
  rotation.transposeInPlace();
  
  currentMatrix = toMatrix4(rotation) * currentMatrix;
}
static void myMultMatrixd(const double* matrix)
{
  MATRIX4 A;
  int i = 0;
  for (int x = 0; x < 4; x++)
    for (int y = 0; y < 4; y++, i++)
      A(x,y) = matrix[i];

  currentMatrix = A * currentMatrix;
}
static void myLoadIdentity()
{
  currentMatrix = MATRIX4::Identity();
}
////////////////SOFTWARE GL END///////////////////////

float DisplaySkeleton::jointColors[NUMBER_JOINT_COLORS][3] =
{
  {0.0f, 1.0f, 0.0f},  // GREEN
  {1.0f, 0.0f, 0.0f},  // RED
  {0.0f, 0.0f, 1.0f}   // BLUE
};

DisplaySkeleton::DisplaySkeleton(void)
{
  m_SpotJoint = -1;
  numSkeletons = 0;
  for(int skeletonIndex = 0; skeletonIndex < MAX_SKELS; skeletonIndex++)
  {
    m_pSkeleton[skeletonIndex] = NULL;
    m_pMotion[skeletonIndex] = NULL;
  }
}

DisplaySkeleton::~DisplaySkeleton(void)
{
  Reset();
}

/*
  Define M_k = Modelview matrix at the kth node (bone) in the heirarchy
  M_k stores the transformation matrix of the kth bone in world coordinates
  Our goal is to draw the (k+1)th bone, using its local information and M_k

  In the k+1th node, compute the following matrices:
  rot_parent_current: this is the rotation matrix that 
  takes us from k+1 to the kth local coordinate system 
  R_k+1 : Rotation matrix for the k+1 th node (bone)
  using angles specified by the AMC file in local coordinates
  T_k+1 : Translation matrix for the k+1th node
  
  The update relation is given by:
  M_k+1 = M_k * (rot_parent_current) * R_k+1 + T_k+1
*/
// TODO: Replace GL calls here with Eigen matrices
void DisplaySkeleton::DrawBone(Bone *pBone,int skelNum)
{
  static double z_dir[3] = {0.0, 0.0, 1.0};
  double r_axis[3], theta;

  //currentMatrix = getCurrentModelview();

  //Transform (rotate) from the local coordinate system of this bone to it's parent
  //This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
  myMultMatrixd((double*)&pBone->rot_parent_current);     

  //translate AMC (rarely used)
  if(pBone->doftz)
    myTranslatef(0.0f, 0.0f, float(pBone->tz));
  if(pBone->dofty) 
    myTranslatef(0.0f, float(pBone->ty), 0.0f);
  if(pBone->doftx)
    myTranslatef(float(pBone->tx), 0.0f, 0.0f);

  //rotate AMC 
  if(pBone->dofrz)
    myRotatef(float(pBone->rz), 0.0f, 0.0f, 1.0f);
  if(pBone->dofry) 
    myRotatef(float(pBone->ry), 0.0f, 1.0f, 0.0f);
  if(pBone->dofrx) 
    myRotatef(float(pBone->rx), 1.0f, 0.0f, 0.0f);

  //Store the current ModelviewMatrix (before adding the translation part)
  myPushMatrix();

  //Compute tx, ty, tz : translation from pBone to its child (in local coordinate system of pBone)
  double tx = pBone->dir[0] * pBone->length;
  double ty = pBone->dir[1] * pBone->length;
  double tz = pBone->dir[2] * pBone->length;

  // Use the current ModelviewMatrix to display the current bone
  // Rotate the bone from its canonical position (elongated sphere 
  // with its major axis parallel to X axis) to its correct orientation
  if(pBone->idx == Skeleton::getRootIndex())
  {
    // do nothing
  }
  else
  { 
    //MATRIX4 currentTransform = getCurrentModelview();
    MATRIX4 currentTransform = currentMatrix;

    //Compute the angle between the canonical pose and the correct orientation 
    //(specified in pBone->dir) using cross product.
    //Using the formula: r_axis = z_dir x pBone->dir
    r_axis[0] = z_dir[1]*pBone->dir[2]-z_dir[2]*pBone->dir[1];
    r_axis[1] = z_dir[2]*pBone->dir[0]-z_dir[0]*pBone->dir[2];
    r_axis[2] = z_dir[0]*pBone->dir[1]-z_dir[1]*pBone->dir[0];

    double dot_prod = z_dir[0] * pBone->dir[0] + z_dir[1] * pBone->dir[1] + z_dir[2] * pBone->dir[2] ;
    double r_axis_len = sqrt(r_axis[0] * r_axis[0] + r_axis[1] * r_axis[1] + r_axis[2] * r_axis[2]);
    theta = atan2(r_axis_len, dot_prod);
    VEC3 axis(r_axis[0], r_axis[1], r_axis[2]);
    axis.normalize();
    MATRIX3 rotation;
    rotation = AngleAxisd(theta, axis);
    rotation.transposeInPlace();

    MATRIX4 scaling = MATRIX4::Identity();
    scaling(0,0) = pBone->aspx;
    scaling(1,1) = pBone->aspy;
    boneScalings[pBone->idx] = scaling;
    
    currentTransform = scaling * toMatrix4(rotation) * currentTransform;

    VEC4 currentTranslation;
    currentTranslation[0] = currentTransform(3,0);
    currentTranslation[1] = currentTransform(3,1);
    currentTranslation[2] = currentTransform(3,2);
    currentTransform(3,0) = 0;
    currentTransform(3,1) = 0;
    currentTransform(3,2) = 0;

    boneRotations[pBone->idx] = currentTransform.transpose();
    boneTranslations[pBone->idx] = currentTranslation;
  }

  myPopMatrix();

  // Finally, translate the bone, depending on its length and direction
  // This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
  myTranslatef(float(tx), float(ty), float(tz));
}

//Traverse the hierarchy starting from the root 
//Every node in the data structure has just one child pointer. 
//If there are more than one children for any node, they are stored as sibling pointers
//The algorithm draws the current node (bone), visits its child and then visits siblings
// TODO: Replace GL calls here with Eigen matrices
void DisplaySkeleton::Traverse(Bone *ptr,int skelNum)
{
  if(ptr != NULL)
  {
    myPushMatrix();

    DrawBone(ptr,skelNum);
    Traverse(ptr->child,skelNum);

    myPopMatrix();

    Traverse(ptr->sibling,skelNum);
  }
}

//Draw the skeleton
// TODO: Replace GL calls here with Eigen matrices
void DisplaySkeleton::ComputeBonePositions(RenderMode renderMode_)
{
  unsigned int numbones = m_pSkeleton[0]->numBonesInSkel(*m_pSkeleton[0]->getRoot());
  if (boneRotations.size() != numbones)
  {
    boneRotations.resize(numbones); 
    boneTranslations.resize(numbones); 
    boneScalings.resize(numbones); 
    boneLengths.resize(numbones); 
    for (unsigned int x = 0; x < numbones; x++)
      boneLengths[x] = m_pSkeleton[0]->getBone(x).length;
  }

  // Set render mode
  renderMode = renderMode_;

  myPushMatrix();

  // load up identity so we just get the pure transforms out at the end,
  // not the modelview as well
  myLoadIdentity();

  //draw the skeleton starting from the root
  for (int i = 0; i < numSkeletons; i++)
  {
    myPushMatrix();

    double translation[3];
    m_pSkeleton[i]->GetTranslation(translation);
    double rotationAngle[3];
    m_pSkeleton[i]->GetRotationAngle(rotationAngle);

    myTranslatef(float(MOCAP_SCALE * translation[0]), float(MOCAP_SCALE * translation[1]), float(MOCAP_SCALE * translation[2]));
    myRotatef(float(rotationAngle[0]), 1.0f, 0.0f, 0.0f);
    myRotatef(float(rotationAngle[1]), 0.0f, 1.0f, 0.0f);
    myRotatef(float(rotationAngle[2]), 0.0f, 0.0f, 1.0f);

    Traverse(m_pSkeleton[i]->getRoot(),i);

    myPopMatrix();
  }
  myPopMatrix();
}

void DisplaySkeleton::LoadMotion(Motion * pMotion)
{
  // always load the motion for the latest skeleton
  if(m_pMotion[numSkeletons - 1] != NULL) 
    delete m_pMotion[numSkeletons - 1];
  m_pMotion[numSkeletons - 1] = pMotion;
}

//Set skeleton for display
void DisplaySkeleton::LoadSkeleton(Skeleton *pSkeleton)
{
  if (numSkeletons >= MAX_SKELS) 
    return;

  m_pSkeleton[numSkeletons] = pSkeleton;

  //Create the display list for the skeleton
  //All the bones are the elongated spheres centered at (0,0,0).
  //The axis of elongation is the X axis.
  //SetDisplayList(numSkeletons, m_pSkeleton[numSkeletons]->getRoot(), &m_BoneList[numSkeletons]);
  numSkeletons++;
}

Motion * DisplaySkeleton::GetSkeletonMotion(int skeletonIndex)
{
  if (skeletonIndex < 0 || skeletonIndex >= MAX_SKELS)
  {
    printf("Error in DisplaySkeleton::GetSkeletonMotion: index %d is illegal.\n", skeletonIndex);
    exit(0);
  }
  return m_pMotion[skeletonIndex];
}

Skeleton * DisplaySkeleton::GetSkeleton(int skeletonIndex)
{
  if (skeletonIndex < 0 || skeletonIndex >= numSkeletons)
  {
    printf("Error in DisplaySkeleton::GetSkeleton: skeleton index %d is illegal.\n", skeletonIndex);
    exit(0);
  }
  return m_pSkeleton[skeletonIndex];
}

void DisplaySkeleton::Reset(void)
{
  for(int skeletonIndex = 0; skeletonIndex < MAX_SKELS; skeletonIndex++)
  {
    if (m_pSkeleton[skeletonIndex] != NULL)
    {
      delete (m_pSkeleton[skeletonIndex]);
      //glDeleteLists(m_BoneList[skeletonIndex], 1);
      m_pSkeleton[skeletonIndex] = NULL;
    }
    if (m_pMotion[skeletonIndex] != NULL)
    {
      delete (m_pMotion[skeletonIndex]);
      m_pMotion[skeletonIndex] = NULL;
    }
  }
  numSkeletons = 0;
}
