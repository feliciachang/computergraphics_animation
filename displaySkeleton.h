/*
display.h

Display the skeleton, ground plane and other objects.			

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/

#ifndef _DISPLAY_SKELETON_H_
#define _DISPLAY_SKELETON_H_

#include "SETTINGS.h"
#include "skeleton.h"
#include "motion.h"
#include <vector>
#include <map>

using namespace std;

class DisplaySkeleton 
{

//member functions
public: 
  enum RenderMode
  {
    BONES_ONLY, BONES_AND_LOCAL_FRAMES
  };
  enum JointColor
  {
    GREEN, RED, BLUE, NUMBER_JOINT_COLORS
  };

  DisplaySkeleton();
  ~DisplaySkeleton();

  //set skeleton for display
  void LoadSkeleton(Skeleton * pSkeleton);
  //set motion for display
  void LoadMotion(Motion * pMotion);

  //display the scene (skeleton, ground plane ....)
  void ComputeBonePositions(RenderMode renderMode);

  void SetDisplayedSpotJoint(int jointID) {m_SpotJoint = jointID;}
  int GetDisplayedSpotJoint(void) {return m_SpotJoint;}
  int GetNumSkeletons(void) {return numSkeletons;}
  Skeleton * GetSkeleton(int skeletonIndex);
  Motion * GetSkeletonMotion(int skeletonIndex);

  void Reset(void);

  vector<MATRIX4>& rotations()  { return boneRotations; };
  vector<MATRIX4>& scalings()   { return boneScalings; };
  vector<VEC4>& translations() { return boneTranslations; };
  vector<float>& lengths()      { return boneLengths; };

protected:
  RenderMode renderMode;
  // Draw a particular bone
  void DrawBone(Bone *ptr, int skelNum);
  // Draw the skeleton hierarchy
  void Traverse(Bone *ptr, int skelNum);
  
  int m_SpotJoint;		//joint whose local coordinate framework is drawn
  int numSkeletons;
  Skeleton *m_pSkeleton[MAX_SKELS];		//pointer to current skeleton
  Motion *m_pMotion[MAX_SKELS];		//pointer to current motion	

  static float jointColors[NUMBER_JOINT_COLORS][3];

  vector<MATRIX4> boneRotations;
  vector<MATRIX4> boneScalings;
  vector<VEC4> boneTranslations;
  vector<float> boneLengths;
};

#endif
