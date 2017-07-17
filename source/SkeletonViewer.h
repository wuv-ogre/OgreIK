#ifndef _Demo_SkeletonViewer_H_
#define _Demo_SkeletonViewer_H_

#include "OgrePrerequisites.h"

namespace Demo
{
    class SkeletonViewer
	{
    public:
        Ogre::MeshPtr mMesh;

        SkeletonViewer( Ogre::SkeletonInstance* skeleton, const Ogre::Real boneWidthScale = 0.5 );
    };
}
#endif
