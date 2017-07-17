
#ifndef _Demo_IKGameState_H_
#define _Demo_IKGameState_H_

#include "OgrePrerequisites.h"
#include "TutorialGameState.h"

namespace Ogre
{
    class IK;
}

namespace Demo
{
    class IKGameState : public TutorialGameState
    {
        Ogre::SceneNode     *mSceneNode;

        Ogre::SceneNode     *mLightNodes[3];

        bool                mAnimateObjects;
        
        Ogre::IK            *mIK;

        virtual void generateDebugText( float timeSinceLast, Ogre::String &outText );

        void createCerberus();
        
    public:
        IKGameState( const Ogre::String &helpDescription );

        virtual void createScene01(void);

        virtual void update( float timeSinceLast );

        virtual void keyReleased( const SDL_KeyboardEvent &arg );
    };
}

#endif
