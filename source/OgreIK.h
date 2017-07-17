/*
 -----------------------------------------------------------------------------
 This source file is part of OGRE
 (Object-oriented Graphics Rendering Engine)
 For the latest info, see http://www.ogre3d.org
 
 Copyright (c) 2000-2014 Torus Knot Software Ltd
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 -----------------------------------------------------------------------------
 */

#ifndef _OgreIK_H_
#define _OgreIK_H_

#include "OgrePrerequisites.h"

namespace Ogre
{
    
    /**
     * To get support for joint limits uncomment the if guard in BussIK::Node
     * and consider running multiple updates per frame.
    */
    class IK
    {
    private:
        struct PrivateContext;  // uses BussIK
        PrivateContext *m_context;

    public:
        /**
         * @param modifyExistingOrientations
         * If true we have additional rotations that come from animations like ankle twist.
         * If false we are doing something very specific e.g. demo spinner.
         */
        IK( bool modifyExistingOrientations = true );
        ~IK();
        
        /**
         * @note Can only have one root per instance
        */
        void insertRoot(Ogre::Bone* root,
                        const Ogre::Vector3& axis = Ogre::Vector3::UNIT_X,
                        const float minTheta = -Ogre::Math::PI,
                        const float maxTheta = Ogre::Math::PI,
                        const float restTheta = 0.0);
        
        /**
         * @note The bones parent bone must have been added.
        */
        void insertChild(Ogre::Bone* bone,
                         const Ogre::Vector3& axis = Ogre::Vector3::UNIT_X,
                         const float minTheta = -Ogre::Math::PI,
                         const float maxTheta = Ogre::Math::PI,
                         const float restTheta = 0.0);
                
        /**
         * @param hook The hook is at the end of a chain, hook 'tries' to reach effector position.
        */
        void insertEffector(Ogre::Bone* effector, Ogre::Bone* hook);

        /**
         * Call once root children and effector/effectors have all been added.
        */
        void buildTree();

        
        const std::vector<Ogre::Bone*>& getJoints() const;
        const std::vector<Ogre::Bone*>& getEffectors() const;
        
        /*
         * Reorientates so root negative y axis points at the effector do this prior to calling update.
        */
        void reorientateRoot( Ogre::Bone* effector );
        
        /**
         * Reorientates all bones to satify the IK constraints.
        */
        void update();
    };
}

#endif
