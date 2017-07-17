
#include "IKGameState.h"
#include "CameraController.h"
#include "GraphicsSystem.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"

#include "OgreMeshManager.h"
#include "OgreMeshManager2.h"
#include "OgreMesh2.h"

#include "OgreCamera.h"
#include "OgreRenderWindow.h"

#include "OgreHlmsPbsDatablock.h"
#include "OgreHlmsSamplerblock.h"

#include "OgreRoot.h"
#include "OgreHlmsManager.h"
#include "OgreHlmsTextureManager.h"
#include "OgreHlmsPbs.h"

#include "OgreOldSkeletonManager.h"
#include "OgreOldSkeletonInstance.h"
#include "OgreOldBone.h"
#include "Animation/OgreSkeletonInstance.h"
#include "Animation/OgreSkeletonDef.h"
#include "Animation/OgreSkeletonManager.h"
#include "OgreIK.h"
#include "SkeletonViewer.h"
#include "OgreTimer.h"



using namespace Demo;

namespace Demo
{
    IKGameState::IKGameState( const Ogre::String &helpDescription ) :
        TutorialGameState( helpDescription ),
        mAnimateObjects( true )
    {
    }
    //-----------------------------------------------------------------------------------
    void IKGameState::createCerberus()
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

        Ogre::v1::SkeletonPtr skeleton = Ogre::v1::OldSkeletonManager::getSingleton().create( "Demo", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true );
        
        std::vector<Ogre::Vector3> pos =
        {
            Ogre::Vector3(0.0, 0.75, 0.0),
            Ogre::Vector3(0.0, 0.60, 0.0),
            Ogre::Vector3(0.0, 0.90, 0.0),
            Ogre::Vector3(0.0, 0.60, 0.0),
            Ogre::Vector3(0.0, 0.57, 0.0),
            Ogre::Vector3(0.0, 0.61, 0.0),
            Ogre::Vector3(0.0, 0.30, 0.0)
        };
        
        auto create_batch = [&](const Ogre::Quaternion& yRot, int& batchCount, int num)
        {
            assert( num <= pos.size() );
            
            for (size_t i = 0; i < num; i++)
            {
                Ogre::v1::OldBone* bone = skeleton->createBone( "constraint_" + Ogre::StringConverter::toString(batchCount) + "_"+ Ogre::StringConverter::toString(i), skeleton->getNumBones() );
                
                if((i==0 && skeleton->getNumBones() > 1))
                {
                    skeleton->getBone( 0 )->addChild( bone );
                }
                else if( (i > 0) )
                {
                    skeleton->getBone( skeleton->getNumBones()-2 )->addChild( bone );
                }
                
                bone->setPosition( pos[i] );
                
                
                if(i%2 == batchCount%2)
                {
                    bone->setOrientation(yRot * Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI/8), Ogre::Vector3::UNIT_X));
                }
            }
            
            Ogre::Vector3 effectorPos = skeleton->getBone(skeleton->getNumBones()-1)->_getDerivedPosition();
            Ogre::v1::OldBone* bone = skeleton->createBone( "effector_" + Ogre::StringConverter::toString(batchCount), skeleton->getNumBones() );
            bone->setPosition( effectorPos ); // effector
            
            batchCount++;
        };
        
        int batchCount = 0;
        create_batch( Ogre::Quaternion::IDENTITY, batchCount, pos.size() );
        create_batch( Ogre::Quaternion( Ogre::Radian( Ogre::Math::PI/10 ), Ogre::Vector3::UNIT_Y ), batchCount, pos.size()-2);
        create_batch( Ogre::Quaternion( Ogre::Radian( -Ogre::Math::PI/10 ), Ogre::Vector3::UNIT_Y ), batchCount, pos.size()-2);
        
        
        
        skeleton->setBindingPose();
        
        Ogre::SkeletonDefPtr def = Ogre::SkeletonManager::getSingleton().getSkeletonDef( skeleton.get() );
        
        Ogre::SkeletonInstance* skeletonInstance = sceneManager->createSkeletonInstance(def.get());
        
        sceneManager->updateSceneGraph(); // need full transforms for SkeletonViewer
        Ogre::Item* item = sceneManager->createItem( SkeletonViewer(skeletonInstance).mMesh, Ogre::SCENE_DYNAMIC );
        item->setDatablock( "Marble" );
        mSceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
        createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mSceneNode->attachObject( item );
        skeletonInstance = item->getSkeletonInstance();
        
        
        sceneManager->updateSceneGraph(); // need full transforms again! for IK
        mIK = new Ogre::IK( false );
        
        for (size_t i = 0; i < skeletonInstance->getNumBones(); i++)
        {
            Ogre::Bone* bone = skeletonInstance->getBone( i );
            
            if(i==0)
            {
                printf("insertRoot %s\n", bone->getName().c_str());
                mIK->insertRoot( bone );
            }
            else if(bone->getParent())
            {
                printf("insertChild %s\n", bone->getName().c_str());
                mIK->insertChild( bone );
            }
            else
            {
                printf("insertEffector %s hook %s\n", bone->getName().c_str(), skeletonInstance->getBone( i-1 )->getName().c_str());
                mIK->insertEffector( bone, skeletonInstance->getBone( i-1 ) );
            }
        }
        
        mIK->buildTree();
    }
    
    /*
    //-----------------------------------------------------------------------------------
    void IKGameState::imaginaryUsage()
    {
        std::vector<Ogre::Bone*> ebones;    // imagine they run from heel to hip
        Ogre::Bone* effector = 0;           // imagine we have one
        
        Ogre::IK* ik = new Ogre::IK();

        size_t nebones = ebones.size();
        for(int ei = nebones-1; ei >= 0; --ei)
        {
            Ogre::Bone* bone = ebones[ei];
            const Ogre::Vector3 axis = Ogre::Vector3::UNIT_X;
            float minTheta = -Ogre::Math::PI;
            float maxTheta = Ogre::Math::PI;
            
            if( ( ei == nebones-1 ) && ( ik->getJoints().size() == 0 ) )
            {
                ik->insertRoot( bone, axis, minTheta, maxTheta );
            }
            else
            {
                ik->insertChild( bone, axis, minTheta, maxTheta );
            }
        }
        
        ik->insertEffector( effector, ik->getJoints().back() );
        ik->buildTree();

        
     
     
        // update code
        for(Ogre::IK* ik : mIKs)
        {
            ik->reorientateRoot( ik->getEffectors().back() );
            sceneManager->updateSceneGraph();
            
            ik->update();
            sceneManager->updateSceneGraph();
        }
    }
    */
    
    //-----------------------------------------------------------------------------------
    void IKGameState::createScene01(void)
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

        Ogre::v1::MeshPtr planeMeshV1 = Ogre::v1::MeshManager::getSingleton().createPlane( "Plane v1",
                                            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                            Ogre::Plane( Ogre::Vector3::UNIT_Y, 1.0f ), 50.0f, 50.0f,
                                            1, 1, true, 1, 4.0f, 4.0f, Ogre::Vector3::UNIT_Z,
                                            Ogre::v1::HardwareBuffer::HBU_STATIC,
                                            Ogre::v1::HardwareBuffer::HBU_STATIC );

        Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingleton().createManual(
                    "Plane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

        planeMesh->importV1( planeMeshV1.get(), true, true, true );

        {
            Ogre::Item *item = sceneManager->createItem( planeMesh, Ogre::SCENE_DYNAMIC );
            item->setDatablock( "Marble" );
            Ogre::SceneNode *sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                                                    createChildSceneNode( Ogre::SCENE_DYNAMIC );
            sceneNode->setPosition( 0, -1, 0 );
            sceneNode->attachObject( item );

            //Change the addressing mode of the roughness map to wrap via code.
            //Detail maps default to wrap, but the rest to clamp.
            assert( dynamic_cast<Ogre::HlmsPbsDatablock*>( item->getSubItem(0)->getDatablock() ) );
            Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(
                                                            item->getSubItem(0)->getDatablock() );
            //Make a hard copy of the sampler block
            Ogre::HlmsSamplerblock samplerblock( *datablock->getSamplerblock( Ogre::PBSM_ROUGHNESS ) );
            samplerblock.mU = Ogre::TAM_WRAP;
            samplerblock.mV = Ogre::TAM_WRAP;
            samplerblock.mW = Ogre::TAM_WRAP;
            //Set the new samplerblock. The Hlms system will
            //automatically create the API block if necessary
            datablock->setSamplerblock( Ogre::PBSM_ROUGHNESS, samplerblock );
        }

        
        createCerberus();
        
        
        Ogre::SceneNode *rootNode = sceneManager->getRootSceneNode();

        Ogre::Light *light = sceneManager->createLight();
        Ogre::SceneNode *lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setPowerScale( 1.0f );
        light->setType( Ogre::Light::LT_DIRECTIONAL );
        light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

        mLightNodes[0] = lightNode;

        sceneManager->setAmbientLight( Ogre::ColourValue( 0.3f, 0.5f, 0.7f ) * 0.1f * 0.75f,
                                       Ogre::ColourValue( 0.6f, 0.45f, 0.3f ) * 0.065f * 0.75f,
                                       -light->getDirection() + Ogre::Vector3::UNIT_Y * 0.2f );

        light = sceneManager->createLight();
        lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setDiffuseColour( 0.8f, 0.4f, 0.2f ); //Warm
        light->setSpecularColour( 0.8f, 0.4f, 0.2f );
        light->setPowerScale( Ogre::Math::PI );
        light->setType( Ogre::Light::LT_SPOTLIGHT );
        lightNode->setPosition( -10.0f, 10.0f, 10.0f );
        light->setDirection( Ogre::Vector3( 1, -1, -1 ).normalisedCopy() );
        light->setAttenuationBasedOnRadius( 10.0f, 0.01f );

        mLightNodes[1] = lightNode;

        light = sceneManager->createLight();
        lightNode = rootNode->createChildSceneNode();
        lightNode->attachObject( light );
        light->setDiffuseColour( 0.2f, 0.4f, 0.8f ); //Cold
        light->setSpecularColour( 0.2f, 0.4f, 0.8f );
        light->setPowerScale( Ogre::Math::PI );
        light->setType( Ogre::Light::LT_SPOTLIGHT );
        lightNode->setPosition( 10.0f, 10.0f, -10.0f );
        light->setDirection( Ogre::Vector3( -1, -1, 1 ).normalisedCopy() );
        light->setAttenuationBasedOnRadius( 10.0f, 0.01f );

        mLightNodes[2] = lightNode;

        mCameraController = new CameraController( mGraphicsSystem, false );

        TutorialGameState::createScene01();
    }
    //-----------------------------------------------------------------------------------
    void IKGameState::update( float timeSinceLast )
    {
        if( mAnimateObjects )
        {
            double T = Ogre::Root::getSingleton().getTimer()->getMilliseconds()/1000.0;
            
            Ogre::Quaternion ori(Ogre::Radian(T), Ogre::Vector3::UNIT_Y);
            mIK->getJoints()[0]->setOrientation( ori );
            
            // always going to be one frame out of date unless we update skeleton transforms
            
            mIK->getEffectors()[0]->setPosition( ori * Ogre::Vector3(0.0, 1.5+1.2*Ogre::Math::Sin(3 * T), 1.6) );
            if(mIK->getEffectors().size() > 1)
                mIK->getEffectors()[1]->setPosition( ori * Ogre::Vector3(2.0, 1.5+1.2*Ogre::Math::Cos(3 * T), 1.6) );
            if(mIK->getEffectors().size() > 2)
                mIK->getEffectors()[2]->setPosition( ori * Ogre::Vector3(-2.0, 1.5+1.2*Ogre::Math::Cos(3 * T), 1.6) );
            
            mGraphicsSystem->getSceneManager()->updateSceneGraph();
            mIK->update();
            
            mSceneNode->setOrientation(Ogre::Quaternion(Ogre::Radian(T), Ogre::Vector3::UNIT_Y));
            mSceneNode->setPosition(Ogre::Quaternion(Ogre::Radian(T*0.66), Ogre::Vector3::UNIT_Y) * Ogre::Vector3::UNIT_Z*2 );
        }

        TutorialGameState::update( timeSinceLast );
    }
    //-----------------------------------------------------------------------------------
    void IKGameState::generateDebugText( float timeSinceLast, Ogre::String &outText )
    {
        TutorialGameState::generateDebugText( timeSinceLast, outText );
        outText += "\nPress F2 to toggle animation. ";
        outText += mAnimateObjects ? "[On]" : "[Off]";
    }
    //-----------------------------------------------------------------------------------
    void IKGameState::keyReleased( const SDL_KeyboardEvent &arg )
    {
        if( (arg.keysym.mod & ~(KMOD_NUM|KMOD_CAPS)) != 0 )
        {
            TutorialGameState::keyReleased( arg );
            return;
        }

        if( arg.keysym.sym == SDLK_F2 )
        {
            mAnimateObjects = !mAnimateObjects;
        }

        else
        {
            TutorialGameState::keyReleased( arg );
        }
    }
}
