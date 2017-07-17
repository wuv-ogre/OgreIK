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

#include "OgreIK.h"

#include "Animation/OgreBone.h"

#include "BussIK/Node.h"
#include "BussIK/Tree.h"
#include "BussIK/Jacobian.h"
#include "BussIK/VectorRn.h"


namespace BussIK
{
    typedef Node Node; // clashes with Ogre
}

namespace Ogre
{
    struct IK::PrivateContext
    {
        enum Method
        {
            IK_JACOB_TRANS=0,
            IK_PURE_PSEUDO,
            IK_DLS,
            IK_SDLS,
            IK_DLS_SVD
        };
        
        Ogre::uint8 m_method;
        Tree m_tree;
        Jacobian* m_jacobian;
        std::vector<VectorR3> m_targetaa;           // index = effector sequence position
        
        Ogre::Vector3 m_rootPos;                    // derived pos
        Ogre::Quaternion m_rootOri;                 // derived ori
        
        std::vector<Ogre::Quaternion> m_bindOri;    // TODO: get this from the SkeletonDef?
        
        std::vector<Ogre::Bone*> m_effectors;
        std::vector<Ogre::Bone*> m_joints;
        
        bool m_modifyExistingOrientations;

        PrivateContext( bool modifyExistingOrientations ) : m_modifyExistingOrientations( modifyExistingOrientations ), m_jacobian( 0 ), m_method( IK_DLS_SVD )
        {
        }
        
        ~PrivateContext()
        {
            delete m_jacobian;
        }
        
        VectorR3 toBussIK(const Ogre::Vector3& v)
        {
            return VectorR3(v.x, v.y, v.z);
        }
        
        void getFullTransformUpdated(Ogre::Bone* bone, Ogre::Vector3& pos, Ogre::Quaternion& ori)
        {
            Ogre::Matrix4 m;
            bone->_getFullTransformUpdated().store4x3( &m );
            
            ori = m.extractQuaternion();
            pos = m.getTrans();
        }
        
        void getRelativeFromRoot( Ogre::Bone* bone, Ogre::Vector3& pos, Ogre::Quaternion& ori )
        {
            getFullTransformUpdated( bone, pos, ori );
            pos -= m_rootPos;
            pos = m_rootOri.Inverse() * pos;
            ori = m_rootOri.Inverse() * ori;
        }
        
        void reorientateRoot( Ogre::Bone* effector )
        {
            Ogre::Bone* root = m_joints[0];
            getFullTransformUpdated( root, m_rootPos, m_rootOri );
            
            Ogre::Vector3 pos;
            Ogre::Quaternion ori;
            getRelativeFromRoot(effector, pos, ori);
            
            root->setOrientation( root->getOrientation() *  (Ogre::Vector3::NEGATIVE_UNIT_Y).getRotationTo(  pos ) );
        }
        
        size_t findJointIndex(Ogre::Bone* bone)
        {
            auto it = std::find( m_joints.begin(), m_joints.end(), bone );
            if(it == m_joints.end())
            {
                OGRE_EXCEPT( Exception::ERR_ITEM_NOT_FOUND, "parent bone named " + bone->getName() + " must be added.", "IK::findJointIndex" );
            }
            return std::distance( m_joints.begin(), it );
        }
        
        void insertRoot(Ogre::Bone* root, const Ogre::Vector3& axis, const float minTheta, const float maxTheta, const float restTheta)
        {
            if(m_tree.GetRoot())
            {
                OGRE_EXCEPT( Exception::ERR_DUPLICATE_ITEM, "Root already exists.", "IK::insertRoot" );
            }
            
            BussIK::Node* node = new BussIK::Node( VectorR3::Zero, toBussIK(axis), 0.0, JOINT, minTheta, maxTheta, restTheta );
            m_tree.InsertRoot( node );
            
            m_joints.push_back( root );
            
            getFullTransformUpdated(root, m_rootPos, m_rootOri);
        }
        
        void insertChild(Ogre::Bone* bone, const Ogre::Vector3& axis, const float minTheta, const float maxTheta, const float restTheta)
        {
            size_t parentIndex = findJointIndex( bone->getParent() );
            
            Ogre::Vector3 pos;
            Ogre::Quaternion ori;
            getRelativeFromRoot( bone, pos, ori );
            
            BussIK::Node* node = new BussIK::Node( toBussIK( pos ), toBussIK( axis ), 0.0, JOINT, minTheta, maxTheta, restTheta );
            BussIK::Node* parent = m_tree.GetJoint( parentIndex );
            
            if(parent->left)
            {
                BussIK::Node* sibling = parent->left;
                while(sibling->right)
                {
                    sibling = sibling->right;
                }
                m_tree.InsertRightSibling( sibling, node );
            }
            else
            {
                m_tree.InsertLeftChild( parent, node );
            }
            
            m_joints.push_back( bone );
        }
        
        void insertEffector(Ogre::Bone* effector, Ogre::Bone* hook)
        {
            size_t hookIndex = findJointIndex( hook );
            
            Ogre::Vector3 pos;
            Ogre::Quaternion ori;
            getRelativeFromRoot( effector, pos, ori );
            
            BussIK::Node* node = new BussIK::Node( toBussIK( pos ), VectorR3::Zero, 0.08, EFFECTOR );
            BussIK::Node* parent = m_tree.GetJoint( hookIndex );
            
            if(parent->left)
            {
                BussIK::Node* sibling = parent->left;
                while(sibling->right)
                {
                    sibling = sibling->right;
                }
                m_tree.InsertRightSibling( sibling, node );
            }
            else
            {
                m_tree.InsertLeftChild( parent, node );
            }
            
            m_effectors.push_back( effector );
        }
        
        void buildTree()
        {
            if(m_jacobian)
                delete m_jacobian;

            m_targetaa.resize( m_effectors.size() );
            
            for(size_t ci=0; ci<m_joints.size(); ci++)
            {
                m_bindOri.push_back( m_joints[ci]->getOrientation() );
            }

            m_jacobian = new Jacobian( &m_tree );
            m_tree.Init();
            m_tree.Compute();
            m_jacobian->Reset();
            
            // Jacobian matrix based on target positions
            //m_jacobian->SetJtargetActive();
            
            // Jacobian matrix based on end effector positions
            m_jacobian->SetJendActive();
        }
        
        void updateEffectorTargets()
        {
            /*
            BussIK::Node* n = m_tree.GetRoot();
            while ( n )
            {
                if ( n->IsEffector() )
                {
                    Ogre::Vector3 pos;
                    Ogre::Quaternion ori;
                    getRelativeFromRoot( m_effectors[n->seqNumEffector], pos, ori );
                    m_targetaa[n->seqNumEffector].Set(pos.x, pos.y, pos.z);
                    cout << "updateEffectorTargets " << n->seqNumEffector << endl;
                }
                n = m_tree.GetSuccessor( n );
            }
            */
            
            for(size_t i=0; i<m_effectors.size(); ++i)
            {
                Ogre::Vector3 pos;
                Ogre::Quaternion ori;
                getRelativeFromRoot( m_effectors[i], pos, ori );
                m_targetaa[i].Set(pos.x, pos.y, pos.z);
                
            }
        }
        
        void updateJoints()
        {
            BussIK::Node* n = m_tree.GetRoot();
            while ( n )
            {
                if ( !n->IsEffector() )
                {
                    Ogre::Vector3 axis( n->v.x, n->v.y, n->v.z );
                    Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
                    if ( axis.length() )
                    {
                        rot = Ogre::Quaternion( Ogre::Radian( n->theta ), axis );
                    }
                    
                    if(n->seqNumJoint == 0)
                    {
                        m_joints[n->seqNumJoint]->setOrientation( m_joints[n->seqNumJoint]->getOrientation() *  rot );
                    }
                    else
                    {
                        if(m_modifyExistingOrientations)
                        {
                            m_joints[n->seqNumJoint]->setOrientation( m_joints[n->seqNumJoint]->getOrientation() * m_bindOri[n->seqNumJoint] * rot );
                        }
                        else
                        {
                            m_joints[n->seqNumJoint]->setOrientation( m_bindOri[n->seqNumJoint] * rot );
                        }
                    }
                }
                n = m_tree.GetSuccessor( n );
            }
        }
        
        void update()
        {
            getFullTransformUpdated(m_joints[0], m_rootPos, m_rootOri);
            
            updateEffectorTargets();

            // Set up Jacobian and deltaS vectors
            m_jacobian->ComputeJacobian(&m_targetaa[0]);
            
            // Calculate the change in theta values
            switch (m_method)
            {
                case IK_JACOB_TRANS:
                    m_jacobian->CalcDeltaThetasTranspose();		// Jacobian transpose method
                    break;
                case IK_DLS:
                    m_jacobian->CalcDeltaThetasDLS();			// Damped least squares method
                    break;
                case IK_DLS_SVD:
                    m_jacobian->CalcDeltaThetasDLSwithSVD();
                    break;
                case IK_PURE_PSEUDO:
                    m_jacobian->CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
                    break;
                case IK_SDLS:
                    m_jacobian->CalcDeltaThetasSDLS();			// Selectively damped least squares method
                    break;
                default:
                    m_jacobian->ZeroDeltaThetas();
                    break;
            }

            m_jacobian->UpdateThetas();							// Apply the change in the theta values
            m_jacobian->UpdatedSClampValue(&m_targetaa[0]);


            updateJoints();
        }
        
        
        
        
        
        
        
        
        /*
        std::vector<Ogre::SceneNode*> m_debugNodes;
        
        void getLocalTransform(const BussIK::Node* node, Ogre::Matrix4& act)
        {
            Ogre::Vector3 axis(node->v.x, node->v.y, node->v.z);
            Ogre::Quaternion rot = Ogre::Quaternion::IDENTITY;
            if (axis.length())
            {
                rot = Ogre::Quaternion(Ogre::Radian(node->theta), axis);
            }
            
            act.makeTransform(Ogre::Vector3(node->r.x, node->r.y, node->r.z), Ogre::Vector3::UNIT_SCALE, rot);
        }
        
        void draw(BussIK::Node* node, const Ogre::Matrix4& tr, int& index)
        {
            if (node != 0)
            {
                Ogre::Vector3 pos = tr.getTrans();

                Ogre::Vector3 axisLocal(node->v.x, node->v.y, node->v.z);
                Ogre::Vector3 axisWorld = tr.extractQuaternion()*axisLocal;
                
                m_debugNodes[index]->setPosition(m_rootOri * pos);
                m_debugNodes[index]->setOrientation(tr.extractQuaternion());
                
                Ogre::Quaternion accumRot = Ogre::Quaternion::IDENTITY;
                
                BussIK::Node* parent = node->realparent;
                while(parent)
                {
                    if(parent->IsJoint() )
                    {
                        if(parent->seqNumJoint >= m_bindOri.size())
                            assert(false);
                        
                        accumRot = m_bindOri[parent->seqNumJoint] * accumRot ;
                    }
                    parent = parent->realparent;
                }
                
                
                m_debugNodes[index]->setOrientation(m_rootOri * accumRot * tr.extractQuaternion());
                
                // to child
                if(node->left)
                {
                    Ogre::Real l = Ogre::Vector3(node->left->r.x, node->left->r.y, node->left->r.z).length();
                    const Real boneWidthScale = 0.5;
                    m_debugNodes[index]->setScale( Ogre::Vector3( boneWidthScale * l, l, boneWidthScale * l ) );
                }
                
                if (node->left)
                {
                    Ogre::Matrix4 act;
                    getLocalTransform(node->left, act);
                    
                    Ogre::Matrix4 trl = tr*act;
                    draw(node->left, trl, ++index);
                }
                if (node->right)
                {
                    Ogre::Matrix4 act;
                    getLocalTransform(node->right, act);
                    
                    Ogre::Matrix4 trr = tr*act;
                    draw(node->right, trr, ++index);
                }
            }
        }
        
        void debug()
        {
            if(!m_debugNodes.size())
            {
                Ogre::SceneManager* sceneManager = Ogre::Root::getSingleton().getSceneManagerIterator().getNext();

                for(int i=0; i<m_effectors.size() + m_joints.size(); i++)
                {
                    Ogre::SceneNode* node = sceneManager->getRootSceneNode()->createChildSceneNode();
                    Ogre::Item* item = sceneManager->createItem( "Bone.mesh" );
                    node->attachObject( item );
                    m_debugNodes.push_back( node );
                }
            }
            
            int index = 0;
            Ogre::Matrix4 act;
            getLocalTransform(m_tree.GetRoot(), act);

            // for comparison set it one unit down x axis
            act.setTrans(m_joints[0]->getPosition() + Ogre::Vector3::UNIT_X);
            
            draw(m_tree.GetRoot(), act, index);
        }
        */
        
    };
    
    
    IK::IK( bool modifyExistingOrientations )
    {
        m_context = new PrivateContext( modifyExistingOrientations );
    }
    
    IK::~IK()
    {
        delete m_context;
        m_context = 0;
    }
    
    void IK::buildTree()
    {
        m_context->buildTree();
    }
    
    void IK::update()
    {
        m_context->update();
        //m_context->debug();
    }

    void IK::insertRoot(Ogre::Bone* root, const Ogre::Vector3& axis, const float minTheta, const float maxTheta, const float restTheta)
    {
        m_context->insertRoot(root, axis, minTheta, maxTheta, restTheta);
    }
    
    void IK::insertChild(Ogre::Bone* bone, const Ogre::Vector3& axis, const float minTheta, const float maxTheta, const float restTheta)
    {
        m_context->insertChild(bone, axis, minTheta, maxTheta, restTheta);
    }
    
    void IK::insertEffector(Ogre::Bone* effector, Ogre::Bone* hook)
    {
        m_context->insertEffector(effector, hook);
    }
    
    const std::vector<Ogre::Bone*>& IK::getEffectors() const
    {
        return m_context->m_effectors;
    }
    
    const std::vector<Ogre::Bone*>& IK::getJoints() const
    {
        return m_context->m_joints;
    }
    
    void IK::reorientateRoot( Ogre::Bone* effector )
    {
        m_context->reorientateRoot( effector );
    }
}
