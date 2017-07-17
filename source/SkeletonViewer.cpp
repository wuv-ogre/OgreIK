#include "SkeletonViewer.h"
#include <Animation/OgreSkeletonDef.h>

#include <OgreMeshManager2.h>
#include <OgreMesh2.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreAsyncTicket.h>

#include <OgreBitwise.h>
#include <OgreSubItem.h>
#include <Animation/OgreSkeletonInstance.h>

#include <iostream>

namespace Demo
{
    SkeletonViewer::SkeletonViewer( Ogre::SkeletonInstance* skeleton, const Ogre::Real boneWidthScale )
    {

        struct Vertices
        {
            float px, py, pz;   //Position
            float nx, ny, nz;   //Normals
            
            Vertices() {}
            Vertices( float _px, float _py, float _pz,
                     float _nx, float _ny, float _nz ) :
            px( _px ), py( _py ), pz( _pz ),
            nx( _nx ), ny( _ny ), nz( _nz )
            {
            }
        };
        
        /*
        auto read = []( const Ogre::MeshPtr mesh, std::vector< Vertices >& verts, void * &dstData  )
        {
            Ogre::VertexArrayObject *vao = mesh->getSubMesh(0)->mVao[Ogre::VpNormal][0];
            Ogre::VertexBufferPacked* vertexBuffer = vao->getVertexBuffers()[0];

            verts.resize( vertexBuffer->getNumElements() );
            
            Ogre::VertexArrayObject::ReadRequestsArray readRequests;
            readRequests.push_back( Ogre::VES_POSITION );
            readRequests.push_back( Ogre::VES_NORMAL );
            vao->readRequests( readRequests, 0, vao->getPrimitiveCount() );
            
            vao->mapAsyncTickets( readRequests );
            
            for( size_t i=0; i<vertexBuffer->getNumElements(); ++i )
            {
                // Copy positions
                {
                    if( Ogre::v1::VertexElement::getBaseType( readRequests[0].type ) == Ogre::VET_HALF2 )
                    {
                        Ogre::uint16 const * RESTRICT_ALIAS bufferF16 =
                        reinterpret_cast<Ogre::uint16 const * RESTRICT_ALIAS>( readRequests[0].data );
                        
                        verts[i].px = Ogre::Bitwise::halfToFloat( bufferF16[0] );
                        verts[i].py = Ogre::Bitwise::halfToFloat( bufferF16[1] );
                        verts[i].pz = Ogre::Bitwise::halfToFloat( bufferF16[2] );
                    }
                    else
                    {
                        float const * RESTRICT_ALIAS bufferF32 =
                        reinterpret_cast<float const * RESTRICT_ALIAS>( readRequests[0].data );
                        
                        verts[i].px = bufferF32[0];
                        verts[i].py = bufferF32[1];
                        verts[i].pz = bufferF32[2];
                    }
                    readRequests[0].data += readRequests[0].vertexBuffer->getBytesPerElement();
                }
                
                // Copy normals
                {
                    if( Ogre::v1::VertexElement::getBaseType( readRequests[1].type ) == Ogre::VET_SHORT4_SNORM )
                    {
                        const Ogre::int16 *bufferF16 = reinterpret_cast<const Ogre::int16*>( readRequests[1].data );
                        
                        verts[i].nx = Ogre::Bitwise::snorm16ToFloat( bufferF16[0] );
                        verts[i].ny = Ogre::Bitwise::snorm16ToFloat( bufferF16[1] );
                        verts[i].nz = Ogre::Bitwise::snorm16ToFloat( bufferF16[2] );
                    }
                    else
                    {
                        float const * RESTRICT_ALIAS bufferF32 =
                        reinterpret_cast<float const * RESTRICT_ALIAS>( readRequests[1].data );
                        
                        verts[i].nx = bufferF32[0];
                        verts[i].ny = bufferF32[1];
                        verts[i].nz = bufferF32[2];
                    }
                    readRequests[1].data += readRequests[1].vertexBuffer->getBytesPerElement();
                }
            }
            
            vao->unmapAsyncTickets( readRequests );
            
            dstData = OGRE_MALLOC_SIMD( vertexBuffer->getTotalSizeBytes(), Ogre::MEMCATEGORY_GEOMETRY );
            Ogre::FreeOnDestructor dataPtrContainer( dstData );
            
            Ogre::AsyncTicketPtr asyncTicket = vertexBuffer->readRequest( 0, vertexBuffer->getNumElements() );
            const void *srcData = asyncTicket->map();
            memcpy( dstData, srcData, vertexBuffer->getTotalSizeBytes() );
            asyncTicket->unmap();
        };
        
        
        
        
        std::vector< Vertices > verts;
        void * dstData;
        Ogre::MeshPtr boneMesh = Ogre::MeshManager::getSingleton().load( "Bone.mesh", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME );
        read( boneMesh, verts, dstData);
        Ogre::FreeOnDestructor dataPtrContainer( dstData );
        */

        
        mMesh = Ogre::MeshManager::getSingleton().createManual( skeleton->getDefinition()->getNameStr() + "SkeletonViewer", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        mMesh->setSkeletonName( skeleton->getDefinition()->getNameStr() );

        
        
        std::vector< Vertices > verts;
        void * dstData;
        Ogre::MeshPtr boneMesh = Ogre::MeshManager::getSingleton().load( "Bone.mesh", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME );

        {
            Ogre::VertexArrayObject *vao = boneMesh->getSubMesh(0)->mVao[Ogre::VpNormal][0];
            Ogre::VertexBufferPacked* vertexBuffer = vao->getVertexBuffers()[0];
            
            verts.resize (vertexBuffer->getNumElements() );
            
            Ogre::VertexArrayObject::ReadRequestsArray readRequests;
            readRequests.push_back( Ogre::VES_POSITION );
            readRequests.push_back( Ogre::VES_NORMAL );
            vao->readRequests( readRequests, 0, vao->getPrimitiveCount() );
            
            vao->mapAsyncTickets( readRequests );
            
            for( size_t i=0; i<vertexBuffer->getNumElements(); ++i )
            {
                // Copy positions
                {
                    if( Ogre::v1::VertexElement::getBaseType( readRequests[0].type ) == Ogre::VET_HALF2 )
                    {
                        Ogre::uint16 const * RESTRICT_ALIAS bufferF16 =
                        reinterpret_cast<Ogre::uint16 const * RESTRICT_ALIAS>( readRequests[0].data );
                        
                        verts[i].px = Ogre::Bitwise::halfToFloat( bufferF16[0] );
                        verts[i].py = Ogre::Bitwise::halfToFloat( bufferF16[1] );
                        verts[i].pz = Ogre::Bitwise::halfToFloat( bufferF16[2] );
                    }
                    else
                    {
                        float const * RESTRICT_ALIAS bufferF32 =
                        reinterpret_cast<float const * RESTRICT_ALIAS>( readRequests[0].data );
                        
                        verts[i].px = bufferF32[0];
                        verts[i].py = bufferF32[1];
                        verts[i].pz = bufferF32[2];
                    }
                    readRequests[0].data += readRequests[0].vertexBuffer->getBytesPerElement();
                }
                
                // Copy normals
                {
                    if( Ogre::v1::VertexElement::getBaseType( readRequests[1].type ) == Ogre::VET_SHORT4_SNORM )
                    {
                        const Ogre::int16 *bufferF16 =
                        reinterpret_cast<const Ogre::int16*>( readRequests[1].data );
                        
                        verts[i].nx = Ogre::Bitwise::snorm16ToFloat( bufferF16[0] );
                        verts[i].ny = Ogre::Bitwise::snorm16ToFloat( bufferF16[1] );
                        verts[i].nz = Ogre::Bitwise::snorm16ToFloat( bufferF16[2] );
                    }
                    else
                    {
                        float const * RESTRICT_ALIAS bufferF32 =
                        reinterpret_cast<float const * RESTRICT_ALIAS>( readRequests[1].data );
                        
                        verts[i].nx = bufferF32[0];
                        verts[i].ny = bufferF32[1];
                        verts[i].nz = bufferF32[2];
                    }
                    readRequests[1].data += readRequests[1].vertexBuffer->getBytesPerElement();
                }
            }
            
            vao->unmapAsyncTickets( readRequests );
            
            dstData = OGRE_MALLOC_SIMD( vertexBuffer->getTotalSizeBytes(), Ogre::MEMCATEGORY_GEOMETRY );
            
            Ogre::AsyncTicketPtr asyncTicket = vertexBuffer->readRequest( 0, vertexBuffer->getNumElements() );
            const void *srcData = asyncTicket->map();
            memcpy( dstData, srcData, vertexBuffer->getTotalSizeBytes() );
            asyncTicket->unmap();
        }
        Ogre::FreeOnDestructor dataPtrContainer( dstData );
        
        
        
        
        
        

        
        
        Ogre::uint16 nbones = skeleton->getNumBones();
        for(Ogre::uint16 bi=0; bi<nbones; ++bi)
        {
            Ogre::Bone* child = skeleton->getBone(bi);
            Ogre::Bone* bone = child->getParent();
            
            boneMesh->getSubMesh(0)->clone(mMesh.get(), Ogre::BT_DEFAULT);
            
            Ogre::uint16 pbi = bi; //find_bone_index( skeleton, bone);
            
            Ogre::Quaternion rot ( Ogre::Quaternion::IDENTITY );
            Ogre::Matrix4 m;
            if(bone)
            {
                
                bone->_getFullTransformUpdated().store4x3(&m);
                
                Ogre::Real l = child->getPosition().length();
                Ogre::Vector3 scale( boneWidthScale * l, l, boneWidthScale * l );
                
                Ogre::Quaternion localRot = Ogre::Vector3::UNIT_Y.getRotationTo( child->getPosition() );
                rot = localRot * m.extractQuaternion();
                
                m.makeTransform( m.getTrans(), scale, rot );

                
                for(Ogre::uint16 i = 0; i<skeleton->getNumBones(); ++i)
                {
                    if(skeleton->getBone(i) == bone)
                    {
                        pbi = i;
                        break;
                    }
                }
            }
            else
            {
                child->_getFullTransformUpdated().store4x3( &m );
            }
            
            
            Ogre::SubMesh* submesh = mMesh->getSubMesh(bi);
            
            Ogre::VertexArrayObject *vao = submesh->mVao[Ogre::VpNormal][0];
            Ogre::VertexBufferPacked* vertexBuffer = vao->getVertexBuffers()[0];

            Ogre::uint32 bytesPerElement = vertexBuffer->getTotalSizeBytes() / vertexBuffer->getNumElements();
            
            char* RESTRICT_ALIAS modData = reinterpret_cast< char* RESTRICT_ALIAS > (dstData);
            for( size_t i=0; i<vertexBuffer->getNumElements(); ++i )
            {
                Ogre::Vector3 p = m * Ogre::Vector3(verts[i].px, verts[i].py, verts[i].pz);
                
                //std::cout << "p" << p << std::endl;

                const Ogre::VertexElementType & vet = vertexBuffer->getVertexElements()[0].mType;
                if( Ogre::v1::VertexElement::getBaseType( vet ) == Ogre::VET_HALF2 ) //
                {
                    Ogre::uint16 * RESTRICT_ALIAS bufferF16 =
                    reinterpret_cast<Ogre::uint16 * RESTRICT_ALIAS>( modData );
                    
                    bufferF16[0] = Ogre::Bitwise::floatToHalf(p.x);
                    bufferF16[1] = Ogre::Bitwise::floatToHalf(p.y);
                    bufferF16[2] = Ogre::Bitwise::floatToHalf(p.z);
                }
                else
                {
                    float * RESTRICT_ALIAS bufferF32 =
                    reinterpret_cast<float * RESTRICT_ALIAS>( modData );
                    
                    bufferF32[0] = p.x;
                    bufferF32[1] = p.y;
                    bufferF32[2] = p.z;
                }
                
                modData += Ogre::v1::VertexElement::getTypeSize( vet );
                
                Ogre::Vector3 n = rot * Ogre::Vector3(verts[i].nx, verts[i].ny, verts[i].nz);
                
                {
                    float * RESTRICT_ALIAS bufferF32 =
                    reinterpret_cast<float * RESTRICT_ALIAS>( modData );
                    
                    bufferF32[0] = n.x;
                    bufferF32[1] = n.y;
                    bufferF32[2] = n.z;
                }
                
                modData += bytesPerElement - Ogre::v1::VertexElement::getTypeSize( vet );
            }
            
            // upload modified data
            vertexBuffer->upload(dstData, 0, vertexBuffer->getNumElements());
            
            for( size_t i=0; i<vertexBuffer->getNumElements(); ++i )
            {
                submesh->addBoneAssignment( Ogre::VertexBoneAssignment( i, pbi, 1.0 ) );
            }
            submesh->_compileBoneAssignments();
        }
        
        mMesh->_updateBoundsFromVertexBuffers();
        
        dataPtrContainer.ptr = 0;
    }
    //-----------------------------------------------------------------------------------
    
}
