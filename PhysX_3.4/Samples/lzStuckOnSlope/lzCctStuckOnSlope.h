// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef LZ_CCT_STUCK_ON_SLOPE_H
#define LZ_CCT_STUCK_ON_SLOPE_H

#include "PhysXSample.h"

class ControlledActor;

class LzCctStuckOnSlope :
    public PhysXSample,
    public PxControllerBehaviorCallback,
    public PxUserControllerHitReport
{
public:
    ControlledActor * mActor;

    PxControllerManager* mControllerManager;
    PxCapsuleController* mController;
    PxShape* mShape;
    bool	mIsLanded;
    bool	mIsStopped;

    // debug params
    float characterHeight;
    float characterSkinWidth;
    float slopeAngle;


    LzCctStuckOnSlope(PhysXSampleApplication& app);
    virtual									~LzCctStuckOnSlope();

    virtual	void							onTickPreRender(float dtime);
    virtual	void							onTickPostRender(float dtime);
    virtual	void							customizeSceneDesc(PxSceneDesc&);

    virtual	void							newMesh(const RAWMesh&);
    virtual	void							onInit();
    virtual	void						    onInit(bool restart) { onInit(); }

    virtual void							collectInputEvents(std::vector<const SampleFramework::InputEvent*>& inputEvents);
    virtual void							helpRender(PxU32 x, PxU32 y, PxU8 textAlpha);
    virtual	void							descriptionRender(PxU32 x, PxU32 y, PxU8 textAlpha);
    virtual PxU32							getDebugObjectTypes() const;

    virtual void							onShapeHit(const PxControllerShapeHit& hit);
    virtual void							onControllerHit(const PxControllersHit& hit) {}
    virtual void							onObstacleHit(const PxControllerObstacleHit& hit) {}

    virtual PxControllerBehaviorFlags		getBehaviorFlags(const PxShape& shape, const PxActor& actor);
    virtual PxControllerBehaviorFlags		getBehaviorFlags(const PxController& controller);
    virtual PxControllerBehaviorFlags		getBehaviorFlags(const PxObstacle& obstacle);

    PxRigidActor* CreateStaticBox(const PxVec3& pos, const PxQuat& rot, const PxVec3& dimensions);


};

#endif
