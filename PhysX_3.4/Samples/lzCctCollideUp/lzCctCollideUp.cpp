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

#include "SamplePreprocessor.h"
#include "lzCctCollideUp.h"
#include "SampleUtils.h"
#include "SampleConsole.h"
#include "RendererMemoryMacros.h"
#include "RenderMeshActor.h"

#include "PxPhysics.h"
#include "PxScene.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PxPhysicsAPI.h"
#include "RenderBoxActor.h"

#include <SampleBaseInputEventIds.h>
#include <SamplePlatform.h>
#include <SampleUserInput.h>
#include <SampleUserInputIds.h>
#include <SampleUserInputDefines.h>

#include <SampleCCTActor.h>

#include <iostream>
#include <iomanip>

using namespace SampleRenderer;
using namespace SampleFramework;



ControllerContactFilter::ControllerContactFilter()
	: PxSceneQueryFilterCallback()
	, mCharacterControllerShape(NULL)
{
}

ControllerContactFilter::ControllerContactFilter(const physx::PxShape* characterControllerShape)
	: PxSceneQueryFilterCallback()
	, mCharacterControllerShape(characterControllerShape)
{
}

ControllerContactFilter::~ControllerContactFilter()
{
}

void ControllerContactFilter::setCharacterControllerShape(const physx::PxShape* characterControllerShape)
{
	mCharacterControllerShape = characterControllerShape;
}

physx::PxQueryHitType::Enum ControllerContactFilter::preFilter(const physx::PxFilterData& filterData0, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& filterFlags)
{
	if (shape == mCharacterControllerShape)
		return physx::PxQueryHitType::eNONE;

	return physx::PxQueryHitType::eBLOCK;
}

physx::PxQueryHitType::Enum ControllerContactFilter::postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit)
{
	return physx::PxQueryHitType::eBLOCK;
}


REGISTER_SAMPLE(LzCctCollideUp, "LzCctCollideUp")

///////////////////////////////////////////////////////////////////////////////

LzCctCollideUp::LzCctCollideUp(PhysXSampleApplication& app)
	: PhysXSample(app)
	, mControllerManager(NULL)
	, mController(NULL)
	, mShape(NULL)
	, mIsLanded(false)
	, mIsStopped(false)
{
	mCreateGroundPlane = false;
}

LzCctCollideUp::~LzCctCollideUp()
{
}

void LzCctCollideUp::onTickPreRender(float dtime)
{
	static int frameCount = 1;
	PxSceneWriteLock scopedLock(*mScene);

	mController->invalidateCache();

	PxFilterData data = mShape->getQueryFilterData();

	const PxControllerFilters filters(&data, &mControllerContactFilter);

	PxVec3 dist = PxVec3(0, -0.01f, 0);
	if (mIsLanded && !mIsStopped)
		dist.x = 0.01f;

	PxExtendedVec3 dbgpos = mController->getPosition();
	if (dbgpos.x > 0.579f && dbgpos.x < 0.581f)
	{
		std::cout<<"This frame collide side"<<std::endl;
	}

	PxControllerCollisionFlags collisionFlags = mController->move(dist, 0.001f, 0.02f, filters);

	if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_DOWN)
		mIsLanded = true;
	PxExtendedVec3 pos = mController->getPosition();
	mIsStopped = pos.x > 1.5f;

	if (!mIsStopped)
	{
		std::cout << std::fixed << std::setprecision(4) << "[" << frameCount << "] " << pos.x << "," << pos.y << "," << pos.z;
		std::cout << " T1: " << pos.y + characterHeight * 0.5f;
		std::cout << " T2: " << pos.y + characterHeight * 0.5f + characterSkinWidth;
		if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_SIDES)
			std::cout << " eCOLLISION_SIDES";
		if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_UP)
			std::cout << " eCOLLISION_UP";
		std::cout << std::endl;
	}

	PhysXSample::onTickPreRender(dtime);

	mActor->sync();

	frameCount++;
	Sleep(30);
}

void LzCctCollideUp::onTickPostRender(float dtime)
{
	PhysXSample::onTickPostRender(dtime);
}

void LzCctCollideUp::customizeSceneDesc(PxSceneDesc& sceneDesc)
{
	sceneDesc.flags |= PxSceneFlag::eREQUIRE_RW_LOCK;
}

void LzCctCollideUp::newMesh(const RAWMesh& data)
{
}

static void gValue(Console* console, const char* text, void* userData)
{
	if(!text)
	{
		console->out("Usage: value <float>");
		return;
	}

	const float val = (float)::atof(text);
	shdfnd::printFormatted("value: %f\n", val);
}

static void gExport(Console* console, const char* text, void* userData)
{
	if(!text)
	{
		console->out("Usage: export <filename>");
		return;
	}
}

static void gImport(Console* console, const char* text, void* userData)
{
	if(!text)
	{
		console->out("Usage: import <filename>");
		return;
	}
}

void LzCctCollideUp::onInit()
{
	roofHalfThickness = 0.05f;
	roofHeight = 1.85f;
	characterHeight = 1.75f;
	characterSkinWidth = 0.01f;

	if(getConsole())
	{
		getConsole()->addCmd("value", gValue);
		getConsole()->addCmd("export", gExport);
		getConsole()->addCmd("import", gImport);
	}

	PhysXSample::onInit();
	PxSceneWriteLock scopedLock(*mScene);

	getActiveScene().setGravity(PxVec3(0.0f, -9.81f, 0.0f));

	mApplication.setMouseCursorHiding(true);
	mApplication.setMouseCursorRecentering(true);
	mCameraController.init(PxVec3(0.0f, 1.0f, 3.0f), PxVec3(0.0f, 0.0f, 0.0f));
	mCameraController.setMouseSensitivity(0.3f);

	mControllerManager = PxCreateControllerManager(getActiveScene());

	ControlledActorDesc desc;
	desc.mType = PxControllerShapeType::eCAPSULE;
	desc.mPosition = PxExtendedVec3(0.0f, characterHeight * 0.5f + 0.2f, -3.0f);
	desc.mSlopeLimit = 0.0f;
	desc.mContactOffset = characterSkinWidth;
	// desc.mStepOffset = 0.05f;
	desc.mStepOffset = 0.3f;
	desc.mInvisibleWallHeight = 0.0f;
	desc.mMaxJumpHeight = 0.0f;
	desc.mRadius = 0.5f;
	desc.mHeight = characterHeight - 2.0f * desc.mRadius;
	desc.mCrouchHeight = 1.0f;
	desc.mReportCallback = this;
	desc.mBehaviorCallback = this;

	mActor = SAMPLE_NEW(ControlledActor)(*this);
	mActor->init(desc, mControllerManager);

	RenderBaseActor* actor0 = mActor->getRenderActorStanding();
	RenderBaseActor* actor1 = mActor->getRenderActorCrouching();
	if (actor0)
		mRenderActors.push_back(actor0);
	if (actor1)
		mRenderActors.push_back(actor1);

	mController = static_cast<PxCapsuleController*>(mActor->getController());
	mController->getActor()->getShapes(&mShape, 1);

	// floor
	CreateStaticBox(PxVec3(0.0f, -0.5f, 0.0f), PxVec3(4.0f, 0.5f, 4.0f));

	// ceiling
	CreateStaticBox(PxVec3(1.5f, roofHeight + roofHalfThickness, -3.0f), PxVec3(0.5f, roofHalfThickness, 0.5f));


}

void LzCctCollideUp::collectInputEvents(std::vector<const SampleFramework::InputEvent*>& inputEvents)
{
	PhysXSample::collectInputEvents(inputEvents);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_SPEED_INCREASE);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_SPEED_DECREASE);
    
	//touch events
	TOUCH_INPUT_EVENT_DEF(SPAWN_DEBUG_OBJECT,	"Throw Object",		ABUTTON_5,	IBUTTON_5);
}

void LzCctCollideUp::helpRender(PxU32 x, PxU32 y, PxU8 textAlpha)
{
	Renderer* renderer = getRenderer();
	const PxU32 yInc = 18;
	const PxReal scale = 0.5f;
	const PxReal shadowOffset = 6.0f;
	const RendererColor textColor(255, 255, 255, textAlpha);
	const bool isMouseSupported = getApplication().getPlatform()->getSampleUserInput()->mouseSupported();
	const bool isPadSupported = getApplication().getPlatform()->getSampleUserInput()->gamepadSupported();
	const char* msg;

	if (isMouseSupported && isPadSupported)
		renderer->print(x, y += yInc, "Use mouse or right stick to rotate", scale, shadowOffset, textColor);
	else if (isMouseSupported)
		renderer->print(x, y += yInc, "Use mouse to rotate", scale, shadowOffset, textColor);
	else if (isPadSupported)
		renderer->print(x, y += yInc, "Use right stick to rotate", scale, shadowOffset, textColor);
	if (isPadSupported)
		renderer->print(x, y += yInc, "Use left stick to move",scale, shadowOffset, textColor);
	msg = mApplication.inputMoveInfoMsg("Press "," to move", CAMERA_MOVE_FORWARD,CAMERA_MOVE_BACKWARD, CAMERA_MOVE_LEFT, CAMERA_MOVE_RIGHT);
	if(msg)
		renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
	msg = mApplication.inputInfoMsg("Press "," to move fast", CAMERA_SHIFT_SPEED, -1);
	if(msg)
		renderer->print(x, y += yInc, msg, scale, shadowOffset, textColor);
	msg = mApplication.inputInfoMsg("Press "," to throw an object", SPAWN_DEBUG_OBJECT, -1);
	if(msg)
		renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
}

void LzCctCollideUp::descriptionRender(PxU32 x, PxU32 y, PxU8 textAlpha)
{
	bool print=(textAlpha!=0.0f);

	if(print)
	{
		Renderer* renderer = getRenderer();
		const PxU32 yInc = 18;
		const PxReal scale = 0.5f;
		const PxReal shadowOffset = 6.0f;
		const RendererColor textColor(255, 255, 255, textAlpha);

		char line1[256]="This sample demonstrates how to set up and simulate a PhysX";
		char line2[256]="scene.  Further, it illustrates the creation, simulation and";
		char line3[256]="collision of simple dynamic objects.";
		renderer->print(x, y+=yInc, line1, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line2, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line3, scale, shadowOffset, textColor);
	}
}

PxU32 LzCctCollideUp::getDebugObjectTypes() const
{
	return DEBUG_OBJECT_BOX | DEBUG_OBJECT_SPHERE | DEBUG_OBJECT_CAPSULE | DEBUG_OBJECT_CONVEX;
}

void LzCctCollideUp::onShapeHit(const PxControllerShapeHit& hit)
{
	defaultCCTInteraction(hit);
}

PxControllerBehaviorFlags LzCctCollideUp::getBehaviorFlags(const PxShape& shape, const PxActor& actor)
{
	const char* gPlankName = "Plank";
	const char* gPlatformName = "Platform";

	const char* actorName = actor.getName();
#ifdef PLATFORMS_AS_OBSTACLES
	PX_ASSERT(actorName != gPlatformName);	// PT: in this mode we should have filtered out those guys already
#endif

											// PT: ride on planks
	if (actorName == gPlankName)
		return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;

	// PT: ride & slide on platforms
	if (actorName == gPlatformName)
		return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT | PxControllerBehaviorFlag::eCCT_SLIDE;

	return PxControllerBehaviorFlags(0);
}

PxControllerBehaviorFlags LzCctCollideUp::getBehaviorFlags(const PxController&)
{
	return PxControllerBehaviorFlags(0);
}

PxControllerBehaviorFlags LzCctCollideUp::getBehaviorFlags(const PxObstacle&)
{
	return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT | PxControllerBehaviorFlag::eCCT_SLIDE;
}

PxRigidActor* LzCctCollideUp::CreateStaticBox(const PxVec3& pos, const PxVec3& dimensions)
{
	PxTransform	boxPose = PxTransform(pos, PxQuat(PxIdentity));

	PxRigidActor* actor = static_cast<PxRigidActor*>(getPhysics().createRigidStatic(boxPose));
	if (!actor)
		return NULL;

	PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, PxBoxGeometry(dimensions), getDefaultMaterial());
	if (!shape)
	{
		actor->release();
		return NULL;
	}

	mScene->addActor(*actor);

	PxShape* boxShape;
	PxU32 nb = actor->getShapes(&boxShape, 1);
	createRenderBoxFromShape(actor, boxShape);

	return actor;
}
