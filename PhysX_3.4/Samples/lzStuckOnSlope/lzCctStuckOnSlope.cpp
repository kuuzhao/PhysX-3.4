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
#include "lzCctStuckOnSlope.h"
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

REGISTER_SAMPLE(LzCctStuckOnSlope, "LzCctStuckOnSlope")

#define LZ_PI 3.1415926f

///////////////////////////////////////////////////////////////////////////////

LzCctStuckOnSlope::LzCctStuckOnSlope(PhysXSampleApplication& app)
    : PhysXSample(app)
    , mControllerManager(NULL)
    , mController(NULL)
    , mShape(NULL)
    , mIsLanded(false)
    , mIsStopped(false)
{
    mCreateGroundPlane = false;
}

LzCctStuckOnSlope::~LzCctStuckOnSlope()
{
}


class ControllerContactFilter2 : public physx::PxSceneQueryFilterCallback
{
public:

    ControllerContactFilter2() : PxSceneQueryFilterCallback()
        , mCharacterControllerShape(NULL)
    {
    }
    ControllerContactFilter2(const physx::PxShape* characterControllerShape) : PxSceneQueryFilterCallback()
        , mCharacterControllerShape(characterControllerShape)
    {
    }

    ~ControllerContactFilter2() {}

    void setCharacterControllerShape(const physx::PxShape* characterControllerShape)
    {
        mCharacterControllerShape = characterControllerShape;
    }

    physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filterData0, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& filterFlags)
    {
        if (shape == mCharacterControllerShape)
            return physx::PxQueryHitType::eNONE;

        return physx::PxQueryHitType::eBLOCK;
    }
    physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit)
    {
        return physx::PxQueryHitType::eBLOCK;
    }

private:

    const physx::PxShape* mCharacterControllerShape;
};

static ControllerContactFilter2 sControllerContactFilter;

void LzCctStuckOnSlope::onTickPreRender(float dtime)
{
    static int frameCount = 1;
    PxSceneWriteLock scopedLock(*mScene);

    mController->invalidateCache();

    PxFilterData data = mShape->getQueryFilterData();

    const PxControllerFilters filters(&data, &sControllerContactFilter);

    PxVec3 dist = PxVec3(0.0f, -1.00000000f, 0.0f);
    if (mIsLanded && !mIsStopped)
    {
        dist.x += 0.0317387618f;
        dist.z += 0.0948295891;
    }

    PxControllerCollisionFlags collisionFlags = mController->move(dist, 0.001f, 0.02f, filters);

    if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_DOWN)
        mIsLanded = true;
    PxExtendedVec3 pos = mController->getPosition();
    mIsStopped = pos.z > 10.0f;

    if (frameCount < 80)
    {
#if 1
        std::cout << std::fixed << std::setprecision(7) << "[" << frameCount << "] " << pos.x << "," << pos.y << "," << pos.z;
        if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_SIDES)
            std::cout << " eCOLLISION_SIDES";
        if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_UP)
            std::cout << " eCOLLISION_UP";
        std::cout << std::endl;
#endif
    }

    PhysXSample::onTickPreRender(dtime);

    mActor->sync();

    frameCount++;
    Sleep(30);
}

void LzCctStuckOnSlope::onTickPostRender(float dtime)
{
    PhysXSample::onTickPostRender(dtime);
}

void LzCctStuckOnSlope::customizeSceneDesc(PxSceneDesc& sceneDesc)
{
    sceneDesc.flags |= PxSceneFlag::eREQUIRE_RW_LOCK;
}

void LzCctStuckOnSlope::newMesh(const RAWMesh& data)
{
}

static void gValue(Console* console, const char* text, void* userData)
{
    if (!text)
    {
        console->out("Usage: value <float>");
        return;
    }

    const float val = (float)::atof(text);
    shdfnd::printFormatted("value: %f\n", val);
}

static void gExport(Console* console, const char* text, void* userData)
{
    if (!text)
    {
        console->out("Usage: export <filename>");
        return;
    }
}

static void gImport(Console* console, const char* text, void* userData)
{
    if (!text)
    {
        console->out("Usage: import <filename>");
        return;
    }
}

void LzCctStuckOnSlope::onInit()
{
    characterHeight = 2.0f;
    characterSkinWidth = 0.08f;
    slopeAngle = -25.0f;

    if (getConsole())
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
    mCameraController.init(PxVec3(-1.0f, 3.0f, -5.0f), PxVec3(0.0f, LZ_PI-0.1f, 0.0f));
    mCameraController.setMouseSensitivity(0.3f);

    mControllerManager = PxCreateControllerManager(getActiveScene());

    ControlledActorDesc desc;
    desc.mType = PxControllerShapeType::eCAPSULE;
    desc.mPosition = PxExtendedVec3(-1.178261f, characterHeight * 0.5f + 0.8110746f, 1.98483f);
    desc.mSlopeLimit = 0.707106769f;
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
    CreateStaticBox(PxVec3(0.0f, -0.5f, 5.0f), PxQuat(PxIdentity), PxVec3(8.0f, 0.5f, 8.0f));

    // slope
    CreateStaticBox(PxVec3(-2.0f, 2.0f, 6.0f),
        PxQuat(slopeAngle / 180.0f * 3.1415926, PxVec3(1.0f, 0.0f, 0.0f)), PxVec3(2.5f, 0.5f, 5.0f));

    // wall
    CreateStaticBox(PxVec3(2.5f, 2.0f, 6.0f),
        PxQuat(PxIdentity), PxVec3(2.5f, 2.5f, 5.0f));

    CreateStaticBox(PxVec3(0.0f, 0.0f, 0.0f),
        PxQuat(PxIdentity), PxVec3(0.5f, 0.5f, 0.5f));

}

void LzCctStuckOnSlope::collectInputEvents(std::vector<const SampleFramework::InputEvent*>& inputEvents)
{
    PhysXSample::collectInputEvents(inputEvents);
    getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_SPEED_INCREASE);
    getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_SPEED_DECREASE);

    //touch events
    TOUCH_INPUT_EVENT_DEF(SPAWN_DEBUG_OBJECT, "Throw Object", ABUTTON_5, IBUTTON_5);
}

void LzCctStuckOnSlope::helpRender(PxU32 x, PxU32 y, PxU8 textAlpha)
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
        renderer->print(x, y += yInc, "Use left stick to move", scale, shadowOffset, textColor);
    msg = mApplication.inputMoveInfoMsg("Press ", " to move", CAMERA_MOVE_FORWARD, CAMERA_MOVE_BACKWARD, CAMERA_MOVE_LEFT, CAMERA_MOVE_RIGHT);
    if (msg)
        renderer->print(x, y += yInc, msg, scale, shadowOffset, textColor);
    msg = mApplication.inputInfoMsg("Press ", " to move fast", CAMERA_SHIFT_SPEED, -1);
    if (msg)
        renderer->print(x, y += yInc, msg, scale, shadowOffset, textColor);
    msg = mApplication.inputInfoMsg("Press ", " to throw an object", SPAWN_DEBUG_OBJECT, -1);
    if (msg)
        renderer->print(x, y += yInc, msg, scale, shadowOffset, textColor);
}

void LzCctStuckOnSlope::descriptionRender(PxU32 x, PxU32 y, PxU8 textAlpha)
{
    bool print = (textAlpha != 0.0f);

    if (print)
    {
        Renderer* renderer = getRenderer();
        const PxU32 yInc = 18;
        const PxReal scale = 0.5f;
        const PxReal shadowOffset = 6.0f;
        const RendererColor textColor(255, 255, 255, textAlpha);

        char line1[256] = "This sample demonstrates how to set up and simulate a PhysX";
        char line2[256] = "scene.  Further, it illustrates the creation, simulation and";
        char line3[256] = "collision of simple dynamic objects.";
        renderer->print(x, y += yInc, line1, scale, shadowOffset, textColor);
        renderer->print(x, y += yInc, line2, scale, shadowOffset, textColor);
        renderer->print(x, y += yInc, line3, scale, shadowOffset, textColor);
    }
}

PxU32 LzCctStuckOnSlope::getDebugObjectTypes() const
{
    return DEBUG_OBJECT_BOX | DEBUG_OBJECT_SPHERE | DEBUG_OBJECT_CAPSULE | DEBUG_OBJECT_CONVEX;
}

void LzCctStuckOnSlope::onShapeHit(const PxControllerShapeHit& hit)
{
    defaultCCTInteraction(hit);
}

PxControllerBehaviorFlags LzCctStuckOnSlope::getBehaviorFlags(const PxShape& shape, const PxActor& actor)
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

PxControllerBehaviorFlags LzCctStuckOnSlope::getBehaviorFlags(const PxController&)
{
    return PxControllerBehaviorFlags(0);
}

PxControllerBehaviorFlags LzCctStuckOnSlope::getBehaviorFlags(const PxObstacle&)
{
    return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT | PxControllerBehaviorFlag::eCCT_SLIDE;
}

PxRigidActor* LzCctStuckOnSlope::CreateStaticBox(const PxVec3& pos, const PxQuat& rot, const PxVec3& dimensions)
{
    PxTransform	boxPose = PxTransform(pos, rot);

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
