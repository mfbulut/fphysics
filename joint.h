// Experimental Joint Addon may be unstable

#ifndef FPHYSICS_JOINT_H
#define FPHYSICS_JOINT_H

#include "fphysics.h"

typedef enum {
    FORCE_JOINT,
    HINGE_JOINT,
    FIXED_JOINT
} JointType;

typedef struct {
    Anchor anchorA;
    Anchor anchorB;
    float initialLength;
    float relativeOrientation;
    JointType type;
} Joint;

Joint CreateJoint(Anchor anchorA, Anchor anchorB, JointType type) {
    Joint joint = { 0 };
    joint.anchorA = anchorA;
    joint.anchorB = anchorB;
    joint.type = type;

    Vector2 worldAnchorA = AnchorPosition(joint.anchorA);
    Vector2 worldAnchorB = AnchorPosition(joint.anchorB);
    joint.initialLength = Vector2Length(Vector2Subtract(worldAnchorA, worldAnchorB));
    joint.relativeOrientation = anchorB.rigidbody->polygon->orientation - anchorA.rigidbody->polygon->orientation;

    return joint;
}

void ForceConstraint(Joint* joint, float dt) {
    Vector2 anchorAPos = AnchorPosition(joint->anchorA);
    Vector2 anchorBPos = AnchorPosition(joint->anchorB);

    Vector2 delta = Vector2Subtract(anchorBPos, anchorAPos);
    float currentLength = Vector2Length(delta);

    float lengthError = currentLength - joint->initialLength;
    Vector2 correction = Vector2Scale(Vector2Normalize(delta), lengthError * 4000.0f);

    ApplyForceAtPoint(joint->anchorA.rigidbody, correction, anchorAPos);
    ApplyForceAtPoint(joint->anchorB.rigidbody, Vector2Negate(correction), anchorBPos);
}

void HingeConstraint(Joint* joint, float dt) {
    Vector2 anchorAPos = AnchorPosition(joint->anchorA);
    Vector2 anchorBPos = AnchorPosition(joint->anchorB);

    Vector2 anchorDir = Vector2Subtract(anchorAPos, anchorBPos);
    float distance = Vector2Length(anchorDir);
    if (distance < 0.01f) {
        return;
    }

    anchorDir = Vector2Normalize(anchorDir);
    CollisionManifold contact = CreateCollisionManifold(0, anchorDir, anchorBPos);
    contact.rigidbodyA = joint->anchorA.rigidbody;
    contact.rigidbodyB = joint->anchorB.rigidbody;

    if (distance > joint->initialLength) {
        contact.depth = distance - joint->initialLength;
    } else {
        contact.depth = joint->initialLength - distance;
        contact.normal = Vector2Scale(contact.normal, -1);
    }

    ResolveCollision(contact);
    PositionalCorrection(contact);
}

void RotationConstraint(Joint* joint, float dt) {
    float currentOrientationDiff = joint->anchorB.rigidbody->polygon->orientation - joint->anchorA.rigidbody->polygon->orientation;
    float orientationError = joint->relativeOrientation - currentOrientationDiff;
    joint->anchorB.rigidbody->angularVelocity += orientationError * 5.0f;
}

void UpdateJoint(Joint* joint, float dt) {
    switch (joint->type) {
        case FORCE_JOINT:
            ForceConstraint(joint, dt);
            break;
        case HINGE_JOINT:
            HingeConstraint(joint, dt);
            break;
        case FIXED_JOINT:
            HingeConstraint(joint, dt);
            RotationConstraint(joint, dt);
            break;
        default:
            break;
    }
}

#endif
