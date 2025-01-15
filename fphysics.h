/**
 ********************************************************************************
 * @file    fphysics.h
 * @author  mfbulut
 * @date    15.01.2025
 ********************************************************************************
**/

#ifndef FPHYSICS_H
#define FPHYSICS_H

#include "raylib.h"
#include "raymath.h"

// Polygon
typedef struct {
    Vector2 centroid;
    Vector2* vertices;
    Vector2* normals;
    int vertexCount;
    float orientation;
} Polygon;

Polygon* CreatePolygon(Vector2* vertices, int vertexCount);
Polygon* CreateRectangle(Vector2 position, float width, float height);
void DestroyPolygon(Polygon* polygon);
Vector2 CalculateCentroid(Vector2* vertices, int vertexCount);
Vector2* CalculateNormals(Vector2* vertices, int vertexCount);
float CalculateArea(Vector2* vertices, int vertexCount);
void MovePolygon(Polygon* polygon, Vector2 delta);
void RotatePolygon(Polygon* polygon, float radians);
float CalculateInertia(Polygon* polygon, float mass);
Vector2 RotateAroundPoint(Vector2 toRotate, Vector2 point, float radians);

// Rigidbody
typedef struct {
    float restitution;
    float friction;
} PhysicsMaterial;

typedef struct {
    Polygon* polygon;
    Vector2 forceAccumulator;
    Vector2 velocity;
    PhysicsMaterial material;
    float mass;
    float invMass;
    float torqueAccumulator;
    float angularVelocity;
    float inertia;
    float invInertia;
    int isKinematic;
} Rigidbody;

Rigidbody CreateRigidbody(Polygon* polygon, float mass, PhysicsMaterial material);
void AddForce(Rigidbody* body, Vector2 force);
void UpdateRigidbody(Rigidbody* body, float deltaTime);

// Anchor
typedef struct {
    Vector2 point;
    Rigidbody* rigidbody;
} Anchor;

Anchor CreateAnchor(Rigidbody* rigidbody, Vector2 p3);
Vector2 AnchorPosition(Anchor anchor);

// Collision Manifold
typedef struct {
    float depth;
    Vector2 normal;
    Vector2 penetrationPoint;
    Rigidbody* rigidbodyA;
    Rigidbody* rigidbodyB;
    bool colliding;
} CollisionManifold;

CollisionManifold CreateCollisionManifold(float depth, Vector2 normal, Vector2 penetrationPoint);
void ResolveCollision(CollisionManifold manifold);
void PositionalCorrection(CollisionManifold manifold);

// Collision Dedection
typedef struct {
    Vector2 vertex;
    float penetrationDepth;
    bool valid;
} SupportPoint;

CollisionManifold CheckCollisions(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB);
CollisionManifold GetContactPoint(Polygon* shapePolygonA, Polygon* shapePolygonB);
SupportPoint FindSupportPoint(Vector2 normalOnEdge, Vector2 pointOnEdge, Vector2* otherPolygonVertices, int vertexCount);
void HandleCollision(Rigidbody* rb1, Rigidbody* rb2);

#ifdef FPHYSICS_IMPLEMENTATION

// Polygon

Polygon* CreatePolygon(Vector2* vertices, int vertexCount) {
    Polygon* polygon = (Polygon*)MemAlloc(sizeof(Polygon));
    polygon->vertices = (Vector2*)MemAlloc(vertexCount * sizeof(Vector2));
    for (int i = 0; i < vertexCount; i++) {
        polygon->vertices[i] = vertices[i];
    }
    polygon->vertexCount = vertexCount;
    polygon->centroid = CalculateCentroid(vertices, vertexCount);
    polygon->normals = CalculateNormals(vertices, vertexCount);
    return polygon;
}

Polygon* CreateRectangle(Vector2 position, float width, float height) {
    Vector2 vertices[] = {
        (Vector2){position.x - width / 2, position.y + height / 2},
        (Vector2){position.x + width / 2, position.y + height / 2},
        (Vector2){position.x + width / 2, position.y - height / 2},
        (Vector2){position.x - width / 2, position.y - height / 2}
    };

    Polygon* rectangle = CreatePolygon(&vertices[0], 4);
    return rectangle;
}

void DestroyPolygon(Polygon* polygon) {
    MemFree(polygon->vertices);
    MemFree(polygon->normals);
    MemFree(polygon);
}

Vector2 CalculateCentroid(Vector2* vertices, int vertexCount) {
    float area = CalculateArea(vertices, vertexCount);
    float Cx = 0, Cy = 0;

    for (int i = 0; i < vertexCount; i++) {
        int next = (i + 1) % vertexCount;
        float commonTerm = (vertices[i].x * vertices[next].y) - (vertices[next].x * vertices[i].y);
        Cx += (vertices[i].x + vertices[next].x) * commonTerm;
        Cy += (vertices[i].y + vertices[next].y) * commonTerm;
    }

    Cx /= (6 * area);
    Cy /= (6 * area);

    return (Vector2){ Cx, Cy };
}

Vector2* CalculateNormals(Vector2* vertices, int vertexCount) {
    Vector2* normals = (Vector2*)MemAlloc(vertexCount * sizeof(Vector2));
    for (int i = 0; i < vertexCount; i++) {
        Vector2 direction = Vector2Subtract(vertices[i], vertices[(i + 1) % vertexCount]);
        direction = Vector2Normalize(direction);
        normals[i] = (Vector2){ direction.y, -direction.x };
    }
    return normals;
}

float CalculateArea(Vector2* vertices, int vertexCount) {
    float A = 0;
    for (int i = 0; i < vertexCount; i++) {
        int next = (i + 1) % vertexCount;
        A += vertices[i].x * vertices[next].y - vertices[next].x * vertices[i].y;
    }
    return A / 2;
}

void MovePolygon(Polygon* polygon, Vector2 delta) {
    for (int i = 0; i < polygon->vertexCount; i++) {
        polygon->vertices[i] = Vector2Add(polygon->vertices[i], delta);
    }
    polygon->centroid = Vector2Add(polygon->centroid, delta);
}

void RotatePolygon(Polygon* polygon, float radians) {
    for (int i = 0; i < polygon->vertexCount; i++) {
        polygon->vertices[i] = RotateAroundPoint(polygon->vertices[i], polygon->centroid, radians);
    }
    MemFree(polygon->normals);
    polygon->normals = CalculateNormals(polygon->vertices, polygon->vertexCount);
    polygon->orientation += radians;
}

float CalculateInertia(Polygon* polygon, float mass) {
    float inertia = 0;
    float massPerTriangleFace = mass / polygon->vertexCount;
    for (int i = 0; i < polygon->vertexCount; i++) {
        Vector2 centerToVertice0 = Vector2Subtract(polygon->vertices[i], polygon->centroid);
        int indexVertice1 = (i + 1) % polygon->vertexCount;
        Vector2 centerToVertice1 = Vector2Subtract(polygon->vertices[indexVertice1], polygon->centroid);
        float inertiaTriangle = massPerTriangleFace * (Vector2LengthSqr(centerToVertice0) + Vector2LengthSqr(centerToVertice1) + Vector2DotProduct(centerToVertice0, centerToVertice1)) / 6;
        inertia += inertiaTriangle;
    }
    return inertia;
}

Vector2 RotateAroundPoint(Vector2 toRotate, Vector2 point, float radians) {
    Vector2 dir = Vector2Subtract(toRotate, point);
    Vector2 rotated = { dir.x * cosf(radians) - dir.y * sinf(radians), dir.x * sinf(radians) + dir.y * cosf(radians) };
    return Vector2Add(rotated, point);
}

// Rigidbody

Rigidbody CreateRigidbody(Polygon* polygon, float mass, PhysicsMaterial material) {
    Rigidbody body = { 0 };
    body.polygon = polygon;
    body.mass = mass;
    body.isKinematic = 0;
    if (mass > 0.000001) {
        body.invMass = 1.0f / mass;
    } else {
        body.invMass = 0;
        body.isKinematic = 1;
    }
    body.torqueAccumulator = 0;
    body.forceAccumulator = (Vector2){ 0, 0 };
    body.velocity = (Vector2){ 0, 0 };
    body.angularVelocity = 0;
    body.material = material;
    body.inertia = CalculateInertia(polygon, mass);
    if (body.inertia > 0.00001) {
        body.invInertia = 1.0f / body.inertia;
    } else {
        body.invInertia = 0;
    }
    return body;
}

void AddForce(Rigidbody* body, Vector2 force) {
    body->forceAccumulator = Vector2Add(body->forceAccumulator, force);
}

void ApplyForceAtPoint(Rigidbody* body, Vector2 force, Vector2 point) {
    Vector2 centerOfMass = body->polygon->centroid;
    Vector2 r = Vector2Subtract(point, centerOfMass);
    float torque = Vector2CrossProduct(r, force);

    body->torqueAccumulator += torque;
    body->forceAccumulator = Vector2Add(body->forceAccumulator, force);
}

void UpdateRigidbody(Rigidbody* body, float deltaTime) {
    if (body->invMass > 0) {
        Vector2 acceleration = Vector2Scale(body->forceAccumulator, body->invMass);
        body->velocity = Vector2Add(body->velocity, Vector2Scale(acceleration, deltaTime));
        Vector2 deltaPosition = Vector2Scale(body->velocity, deltaTime);
        MovePolygon(body->polygon, deltaPosition);
    }

    if (body->invInertia > 0) {
        float rotationalAcceleration = body->torqueAccumulator * body->invInertia;
        body->angularVelocity += rotationalAcceleration * deltaTime;
        float deltaRotation = body->angularVelocity * deltaTime;
        RotatePolygon(body->polygon, deltaRotation);
    }

    // body->velocity = Vector2Scale(body->velocity, 0.999f);
    body->angularVelocity *= 0.995f;
    body->forceAccumulator = (Vector2){ 0, 0 };
    body->torqueAccumulator = 0;
}

// Anchor
Anchor CreateAnchor(Rigidbody* rigidbody, Vector2 p3) {
    Vector2 p1 = rigidbody->polygon->vertices[0];
    Vector2 p2 = rigidbody->polygon->vertices[1];
    Vector2 d = Vector2Subtract(p2, p1);
    Vector2 dPerp = (Vector2){-d.y, d.x};
    Vector2 dp3 = Vector2Subtract(p3, p1);

    float denom = d.x * d.x + d.y * d.y;
    float x = (dp3.x * d.x + dp3.y * d.y) / denom;
    float y = (dp3.x * dPerp.x + dp3.y * dPerp.y) / denom;

    Anchor anchor = { .point = {x, y}, .rigidbody = rigidbody };
    return anchor;
}

Vector2 AnchorPosition(Anchor anchor) {
    Vector2 p1 = anchor.rigidbody->polygon->vertices[0];
    Vector2 p2 = anchor.rigidbody->polygon->vertices[1];
    Vector2 d = Vector2Subtract(p2, p1);
    Vector2 dPerp = (Vector2){-d.y, d.x};

    return Vector2Add(p1, Vector2Add(Vector2Scale(d, anchor.point.x), Vector2Scale(dPerp, anchor.point.y)));
}

// Collision Manifold

CollisionManifold CreateCollisionManifold(float depth, Vector2 normal, Vector2 penetrationPoint) {
    CollisionManifold manifold;
    manifold.depth = depth;
    manifold.normal = normal;
    manifold.penetrationPoint = penetrationPoint;
    manifold.rigidbodyA = 0;
    manifold.rigidbodyB = 0;
    manifold.colliding = false;
    return manifold;
}

void ResolveCollision(CollisionManifold manifold) {
    if (manifold.rigidbodyA->isKinematic && manifold.rigidbodyB->isKinematic) return;

    Vector2 penetrationToCentroidA = Vector2Subtract(manifold.penetrationPoint, manifold.rigidbodyA->polygon->centroid);
    Vector2 penetrationToCentroidB = Vector2Subtract(manifold.penetrationPoint, manifold.rigidbodyB->polygon->centroid);

    Vector2 angularVelocityPenetrationCentroidA = (Vector2){
        -1 * manifold.rigidbodyA->angularVelocity * penetrationToCentroidA.y,
        manifold.rigidbodyA->angularVelocity * penetrationToCentroidA.x
    };

    Vector2 angularVelocityPenetrationCentroidB = (Vector2){
        -1 * manifold.rigidbodyB->angularVelocity * penetrationToCentroidB.y,
        manifold.rigidbodyB->angularVelocity * penetrationToCentroidB.x
    };

    Vector2 relativeVelocityA = Vector2Add(manifold.rigidbodyA->velocity, angularVelocityPenetrationCentroidA);
    Vector2 relativeVelocityB = Vector2Add(manifold.rigidbodyB->velocity, angularVelocityPenetrationCentroidB);

    Vector2 relativeVel = Vector2Subtract(relativeVelocityB, relativeVelocityA);
    float velocityInNormal = Vector2DotProduct(relativeVel, manifold.normal);

    if (velocityInNormal > 0) return;

    float e = fmin(manifold.rigidbodyA->material.restitution, manifold.rigidbodyB->material.restitution);
    float pToCentroidCrossNormalA = Vector2CrossProduct(penetrationToCentroidA, manifold.normal);
    float pToCentroidCrossNormalB = Vector2CrossProduct(penetrationToCentroidB, manifold.normal);

    float invMassSum = manifold.rigidbodyA->invMass + manifold.rigidbodyB->invMass;

    float rigiAInvInertia = manifold.rigidbodyA->invInertia;
    float rigiBInvInertia = manifold.rigidbodyB->invInertia;
    float crossNSum = pToCentroidCrossNormalA * pToCentroidCrossNormalA * rigiAInvInertia + pToCentroidCrossNormalB * pToCentroidCrossNormalB * rigiBInvInertia;

    float j = -(1 + e) * velocityInNormal;
    j /= (invMassSum + crossNSum);

    Vector2 impulseVector = Vector2Scale(manifold.normal, j);

    manifold.rigidbodyA->velocity = Vector2Subtract(manifold.rigidbodyA->velocity, Vector2Scale(impulseVector, manifold.rigidbodyA->invMass));
    manifold.rigidbodyB->velocity = Vector2Add(manifold.rigidbodyB->velocity, Vector2Scale(impulseVector, manifold.rigidbodyB->invMass));
    manifold.rigidbodyA->angularVelocity += -pToCentroidCrossNormalA * j * rigiAInvInertia;
    manifold.rigidbodyB->angularVelocity += pToCentroidCrossNormalB * j * rigiBInvInertia;

    // Frictional impulse
    Vector2 velocityInNormalDirection = Vector2Scale(manifold.normal, Vector2DotProduct(relativeVel, manifold.normal));
    Vector2 tangent = Vector2Subtract(relativeVel, velocityInNormalDirection);
    tangent = Vector2Scale(tangent, -1);
    float minFriction = fmin(manifold.rigidbodyA->material.friction, manifold.rigidbodyB->material.friction);
    if (Vector2Length(tangent) > 0.00001f) {
        tangent = Vector2Normalize(tangent);
    }

    float pToCentroidCrossTangentA = Vector2CrossProduct(penetrationToCentroidA, tangent);
    float pToCentroidCrossTangentB = Vector2CrossProduct(penetrationToCentroidB, tangent);

    float crossSumTangent = pToCentroidCrossTangentA * pToCentroidCrossTangentA * rigiAInvInertia + pToCentroidCrossTangentB * pToCentroidCrossTangentB * rigiBInvInertia;
    float frictionalImpulse = -(1 + e) * Vector2DotProduct(relativeVel, tangent) * minFriction;
    frictionalImpulse /= (invMassSum + crossSumTangent);
    if (frictionalImpulse > j) {
        frictionalImpulse = j;
    }

    Vector2 frictionalImpulseVector = Vector2Scale(tangent, frictionalImpulse);

    manifold.rigidbodyA->velocity = Vector2Subtract(manifold.rigidbodyA->velocity, Vector2Scale(frictionalImpulseVector, manifold.rigidbodyA->invMass));
    manifold.rigidbodyB->velocity = Vector2Add(manifold.rigidbodyB->velocity, Vector2Scale(frictionalImpulseVector, manifold.rigidbodyB->invMass));

    manifold.rigidbodyA->angularVelocity += -pToCentroidCrossTangentA * frictionalImpulse * rigiAInvInertia;
    manifold.rigidbodyB->angularVelocity += pToCentroidCrossTangentB * frictionalImpulse * rigiBInvInertia;
}

void PositionalCorrection(CollisionManifold manifold) {
    float correctionPercentage = 0.9f;
    float amountToCorrect = (manifold.depth / (manifold.rigidbodyA->invMass + manifold.rigidbodyB->invMass)) * correctionPercentage;
    Vector2 correctionVector = Vector2Scale(manifold.normal, amountToCorrect);

    Vector2 rigiAMovement = Vector2Scale(correctionVector, manifold.rigidbodyA->invMass * -1);
    Vector2 rigiBMovement = Vector2Scale(correctionVector, manifold.rigidbodyB->invMass);

    MovePolygon(manifold.rigidbodyA->polygon, rigiAMovement);
    MovePolygon(manifold.rigidbodyB->polygon, rigiBMovement);
}

// Collision Dedection

CollisionManifold CheckCollisions(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB) {
    Polygon* shapeA = rigidbodyA->polygon;
    Polygon* shapeB = rigidbodyB->polygon;

    CollisionManifold resultingContact = { 0 };

    CollisionManifold contactPolyA = GetContactPoint(shapeA, shapeB);
    if (!contactPolyA.colliding) return resultingContact;

    CollisionManifold contactPolyB = GetContactPoint(shapeB, shapeA);
    if (!contactPolyB.colliding) return resultingContact;

    if (contactPolyA.depth < contactPolyB.depth) {
        Vector2 minus = Vector2Scale(contactPolyA.normal, contactPolyA.depth);
        resultingContact = CreateCollisionManifold(contactPolyA.depth, contactPolyA.normal, Vector2Subtract(contactPolyA.penetrationPoint, minus));
    } else {
        resultingContact = CreateCollisionManifold(contactPolyB.depth, Vector2Scale(contactPolyB.normal, -1), contactPolyB.penetrationPoint);
    }

    resultingContact.colliding = true;
    resultingContact.rigidbodyA = rigidbodyA;
    resultingContact.rigidbodyB = rigidbodyB;
    return resultingContact;
}

CollisionManifold GetContactPoint(Polygon* shapePolygonA, Polygon* shapePolygonB) {
    CollisionManifold contact = { 0 };
    float minimumPenetration = 100000.0f;

    for (int i = 0; i < shapePolygonA->vertexCount; i++) {
        Vector2 pointOnEdge = shapePolygonA->vertices[i];
        Vector2 normalOnEdge = shapePolygonA->normals[i];

        SupportPoint supportPoint = FindSupportPoint(normalOnEdge, pointOnEdge, shapePolygonB->vertices, shapePolygonB->vertexCount);
        if(!supportPoint.valid) return contact;

        if (supportPoint.penetrationDepth < minimumPenetration) {
            minimumPenetration = supportPoint.penetrationDepth;
            contact = CreateCollisionManifold(minimumPenetration, normalOnEdge, supportPoint.vertex);
        }
    }

    contact.colliding = true;
    return contact;
}

SupportPoint FindSupportPoint(Vector2 normalOnEdge, Vector2 pointOnEdge, Vector2* otherPolygonVertices, int vertexCount) {
    float currentDeepestPenetration = 0;
    SupportPoint supportPoint = { 0 };

    for (int i = 0; i < vertexCount; i++) {
        Vector2 vertice = otherPolygonVertices[i];
        Vector2 verticeToPointEdge = Vector2Subtract(vertice, pointOnEdge);
        float penetrationDepth = Vector2DotProduct(verticeToPointEdge, Vector2Scale(normalOnEdge, -1));

        if (penetrationDepth > currentDeepestPenetration) {
            currentDeepestPenetration = penetrationDepth;
            supportPoint.vertex = vertice;
            supportPoint.penetrationDepth = currentDeepestPenetration;
            supportPoint.valid = true;
        }
    }
    return supportPoint;
}

void HandleCollision(Rigidbody* rb1, Rigidbody* rb2) {
    CollisionManifold collisionManifold = CheckCollisions(rb1, rb2);
    if (collisionManifold.colliding) {
        ResolveCollision(collisionManifold);
        PositionalCorrection(collisionManifold);
    }
}

#endif
#endif