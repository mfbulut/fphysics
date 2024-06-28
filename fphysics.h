/**
 ********************************************************************************
 * @file    fphysics.h
 * @author  mfbulut
 * @date    29.06.2024
 ********************************************************************************
**/

#ifndef FPHYSICS_H
#define FPHYSICS_H

#include "raylib.h"
#include "raymath.h"

// Utils
Vector2 Perpendicular(Vector2 v);
float Vector2CrossProduct(Vector2 v1, Vector2 v2);

// Polygon
typedef struct {
    Vector2* vertices;
    int vertexCount;
    Color color;
    Vector2 centroid;
    Vector2* normals;
} Polygon;

Polygon* CreatePolygon(Vector2* vertices, int vertexCount, Color color);
Polygon* CreateRectangle(Vector2 position, float width, float height, Color color);
void DestroyPolygon(Polygon* polygon);
Vector2 CalculateCentroid(Vector2* vertices, int vertexCount);
Vector2* CalculateNormals(Vector2* vertices, int vertexCount);
float CalculateArea(Vector2* vertices, int vertexCount);
int IsPointInside(Polygon* polygon, Vector2 pos);
void DrawPolygon(Polygon* polygon);
void MovePolygon(Polygon* polygon, Vector2 delta);
void RotatePolygon(Polygon* polygon, float radians);
float CalculateInertia(Polygon* polygon, float mass);
Vector2 RotateAroundPoint(Vector2 toRotate, Vector2 point, float radians);

// Rigidbody
typedef struct {
    float bounce;
    float friction;
} PhysicsMaterial;

typedef struct {
    Polygon* polygon;
    float mass;
    int isKinematic;
    float invMass;
    float torqueAccumulator;
    Vector2 forceAccumulator;
    Vector2 velocity;
    float angularVelocity;
    PhysicsMaterial material;
    float inertia;
    float invInertia;
} Rigidbody;

Rigidbody CreateRigidbody(Polygon* polygon, float mass, PhysicsMaterial material);
void AddForce(Rigidbody* body, Vector2 force);
void UpdateRigidbody(Rigidbody* body, float deltaTime);
void SemiImplicitEuler(Rigidbody* body, float deltaTime);

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
    Rigidbody* rigiA;
    Rigidbody* rigiB;
    bool colliding;
} CollisionManifold;

CollisionManifold CreateCollisionManifold(float depth, Vector2 normal, Vector2 penetrationPoint);
void FlipNormal(CollisionManifold manifold);
void ResolveCollision(CollisionManifold manifold);
void PositionalCorrection(CollisionManifold manifold);

// Collision Dedection
typedef struct {
    Vector2 vertex;
    float penetrationDepth;
    bool valid;
} SupportPoint;

CollisionManifold CheckCollisions(Rigidbody* rigiA, Rigidbody* rigiB);
CollisionManifold GetContactPoint(Polygon* shapePolygonA, Polygon* shapePolygonB);
SupportPoint FindSupportPoint(Vector2 normalOnEdge, Vector2 pointOnEdge, Vector2* otherPolygonVertices, int vertexCount);
bool HandleCollision(Rigidbody* rb1, Rigidbody* rb2);

#ifdef FPHYSICS_IMPLEMENTATION

// Utils

Vector2 Perpendicular(Vector2 v) {
    return (Vector2){-v.y, v.x};
}

float Vector2CrossProduct(Vector2 v1, Vector2 v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

// Polygon

Polygon* CreatePolygon(Vector2* vertices, int vertexCount, Color color) {
    Polygon* polygon = (Polygon*)MemAlloc(sizeof(Polygon));
    polygon->vertices = (Vector2*)MemAlloc(vertexCount * sizeof(Vector2));
    for (int i = 0; i < vertexCount; i++) {
        polygon->vertices[i] = vertices[i];
    }
    polygon->vertexCount = vertexCount;
    polygon->color = color;
    polygon->centroid = CalculateCentroid(vertices, vertexCount);
    polygon->normals = CalculateNormals(vertices, vertexCount);
    return polygon;
}

Polygon* CreateRectangle(Vector2 position, float width, float height, Color color) {
    Vector2 vertices[] = {
        (Vector2){position.x - width / 2, position.y - height / 2},
        (Vector2){position.x + width / 2, position.y - height / 2},
        (Vector2){position.x + width / 2, position.y + height / 2},
        (Vector2){position.x - width / 2, position.y + height / 2}
    };

    Polygon* rectangle = CreatePolygon(&vertices[0], 4, color);
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
        normals[i] = (Vector2){ -direction.y, direction.x };
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

int IsPointInside(Polygon* polygon, Vector2 pos) {
    for (int i = 0; i < polygon->vertexCount; i++) {
        Vector2 vertex = polygon->vertices[i];
        Vector2 normal = polygon->normals[i];
        Vector2 vertToPoint = Vector2Subtract(pos, vertex);
        float dot = Vector2DotProduct(vertToPoint, normal);
        if (dot > 0) return 0;
    }
    return 1;
}

void DrawPolygon(Polygon* polygon) {
    for (int i = 0; i < polygon->vertexCount; i++) {
        DrawLineV(polygon->vertices[i], polygon->vertices[(i + 1) % polygon->vertexCount], polygon->color);
    }
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
    SemiImplicitEuler(body, deltaTime);
    body->velocity = Vector2Scale(body->velocity, 0.999f);
    body->angularVelocity *= 0.995f;
    body->forceAccumulator = (Vector2){ 0, 0 };
    body->torqueAccumulator = 0;
}

void SemiImplicitEuler(Rigidbody* body, float deltaTime) {
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
}

// Anchor
Anchor CreateAnchor(Rigidbody* rigidbody, Vector2 p3) {
    Vector2 p1 = rigidbody->polygon->vertices[0];
    Vector2 p2 = rigidbody->polygon->vertices[1];
    Vector2 d = Vector2Subtract(p2, p1);
    Vector2 dPerp = Perpendicular(d);
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
    Vector2 dPerp = Perpendicular(d);

    return Vector2Add(p1, Vector2Add(Vector2Scale(d, anchor.point.x), Vector2Scale(dPerp, anchor.point.y)));
}

// Collision Manifold

CollisionManifold CreateCollisionManifold(float depth, Vector2 normal, Vector2 penetrationPoint) {
    CollisionManifold manifold;
    manifold.depth = depth;
    manifold.normal = normal;
    manifold.penetrationPoint = penetrationPoint;
    manifold.rigiA = 0;
    manifold.rigiB = 0;
    manifold.colliding = false;
    return manifold;
}

void ResolveCollision(CollisionManifold manifold) {
    if (manifold.rigiA->isKinematic && manifold.rigiB->isKinematic) return;

    Vector2 penetrationToCentroidA = Vector2Subtract(manifold.penetrationPoint, manifold.rigiA->polygon->centroid);
    Vector2 penetrationToCentroidB = Vector2Subtract(manifold.penetrationPoint, manifold.rigiB->polygon->centroid);

    Vector2 angularVelocityPenetrationCentroidA = (Vector2){
        -1 * manifold.rigiA->angularVelocity * penetrationToCentroidA.y,
        manifold.rigiA->angularVelocity * penetrationToCentroidA.x
    };

    Vector2 angularVelocityPenetrationCentroidB = (Vector2){
        -1 * manifold.rigiB->angularVelocity * penetrationToCentroidB.y,
        manifold.rigiB->angularVelocity * penetrationToCentroidB.x
    };

    Vector2 relativeVelocityA = Vector2Add(manifold.rigiA->velocity, angularVelocityPenetrationCentroidA);
    Vector2 relativeVelocityB = Vector2Add(manifold.rigiB->velocity, angularVelocityPenetrationCentroidB);

    Vector2 relativeVel = Vector2Subtract(relativeVelocityB, relativeVelocityA);
    float velocityInNormal = Vector2DotProduct(relativeVel, manifold.normal);

    if (velocityInNormal > 0) return;

    float e = fmin(manifold.rigiA->material.bounce, manifold.rigiB->material.bounce);
    float pToCentroidCrossNormalA = Vector2CrossProduct(penetrationToCentroidA, manifold.normal);
    float pToCentroidCrossNormalB = Vector2CrossProduct(penetrationToCentroidB, manifold.normal);

    float invMassSum = manifold.rigiA->invMass + manifold.rigiB->invMass;

    float rigiAInvInertia = manifold.rigiA->invInertia;
    float rigiBInvInertia = manifold.rigiB->invInertia;
    float crossNSum = pToCentroidCrossNormalA * pToCentroidCrossNormalA * rigiAInvInertia + pToCentroidCrossNormalB * pToCentroidCrossNormalB * rigiBInvInertia;

    float j = -(1 + e) * velocityInNormal;
    j /= (invMassSum + crossNSum);

    Vector2 impulseVector = Vector2Scale(manifold.normal, j);

    manifold.rigiA->velocity = Vector2Subtract(manifold.rigiA->velocity, Vector2Scale(impulseVector, manifold.rigiA->invMass));
    manifold.rigiB->velocity = Vector2Add(manifold.rigiB->velocity, Vector2Scale(impulseVector, manifold.rigiB->invMass));
    manifold.rigiA->angularVelocity += -pToCentroidCrossNormalA * j * rigiAInvInertia;
    manifold.rigiB->angularVelocity += pToCentroidCrossNormalB * j * rigiBInvInertia;

    // Frictional impulse
    Vector2 velocityInNormalDirection = Vector2Scale(manifold.normal, Vector2DotProduct(relativeVel, manifold.normal));
    Vector2 tangent = Vector2Subtract(relativeVel, velocityInNormalDirection);
    tangent = Vector2Scale(tangent, -1);
    float minFriction = fmin(manifold.rigiA->material.friction, manifold.rigiB->material.friction);
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

    manifold.rigiA->velocity = Vector2Subtract(manifold.rigiA->velocity, Vector2Scale(frictionalImpulseVector, manifold.rigiA->invMass));
    manifold.rigiB->velocity = Vector2Add(manifold.rigiB->velocity, Vector2Scale(frictionalImpulseVector, manifold.rigiB->invMass));

    manifold.rigiA->angularVelocity += -pToCentroidCrossTangentA * frictionalImpulse * rigiAInvInertia;
    manifold.rigiB->angularVelocity += pToCentroidCrossTangentB * frictionalImpulse * rigiBInvInertia;
}

void PositionalCorrection(CollisionManifold manifold) {
    float correctionPercentage = 0.9f;
    float amountToCorrect = (manifold.depth / (manifold.rigiA->invMass + manifold.rigiB->invMass)) * correctionPercentage;
    Vector2 correctionVector = Vector2Scale(manifold.normal, amountToCorrect);

    Vector2 rigiAMovement = Vector2Scale(correctionVector, manifold.rigiA->invMass * -1);
    Vector2 rigiBMovement = Vector2Scale(correctionVector, manifold.rigiB->invMass);

    MovePolygon(manifold.rigiA->polygon, rigiAMovement);
    MovePolygon(manifold.rigiB->polygon, rigiBMovement);
}

// Collision Dedection

CollisionManifold CheckCollisions(Rigidbody* rigiA, Rigidbody* rigiB) {
    Polygon* shapeA = rigiA->polygon;
    Polygon* shapeB = rigiB->polygon;

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
    resultingContact.rigiA = rigiA;
    resultingContact.rigiB = rigiB;
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

bool HandleCollision(Rigidbody* rb1, Rigidbody* rb2) {
    CollisionManifold collisionManifold = CheckCollisions(rb1, rb2);
    if (collisionManifold.colliding) {
        ResolveCollision(collisionManifold);
        PositionalCorrection(collisionManifold);
        return true;
    }
    return false;
}

#endif
#endif