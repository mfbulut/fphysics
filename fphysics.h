#ifndef FPHYSICS_H
#define FPHYSICS_H

#include "raylib.h"
#include "raymath.h"

typedef struct {
    Vector2 *vertices;
    Vector2 *normals;
    Vector2 *transformedVertices;
    Vector2 *transformedNormals;
    Vector2 position;
    Vector2 velocity;
    float rotation;
    float angularVelocity;
    float mass;
    float invMass;
    float inertia;
    float invInertia;
    float restitution;
    float friction;
    int vertexCount;
    bool isKinematic;
} Rigidbody;

typedef struct {
    float depth;
    Vector2 normal;
    Vector2 penetrationPoint;
    bool colliding;
} Manifold;

typedef struct {
    Vector2 vertex;
    float penetrationDepth;
    bool valid;
} SupportPoint;

typedef struct {
    Rigidbody* rigidbody;
    Vector2 point;
} Anchor;

float CalculatePolygonInertia(Vector2 *vertices, int vertexCount, float mass);
void TransformPoints(Rigidbody *rb);

Rigidbody CreateBody(Vector2 *vertices, int count, Vector2 position, float mass);
void UpdateBody(Rigidbody *rb, float dt);
void DestroyBody(Rigidbody rigidbody);

SupportPoint FindSupportPoint(Vector2 normalOnEdge, Vector2 pointOnEdge, Vector2* otherPolygonVertices, int vertexCount);
Manifold GetContactPoint(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB);
Manifold CheckCollisions(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB);
void ResolveCollision(Manifold manifold, Rigidbody* rigidbodyA, Rigidbody* rigidbodyB);

void AddForce(Rigidbody* body, Vector2 force);
void AddTorque(Rigidbody* body, float torque);
void ApplyForceAtPoint(Rigidbody* body, Vector2 force, Vector2 point);

Anchor CreateAnchor(Rigidbody* rigidbody, Vector2 point);
Vector2 AnchorPosition(Anchor anchor);

#ifdef FPHYSICS_IMPLEMENTATION

float CalculatePolygonInertia(Vector2 *vertices, int vertexCount, float mass) {
    float inertia = 0;
    float area = 0;

    for (int i = 0; i < vertexCount; i++) {
        Vector2 v1 = vertices[i];
        Vector2 v2 = vertices[(i + 1) % vertexCount];
        float cross = Vector2CrossProduct(v1, v2);
        area += cross;
        inertia += cross * (Vector2DotProduct(v1, v1) + Vector2DotProduct(v2, v2) + Vector2DotProduct(v1, v2));
    }

    inertia *= (mass / 3.0f) / area;

    return inertia;
}

void TransformPoints(Rigidbody *rb) {
    float cosTheta = cosf(rb->rotation);
    float sinTheta = sinf(rb->rotation);

    for (int i = 0; i < rb->vertexCount; i++) {
        Vector2 v = rb->vertices[i];
        rb->transformedVertices[i] = (Vector2){
            v.x * cosTheta - v.y * sinTheta + rb->position.x,
            v.x * sinTheta + v.y * cosTheta + rb->position.y
        };

        Vector2 n = rb->normals[i];
        rb->transformedNormals[i] = (Vector2){
            n.x * cosTheta - n.y * sinTheta,
            n.x * sinTheta + n.y * cosTheta
        };
    }
}

Rigidbody CreateBody(Vector2 *vertices, int count, Vector2 position, float mass) {
    Rigidbody rb = { 0 };
    rb.position = position;
    rb.velocity = (Vector2){0, 0};
    rb.rotation = 0;
    rb.angularVelocity = 0;
    rb.mass = mass;
    if(mass != 0) {
        rb.invMass =  1.0f / mass;
    } else {
        rb.invMass = 0;
        rb.isKinematic = true;
    }
    rb.vertexCount = count;

    rb.vertices = (Vector2 *)MemAlloc(count * sizeof(Vector2));
    rb.normals = (Vector2 *)MemAlloc(count * sizeof(Vector2));
    rb.transformedVertices = (Vector2 *)MemAlloc(count * sizeof(Vector2));
    rb.transformedNormals = (Vector2 *)MemAlloc(count * sizeof(Vector2));

    rb.restitution = 0.1f;
    rb.friction = 0.05f;

    for (int i = 0; i < count; i++) {
        rb.vertices[i] = vertices[i];
    }

    for (int i = 0; i < count; i++) {
        Vector2 edge = Vector2Subtract(rb.vertices[i], rb.vertices[(i + 1) % count]);
        rb.normals[i] = Vector2Normalize((Vector2){edge.y, -edge.x});
    }

    rb.inertia = CalculatePolygonInertia(rb.vertices, rb.vertexCount, rb.mass);
    rb.invInertia = (rb.inertia != 0) ? 1.0f / rb.inertia : 0;

    TransformPoints(&rb);

    return rb;
}

void UpdateBody(Rigidbody *rb, float dt) {
    rb->position = Vector2Add(rb->position, Vector2Scale(rb->velocity, dt));
    rb->rotation += rb->angularVelocity * dt;
    TransformPoints(rb);
}

void DestroyBody(Rigidbody rigidbody) {
    MemFree(rigidbody.vertices);
    MemFree(rigidbody.normals);
    MemFree(rigidbody.transformedVertices);
    MemFree(rigidbody.transformedNormals);
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

Manifold GetContactPoint(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB) {
    Manifold contact = { 0 };
    float minimumPenetration = 1000000.0f;

    for (int i = 0; i < rigidbodyA->vertexCount; i++) {
        Vector2 pointOnEdge = rigidbodyA->transformedVertices[i];
        Vector2 normalOnEdge = rigidbodyA->transformedNormals[i];

        SupportPoint supportPoint = FindSupportPoint(normalOnEdge, pointOnEdge, rigidbodyB->transformedVertices, rigidbodyB->vertexCount);
        if(!supportPoint.valid) return contact;

        if (supportPoint.penetrationDepth < minimumPenetration) {
            minimumPenetration = supportPoint.penetrationDepth;

            contact.depth = minimumPenetration;
            contact.normal = normalOnEdge;
            contact.penetrationPoint = supportPoint.vertex;
        }
    }

    contact.colliding = true;
    return contact;
}

Manifold CheckCollisions(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB) {
    Manifold result = { 0 };

    Manifold contactPolyA = GetContactPoint(rigidbodyA, rigidbodyB);
    if (!contactPolyA.colliding) return result;

    Manifold contactPolyB = GetContactPoint(rigidbodyB, rigidbodyA);
    if (!contactPolyB.colliding) return result;

    if (contactPolyA.depth < contactPolyB.depth) {
        Vector2 minus = Vector2Scale(contactPolyA.normal, contactPolyA.depth);
        result.depth = contactPolyA.depth;
        result.normal = contactPolyA.normal;
        result.penetrationPoint = Vector2Subtract(contactPolyA.penetrationPoint, minus);
    } else {
        result.depth = contactPolyB.depth;
        result.normal = Vector2Scale(contactPolyB.normal, -1);
        result.penetrationPoint = contactPolyB.penetrationPoint;
    }

    result.colliding = true;
    return result;
}

void ResolveCollision(Manifold manifold, Rigidbody* rigidbodyA, Rigidbody* rigidbodyB) {
    if (rigidbodyA->isKinematic && rigidbodyB->isKinematic) return;

    Vector2 penetrationToCentroidA = Vector2Subtract(manifold.penetrationPoint, rigidbodyA->position);
    Vector2 penetrationToCentroidB = Vector2Subtract(manifold.penetrationPoint, rigidbodyB->position);

    Vector2 angularVelocityPenetrationCentroidA = (Vector2){
        -1 * rigidbodyA->angularVelocity * penetrationToCentroidA.y,
        rigidbodyA->angularVelocity * penetrationToCentroidA.x
    };

    Vector2 angularVelocityPenetrationCentroidB = (Vector2){
        -1 * rigidbodyB->angularVelocity * penetrationToCentroidB.y,
        rigidbodyB->angularVelocity * penetrationToCentroidB.x
    };

    Vector2 relativeVelocityA = Vector2Add(rigidbodyA->velocity, angularVelocityPenetrationCentroidA);
    Vector2 relativeVelocityB = Vector2Add(rigidbodyB->velocity, angularVelocityPenetrationCentroidB);

    Vector2 relativeVel = Vector2Subtract(relativeVelocityB, relativeVelocityA);
    float velocityInNormal = Vector2DotProduct(relativeVel, manifold.normal);

    if (velocityInNormal > 0) return;

    float e = fmin(rigidbodyA->restitution, rigidbodyB->restitution);
    float pToCentroidCrossNormalA = Vector2CrossProduct(penetrationToCentroidA, manifold.normal);
    float pToCentroidCrossNormalB = Vector2CrossProduct(penetrationToCentroidB, manifold.normal);

    float invMassSum = rigidbodyA->invMass + rigidbodyB->invMass;

    float rigiAInvInertia = rigidbodyA->invInertia;
    float rigiBInvInertia = rigidbodyB->invInertia;
    float crossNSum = pToCentroidCrossNormalA * pToCentroidCrossNormalA * rigiAInvInertia + pToCentroidCrossNormalB * pToCentroidCrossNormalB * rigiBInvInertia;

    float j = -(1 + e) * velocityInNormal;
    j /= (invMassSum + crossNSum);

    Vector2 impulseVector = Vector2Scale(manifold.normal, j);

    rigidbodyA->velocity = Vector2Subtract(rigidbodyA->velocity, Vector2Scale(impulseVector, rigidbodyA->invMass));
    rigidbodyB->velocity = Vector2Add(rigidbodyB->velocity, Vector2Scale(impulseVector, rigidbodyB->invMass));
    rigidbodyA->angularVelocity += -pToCentroidCrossNormalA * j * rigiAInvInertia;
    rigidbodyB->angularVelocity += pToCentroidCrossNormalB * j * rigiBInvInertia;

    // Frictional impulse
    Vector2 velocityInNormalDirection = Vector2Scale(manifold.normal, Vector2DotProduct(relativeVel, manifold.normal));
    Vector2 tangent = Vector2Subtract(relativeVel, velocityInNormalDirection);
    tangent = Vector2Scale(tangent, -1);
    float minFriction = fmin(rigidbodyA->friction, rigidbodyB->friction);
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

    rigidbodyA->velocity = Vector2Subtract(rigidbodyA->velocity, Vector2Scale(frictionalImpulseVector, rigidbodyA->invMass));
    rigidbodyB->velocity = Vector2Add(rigidbodyB->velocity, Vector2Scale(frictionalImpulseVector, rigidbodyB->invMass));

    rigidbodyA->angularVelocity += -pToCentroidCrossTangentA * frictionalImpulse * rigiAInvInertia;
    rigidbodyB->angularVelocity += pToCentroidCrossTangentB * frictionalImpulse * rigiBInvInertia;

    float correctionPercentage = 0.9f;
    float amountToCorrect = (manifold.depth / (rigidbodyA->invMass + rigidbodyB->invMass)) * correctionPercentage;
    Vector2 correctionVector = Vector2Scale(manifold.normal, amountToCorrect);

    Vector2 rigiAMovement = Vector2Scale(correctionVector, rigidbodyA->invMass * -1);
    Vector2 rigiBMovement = Vector2Scale(correctionVector, rigidbodyB->invMass);

    rigidbodyA->position = Vector2Add(rigidbodyA->position, rigiAMovement);
    rigidbodyB->position = Vector2Add(rigidbodyB->position, rigiBMovement);

    TransformPoints(rigidbodyA);
    TransformPoints(rigidbodyB);
}

void AddForce(Rigidbody* body, Vector2 force) {
    body->velocity = Vector2Add(body->velocity, Vector2Scale(force, body->invMass));
}

void ApplyForceAtPoint(Rigidbody* body, Vector2 force, Vector2 point) {
    Vector2 r = Vector2Subtract(point, body->position);
    float torque = Vector2CrossProduct(r, force);

    body->angularVelocity += torque * body->invInertia;
    body->velocity = Vector2Add(body->velocity, Vector2Scale(force, body->invMass));
}

void AddTorque(Rigidbody* body, float torque) {
    body->angularVelocity += torque * body->invInertia;
}

Anchor CreateAnchor(Rigidbody* rigidbody, Vector2 point) {
    Vector2 p1 = rigidbody->transformedVertices[0];
    Vector2 p2 = rigidbody->transformedVertices[1];
    Vector2 d = Vector2Subtract(p2, p1);
    Vector2 dPerp = (Vector2){-d.y, d.x};
    Vector2 dp3 = Vector2Subtract(point, p1);

    float denom = d.x * d.x + d.y * d.y;
    float x = (dp3.x * d.x + dp3.y * d.y) / denom;
    float y = (dp3.x * dPerp.x + dp3.y * dPerp.y) / denom;

    Anchor anchor = { .point = {x, y}, .rigidbody = rigidbody };
    return anchor;
}

Vector2 AnchorPosition(Anchor anchor) {
    Vector2 p1 = anchor.rigidbody->transformedVertices[0];
    Vector2 p2 = anchor.rigidbody->transformedVertices[1];
    Vector2 d = Vector2Subtract(p2, p1);
    Vector2 dPerp = (Vector2){-d.y, d.x};

    return Vector2Add(p1, Vector2Add(Vector2Scale(d, anchor.point.x), Vector2Scale(dPerp, anchor.point.y)));
}

#endif
#endif