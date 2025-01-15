#include "raylib.h"
#include "raymath.h"

#define FPHYSICS_IMPLEMENTATION
#include "fphysics.h"
#include "joint.h"

const int width = 1280, height = 720;

Rigidbody rigidBodies[256];
int rigidBodyCount = 0;

Joint joints[32];
int jointCount = 0;

Anchor anchor = { 0 };
Vector2 pentagonVert[] = {(Vector2){176,-125}, (Vector2){100,-180}, (Vector2){24,-125}, (Vector2){53,-35}, (Vector2){147,-35}};

// #define DRAW_NORMALS

void InitGame() {
    PhysicsMaterial material = {0.1f, 2.0f};

    rigidBodies[0] = CreateRigidbody(CreateRectangle((Vector2){0, 700}, 4000, 100), 0, material);
    rigidBodies[1] = CreateRigidbody(CreateRectangle((Vector2){500, 100}, 50, 50), 1, material);
    rigidBodies[2] = CreateRigidbody(CreateRectangle((Vector2){500, 100}, 50, 50), 1, material);
    rigidBodies[3] = CreateRigidbody(CreateRectangle((Vector2){600, 100}, 50, 50), 1, material);
    rigidBodies[4] = CreateRigidbody(CreateRectangle((Vector2){600, 100}, 50, 50), 1, material);
    rigidBodies[5] = CreateRigidbody(CreateRectangle((Vector2){600, 100}, 50, 50), 1, material);
    rigidBodies[6] = CreateRigidbody(CreatePolygon(pentagonVert, 5), 1, material);
    rigidBodyCount = 7;

    int bridgePlanks = 15;
    float plankWidth = 50;
    float plankHeight = 10;
    Vector2 startPos = { 300, 200 };

    for (int i = 0; i < bridgePlanks; i++) {
        Vector2 plankPos = { startPos.x + i * plankWidth, startPos.y };
        rigidBodies[rigidBodyCount] = CreateRigidbody(CreateRectangle(plankPos, plankWidth, plankHeight), 5, material);

        if (i > 0) {
            joints[jointCount] = CreateJoint(
                CreateAnchor(&rigidBodies[rigidBodyCount - 1], (Vector2){ plankPos.x - plankWidth / 2, plankPos.y }),
                CreateAnchor(&rigidBodies[rigidBodyCount], (Vector2){ plankPos.x - plankWidth / 2, plankPos.y }),
                FORCE_JOINT
            );
            jointCount++;
        }

        rigidBodyCount++;
    }

    joints[jointCount++] = CreateJoint(CreateAnchor(&rigidBodies[0], startPos), CreateAnchor(&rigidBodies[rigidBodyCount - bridgePlanks], (Vector2){ startPos.x, startPos.y }), FORCE_JOINT);
    joints[jointCount++] = CreateJoint(CreateAnchor(&rigidBodies[0], (Vector2){ startPos.x + (bridgePlanks - 1) * plankWidth, startPos.y }), CreateAnchor(&rigidBodies[rigidBodyCount - 1], (Vector2){ startPos.x + (bridgePlanks - 1) * plankWidth, startPos.y }), FORCE_JOINT);
}

void UpdateGame(float deltaTime) {
    if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        for (int i = 0; i < rigidBodyCount; i++) {
            Vector2 mousePos = GetMousePosition();
            if(CheckCollisionPointPoly(mousePos, rigidBodies[i].polygon->vertices, rigidBodies[i].polygon->vertexCount)) {
                anchor = CreateAnchor(&rigidBodies[i], mousePos);
            }
        }
    }

    if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        anchor.rigidbody = 0;
    }

    if(anchor.rigidbody) {
        Vector2 anchorPos = AnchorPosition(anchor);
        Vector2 mouseForce = Vector2Scale(Vector2Subtract(GetMousePosition(), anchorPos), 50.0f * anchor.rigidbody->mass);
        ApplyForceAtPoint(anchor.rigidbody, mouseForce, anchorPos);
    }

    const int subStepCount = 5;
    float sdt = deltaTime / subStepCount;

    for (int i = 0; i < subStepCount; i++) {
        for (int i = 0; i < rigidBodyCount; i++) {
            Rigidbody* rb = &rigidBodies[i];
            AddForce(rb, Vector2Scale((Vector2){0, 800}, rb->mass));
            UpdateRigidbody(rb, sdt);
        }

        for (int i = 0; i < jointCount; i++) {
            UpdateJoint(&joints[i], sdt);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            for (int j = i + 1; j < rigidBodyCount; j++) {
                HandleCollision(&rigidBodies[i], &rigidBodies[j]);
            }
        }
    }
}

void DrawGame() {
    for (int i = 0; i < rigidBodyCount; i++) {
        Polygon* polygon = rigidBodies[i].polygon;
        DrawTriangleFan(polygon->vertices, polygon->vertexCount, DARKGRAY);

        for (int i = 0; i < polygon->vertexCount; i++) {
            DrawLineV(polygon->vertices[i], polygon->vertices[(i + 1) % polygon->vertexCount], WHITE);

            #ifdef DRAW_NORMALS
            Vector2 midpoint = Vector2Scale(Vector2Add(polygon->vertices[i], polygon->vertices[(i + 1) % polygon->vertexCount]), 0.5f);
            Vector2 endpoint = Vector2Add(midpoint, Vector2Scale(polygon->normals[i], 10));
            DrawLineV(midpoint, endpoint, GREEN);
            #endif
        }
    }

    if(anchor.rigidbody) {
        DrawLineV(GetMousePosition(), AnchorPosition(anchor), RED);
        DrawCircleV(AnchorPosition(anchor), 5, RED);
    }

    for (int i = 0; i < jointCount; i++) {
        DrawLineV(AnchorPosition(joints[i].anchorA), AnchorPosition(joints[i].anchorB), BLUE);
    }
}

int main(void) {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(width, height, "Physics Demo");

    InitGame();

    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();
        UpdateGame(deltaTime);
        BeginDrawing();
        ClearBackground(BLACK);
        DrawGame();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}