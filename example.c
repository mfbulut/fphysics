#include "raylib.h"
#include "raymath.h"

#define FPHYSICS_IMPLEMENTATION
#include "physics.h"

Rigidbody rigidBodies[256] = { 0 };
int rigidBodyCount = 0;
Anchor anchor = { 0 };

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 720, "Rigidbody System");

    rigidBodies[rigidBodyCount++] = CreateBody((Vector2[]){{0, 25}, {1280, 25}, {1280, -25}, {0, -25}}, 4, (Vector2){0, 600}, 0);
    Vector2 boxVerts[4] = {{-25, 25} , {25, 25},{25, -25}, {-25, -25}};
    for (int i = 0; i < 16; i++) {
        rigidBodies[rigidBodyCount++] = CreateBody(boxVerts, 4, (Vector2){GetRandomValue(200, 800), GetRandomValue(0, 500)}, 1.0f);
    }

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            for (int i = 0; i < rigidBodyCount; i++) {
                Vector2 mousePos = GetMousePosition();
                if(CheckCollisionPointPoly(mousePos, rigidBodies[i].transformedVertices, rigidBodies[i].vertexCount)) {
                    anchor = CreateAnchor(&rigidBodies[i], mousePos);
                }
            }
        }

        if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            anchor.rigidbody = 0;
        }

        if(anchor.rigidbody) {
            Vector2 anchorPos = AnchorPosition(anchor);
            Vector2 mouseForce = Vector2Scale(Vector2Subtract(GetMousePosition(), anchorPos), 20.0f * dt);
            ApplyForceAtPoint(anchor.rigidbody, mouseForce, anchorPos);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            AddForce(&rigidBodies[i], (Vector2){0, 500 * dt * rigidBodies[i].mass});
            UpdateBody(&rigidBodies[i], dt);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            for (int j = i + 1; j < rigidBodyCount; j++) {
                Manifold manifold = CheckCollisions(&rigidBodies[i], &rigidBodies[j]);
                if (manifold.colliding) {
                    ResolveCollision(manifold, &rigidBodies[i], &rigidBodies[j]);
                }
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);

        for (int i = 0; i < rigidBodyCount; i++) {
            DrawTriangleFan(rigidBodies[i].transformedVertices, rigidBodies[i].vertexCount, DARKGRAY);
            for (int j = 0; j < rigidBodies[i].vertexCount; j++) {
                Vector2 v1 = rigidBodies[i].transformedVertices[j];
                Vector2 v2 = rigidBodies[i].transformedVertices[(j + 1) % rigidBodies[i].vertexCount];
                DrawLineV(v1, v2, WHITE);
            }
        }

        if(anchor.rigidbody) {
            DrawLineV(GetMousePosition(), AnchorPosition(anchor), RED);
            DrawCircleV(AnchorPosition(anchor), 5, RED);
        }

        EndDrawing();
    }

    for (int i = 0; i < rigidBodyCount; i++) {
        DestroyBody(rigidBodies[i]);
    }

    CloseWindow();
}