#include "raylib.h"
#include "raymath.h"

#define FPHYSICS_IMPLEMENTATION
#include "fphysics.h"

#define MAX_RIGIDBODIES 256
int rigidBodyCount;
Rigidbody rigidBodies[MAX_RIGIDBODIES];

const int screenWidth = 1280;
const int screenHeight = 720;

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Physics Simulation");

    PhysicsMaterial material;
    material.bounce = 0.5f;
    material.friction = 0.05f;

    // Boundary
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){screenWidth / 2, -50}, screenWidth, 100, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){screenWidth / 2, screenHeight + 50}, screenWidth, 100, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){-50, screenHeight / 2}, 100, screenHeight, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){screenWidth + 50, screenHeight / 2}, 100, screenHeight, WHITE), 0, material);

    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){400, 200}, 200, 100, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){200, 200}, 200, 50 , WHITE), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){200, 300}, 100, 100, WHITE), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreatePolygon((Vector2[]){(Vector2){0, 0}, (Vector2){100, 0}, (Vector2){0, 100}}, 3, WHITE), 1, material);

    Anchor anchor = { 0 };

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            for (int i = 0; i < rigidBodyCount; i++) {
                Vector2 mousePos = GetMousePosition();
                if(IsPointInside(rigidBodies[i].polygon, mousePos)) {
                    anchor = CreateAnchor(&rigidBodies[i], mousePos);
                }
            }
        }

        if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            anchor.rigidbody = 0;
        }

        if(anchor.rigidbody) {
            Vector2 anchorPos = AnchorPosition(anchor);
            Vector2 mouseForce = Vector2Scale(Vector2Subtract(GetMousePosition(), anchorPos), 5.0f);
            ApplyForceAtPoint(anchor.rigidbody, mouseForce, anchorPos);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            Rigidbody* rb = &rigidBodies[i];
            AddForce(rb, Vector2Scale((Vector2){0, 200}, rb->mass));
            UpdateRigidbody(rb, dt);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            for (int j = i + 1; j < rigidBodyCount; j++) {
                HandleCollision(&rigidBodies[i], &rigidBodies[j]);
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);

        if(anchor.rigidbody) {
            DrawLineV(GetMousePosition(), AnchorPosition(anchor), RED);
            DrawCircleV(AnchorPosition(anchor), 5, RED);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            DrawPolygon(rigidBodies[i].polygon);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}