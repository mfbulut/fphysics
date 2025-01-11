#include "raylib.h"
#include "raymath.h"

#define FPHYSICS_IMPLEMENTATION
#include "fphysics.h"

Rigidbody rigidBodies[256];
int rigidBodyCount;

int width = 1280, height = 720;

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(width, height, "Physics Simulation");

    PhysicsMaterial material = {0.5f, 0.05f};

    // Boundary
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){width / 2, -50}, width, 100), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){width / 2, height + 50}, width, 100), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){-50, height / 2}, 100, height), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){width + 50, height / 2}, 100, height), 0, material);

    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){300, 200}, 100, 100), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){500, 300}, 100, 100), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){700, 300}, 100, 100), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){500, 500}, 100, 100), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){700, 500}, 100, 100), 1, material);

    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreatePolygon((Vector2[]){(Vector2){600, 100}, (Vector2){700, 200}, (Vector2){500, 200}}, 3), 1, material);

    Anchor anchor = { 0 };

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

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
            Polygon* polygon = rigidBodies[i].polygon;
            for (int i = 0; i < polygon->vertexCount; i++) {
                DrawLineV(polygon->vertices[i], polygon->vertices[(i + 1) % polygon->vertexCount], WHITE);
            }
        }

        EndDrawing();
    }

    CloseWindow();
}