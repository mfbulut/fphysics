#include "raylib.h"
#include "raymath.h"

#define FPHYSICS_IMPLEMENTATION
#include "fphysics.h"

const int screenWidth = 1280;
const int screenHeight = 720;

#define MAX_RIGIDBODIES 256
int rigidBodyCount;
Rigidbody rigidBodies[MAX_RIGIDBODIES];

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Physics Simulation");

    PhysicsMaterial material = (PhysicsMaterial){ 0.5f, 0.05f };
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreatePolygon((Vector2[]){(Vector2){300, 300}, (Vector2){400, 300}, (Vector2){300, 400}}, 3, GREEN), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){500, 200}, 200, 50 , WHITE), 1, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){700, 300}, 100, 100, WHITE), 1, material);

    // Borders
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){screenWidth / 2, -50}, screenWidth, 100, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){screenWidth / 2, screenHeight + 50}, screenWidth, 100, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){-50, screenHeight / 2}, 100, screenHeight, WHITE), 0, material);
    rigidBodies[rigidBodyCount++] = CreateRigidbody(CreateRectangle((Vector2){screenWidth + 50, screenHeight / 2}, 100, screenHeight, WHITE), 0, material);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        const float force = 1000;

        if(IsKeyDown(KEY_W)) AddForce(&rigidBodies[0], (Vector2){0, -force});
        if(IsKeyDown(KEY_A)) AddForce(&rigidBodies[0], (Vector2){-force, 0});
        if(IsKeyDown(KEY_S)) AddForce(&rigidBodies[0], (Vector2){0, force});
        if(IsKeyDown(KEY_D)) AddForce(&rigidBodies[0], (Vector2){force, 0});

        if(IsKeyDown(KEY_Q)) rigidBodies[0].angularVelocity += 0.1f;
        if(IsKeyDown(KEY_E)) rigidBodies[0].angularVelocity -= 0.1f;

        for (int i = 0; i < rigidBodyCount; i++) {
            Rigidbody* rb = &rigidBodies[i];
            AddForce(rb, Vector2Scale((Vector2){0, 250}, rb->mass));
            UpdateRigidbody(rb, dt);
        }

        for (int i = 0; i < rigidBodyCount; i++) {
            for (int j = i + 1; j < rigidBodyCount; j++) {
                HandleCollision(&rigidBodies[i], &rigidBodies[j]);
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);

        for (int i = 0; i < rigidBodyCount; i++) {
            DrawPolygon(rigidBodies[i].polygon);
        }

        EndDrawing();
    }

    CloseWindow();
}



