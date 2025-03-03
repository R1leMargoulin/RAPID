
#include "raylib.h"
#include "utils.h"
#include "Agent.h"

#include <cmath>
#include <vector>

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------

using namespace structures;



int main(void)
{
    // Initialization
    //---------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    const int NBAGENTS = 200;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "raylib [shapes] example - bouncing ball");

    //agent creation


    transform2D initial_tf = {GetScreenWidth()/2.0f, GetScreenHeight()/2.0f, 0.0 };
    transform2D ballSpeed = { 3.0f, 0.0f, 0.5};

    int ballRadius = 10;

    std::vector<Agent*> agents = {};


    for(int i=0; i < NBAGENTS; i++){
        float xpos = math::random_float(0,screenWidth);
        float ypos = math::random_float(0,screenHeight);
        agents.insert(agents.end(), new Agent(transform2D({xpos, ypos}), ballRadius, ballSpeed));
    }
    

    bool pause = 0;
    int framesCounter = 0;

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //----------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //-----------------------------------------------------
        if (IsKeyPressed(KEY_SPACE)) pause = !pause;

        if (!pause)
        {
            for(int a = 0; a<NBAGENTS ; a++){
                agents[a]->Update();
            }
        }
        else framesCounter++;
        //-----------------------------------------------------

        // Draw
        //-----------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);


            for(int a = 0; a < NBAGENTS ; a++){
                agents[a]->FrameUpdate();
            }

            DrawText("PRESS SPACE to PAUSE BALL MOVEMENT", 10, GetScreenHeight() - 25, 20, LIGHTGRAY);

            // On pause, we draw a blinking message
            if (pause && ((framesCounter/30)%2)) DrawText("PAUSED", 350, 200, 30, GRAY);

            DrawFPS(10, 10);

        EndDrawing();
        //-----------------------------------------------------
    }

    // De-Initialization
    //---------------------------------------------------------
    for(int i=0; i < NBAGENTS; i++){
        free(agents[i]);
    }
    CloseWindow();        // Close window and OpenGL context
    //----------------------------------------------------------

    return 0;
}