#include "raylib.h"
#include <cmath>
#include <bits/stdc++.h>
#include <random>
#include <vector>

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------

float random_float(float min, float max) {
    // Seed with a real random value, if available
    std::random_device rd;

    // Create a random number generator
    std::mt19937 gen(rd());

    // Define a distribution in the range [min, max]
    std::uniform_real_distribution<> dis(min, max);

    // Generate a random float
    return dis(gen);
}


typedef struct transform2D
{
    float x;/* data */
    float y;
    float w;
}transform2D;



class Agent
{
    private:
        transform2D transform;
        transform2D speed;

        int ballRadius;

        /* data */
    public:
        Agent(transform2D initial_transform, int ballRadius, transform2D speed);
        ~Agent();
        void Update();
        void FrameUpdate();
        void RandomMoveBehavior();
        void Translate(float x, float y);
    };

    Agent::Agent(transform2D initial_transform = transform2D(), int ballRadius = 20, transform2D speed = transform2D())
    {
        this->ballRadius = ballRadius;

        this->transform = initial_transform;

        this->speed = speed;
    }

    //TODO : do the update with speed&behaviors.
    void Agent::Update(){
        //todo : choose behavior  
        this->RandomMoveBehavior(); 
    }

    void Agent::RandomMoveBehavior(){
        this->transform.w += random_float(-this->speed.w, this->speed.w); //random angular speed in rad.

        //todo : réparer les conneries
        float translation_x = this->speed.x * std::cos(this->transform.w);
        float translation_y = this->speed.x * std::sin(this->transform.w);

        this->Translate(translation_x, translation_y);
    }

    void Agent::Translate(float x, float y){
        //transform update through speeds
        this->transform.x += x;
        this->transform.y += y;
        //this->transform.w += this->speed.w;

        // check that we're still in the env.
        this->transform.x = std::min(float(GetScreenWidth() - this->ballRadius), this->transform.x);
        this->transform.x = std::max(float(this->ballRadius), this->transform.x );
        this->transform.y = std::min(float(GetScreenHeight() - this->ballRadius), this->transform.y);
        this->transform.y = std::max(float(this->ballRadius), this->transform.y );
    }

    void Agent::FrameUpdate(){
        DrawCircleV(Vector2({this->transform.x, this->transform.y}), (float)this->ballRadius, MAROON);
    }


    Agent::~Agent()
    {
    }



int main(void)
{
    // Initialization
    //---------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    const int NBAGENTS = 2000;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "raylib [shapes] example - bouncing ball");

    //agent creation


    transform2D initial_tf = {GetScreenWidth()/2.0f, GetScreenHeight()/2.0f, 0.0 };
    transform2D ballSpeed = { 3.0f, 0.0f, 0.5};

    int ballRadius = 10;

    std::vector<Agent*> agents = {};


    for(int i=0; i < NBAGENTS; i++){
        float xpos = random_float(0,screenWidth);
        float ypos = random_float(0,screenHeight);
        agents.insert(agents.end(), new Agent(transform2D({xpos, ypos}), ballRadius, ballSpeed));
        std::cout<< xpos << "|";
        std::cout<< ypos << "||\n";
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
    CloseWindow();        // Close window and OpenGL context
    //----------------------------------------------------------

    return 0;
}