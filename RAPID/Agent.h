#pragma once
#include "raylib.h"
#include "utils.h"

using namespace structures;

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