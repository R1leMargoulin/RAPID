#include "Agent.h"

using namespace structures;

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
        this->transform.w += math::random_float(-this->speed.w, this->speed.w); //random angular speed in rad.

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
