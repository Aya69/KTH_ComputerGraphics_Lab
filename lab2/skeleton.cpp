//  LAB2 skeleton.cpp
//members: Anna Egorova <aegorova@kth.se> ID 0626 3000; Sheetal Borar <borar@kth.se> ID 0624 9020
#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <math.h>
//#include <algorithm>

#define PI 3.14159265
using namespace std;
using glm::vec3;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
vector<Triangle> triangles;

float focalLength =500;
vec3 cameraPos(0,0,-3);
mat3 R;
float yaw=0;
vec3 lightPos( 0, -0.5, -0.7 );
vec3 lightColor = 14.f * vec3( 1, 1, 1 );
vec3 indirectLight = 0.5f*vec3( 1, 1, 1 );

// ----------------------------------------------------------------------------
// STRUCTS
struct Intersection
{
    vec3 position;
    float distance;
    int triangleIndex;
};

// ----------------------------------------------------------------------------
// FUNCTION DECLARATIONS

void Update();
void Draw();
bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles,Intersection& closestIntersection);
void rotation(float);
vec3 DirectLight( const Intersection& i );

//---------------------------------------------------

int main( int argc, char* argv[] )
{
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    t = SDL_GetTicks();	// Set start value for timer.
    // Animation loop, it renders every frame
    while( NoQuitMessageSDL() )
    {
        Update(); // Change contents of frame
        Draw();   // Calls putPixel to fill the screen
    }
    // Take a screenshot of what is shown in the window
    SDL_SaveBMP( screen, "screenshot.bmp" );
    return 0;
}

// Used to rotate in Y when the user presses the right and left keys.
void rotation(float yaw){
    // Rotation matrix for rotation in Y
    R = mat3(cos(yaw)   , 0 , sin(yaw),
             0          , 1 , 0,
             -sin(yaw)  , 0 , cos(yaw));
}

// Update
void Update()
{
    // Compute frame time:
    int t2 = SDL_GetTicks();
    float dt = float(t2-t);
    t = t2;
    cout << "Render time: " << dt << " ms." << endl;
    // Handle key precess, if the key is pressed modify a variable.
    Uint8* keystate = SDL_GetKeyState( 0 );
    if( keystate[SDLK_UP] ){
        cameraPos.y=cameraPos.y+2/focalLength;//change cameras Y position to move up
    }
    if( keystate[SDLK_DOWN] ){
        cameraPos.y=cameraPos.y-2/focalLength;////change cameras Y position to move down
        
    }
    if( keystate[SDLK_LEFT] ){
        yaw=yaw -(5*PI/180);//rotate the around Y axis to change camera angle
        rotation(yaw);
        cameraPos=R*cameraPos;
    }
    if( keystate[SDLK_RIGHT] ){
        yaw=yaw +(5*PI/180); //rotate the around Y axis to change camera angle
        
        rotation(yaw);
        cameraPos=R*cameraPos;
    }
    //change light position to move light around
    if( keystate[SDLK_w] ){
        lightPos.z = lightPos.z + 2/focalLength;
    }
    if( keystate[SDLK_s] ){
        lightPos.z = lightPos.z - 2/focalLength;
    }
    if( keystate[SDLK_a] ){
        lightPos.x = lightPos.x + 2/focalLength;
    }
    if( keystate[SDLK_d] ){
        lightPos.x = lightPos.x - 2/focalLength;
    }
    if( keystate[SDLK_q] ){
        lightPos.y = lightPos.y + 2/focalLength;
    }
    if( keystate[SDLK_e] ){
        lightPos.y = lightPos.y - 2/focalLength;
    }
}

//max number function
float max(float a, float b){
    if(a>=b){
        return a;
    }
    else if(b>a){
        return b;
    }
}

//direct illumination
vec3 DirectLight( const Intersection& i )
{
    //normal calculations
    triangles[i.triangleIndex].ComputeNormal();
    vec3 normal = triangles[i.triangleIndex].normal;
    float nMagnitude = sqrt(pow (normal.x,2) + pow (normal.y,2) + pow (normal.z,2));
    vec3 nUnitVector = normal/nMagnitude;
    
    //light ray vector
    vec3 rVector = lightPos-i.position;
    float rMagnitude = sqrt(pow (rVector.x,2) + pow (rVector.y,2) + pow (rVector.z,2));
    vec3 rUnitVector = rVector/rMagnitude;
    
    //power calculation for each point
    float surfaceArea = 4 * PI * pow(rMagnitude,2);
    vec3 powerPerArea = lightColor / surfaceArea;
    float dotProduct = (rUnitVector.x * nUnitVector.x + rUnitVector.y * nUnitVector.y + rUnitVector.z * nUnitVector.z);
    vec3 directIllumination = powerPerArea * max(0, dotProduct);
    
    //shadows
    Intersection closestIntersectionRay;//from light to each 3d point
    ClosestIntersection(lightPos, -rUnitVector, triangles, closestIntersectionRay);
    if(glm::length(closestIntersectionRay.position-lightPos)+0.00001 < rMagnitude){
        return vec3(0,0,0);
    }
    else{
        return directIllumination;
    }
}

//inderict illumination calculation
vec3 inDirectLight( const Intersection& i )
{
    vec3 indirectIllumination = indirectLight ;
    return indirectIllumination;
}

//closest intersection
bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles,Intersection& closestIntersection)
{
    bool result=false;
    float m=std::numeric_limits<float>::max();
    closestIntersection.distance=m;
    // For each triangle in the model
    for(int i = 0; i < triangles.size(); i++)
    {
        vec3 v0 = triangles[i].v0;
        vec3 v1 = triangles[i].v1;
        vec3 v2 = triangles[i].v2;
        
        //find the intersection point between the ray of camera to that point and the plane of that triangle
        vec3 e1 = v1 - v0;
        vec3 e2 = v2 - v0;
        vec3 b = start - v0;
        mat3 A( -dir, e1, e2 );
        vec3 s = glm::inverse( A ) * b;
        
        if((s.x >= 0) && (s.y >= 0) && (s.z >= 0) && ((s.z+s.y) <= 1)){//check if the intersection is in the triangle
            result=true;
            vec3 position = start + dir * s.x;
            if(s.x < closestIntersection.distance){//see if the intersection is closest to the camera
                closestIntersection.position = position;
                closestIntersection.distance =  s.x;
                closestIntersection.triangleIndex = i;
            }
        }
    }
    return result;
}

void Draw()
{
    if( SDL_MUSTLOCK(screen) )
        SDL_LockSurface(screen);
    LoadTestModel( triangles );
    
    
    Intersection closestIntersection;
    // For each pixel on the screen
    for( int u=0; u<SCREEN_HEIGHT; ++u )
    {
        for( int v=0; v<SCREEN_WIDTH; ++v )
        {
            // Cast a ray from the pixel and see if it intersects with anything
            vec3 d = vec3(u-SCREEN_WIDTH/2, v-SCREEN_HEIGHT/2, focalLength);
            bool intersectionExists=ClosestIntersection(cameraPos, d, triangles, closestIntersection);
            vec3 color(0,0,0);
            // If it does intersect with something then draw it on the screen.
            if(intersectionExists){
                vec3 D = DirectLight(closestIntersection);
                vec3 N = inDirectLight(closestIntersection);
                color =  (D + N) * triangles[closestIntersection.triangleIndex].color ;//direct light +indirect light *color
            }
            PutPixelSDL( screen, u, v, color );
        }
    }
    
    if( SDL_MUSTLOCK(screen) )
        SDL_UnlockSurface(screen);
    
    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}
