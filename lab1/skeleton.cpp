//  LAB1 skeleton.cpp
//members: Anna Egorova <aegorova@kth.se> ID 0626 3000; Sheetal Borar <borar@kth.se> ID 0624 9020


// Introduction lab that covers:
// * C++
// * SDL
// * 2D graphics
// * Plotting pixels
// * Video memory
// * Color representation
// * Linear interpolation
// * glm::vec3 and std::vector


////// BLOCK 1 : Include the relevant libraries
#include "SDL.h"
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include "SDLauxiliary.h"

using namespace std;
using glm::vec3;

///// BLOCK 2:  Initialize global variables
// --------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
SDL_Surface* screen;
vector<vec3> stars( 1000 );
int t;


////// BLOCK 3: functions
// --------------------------------------------------------
// FUNCTION DECLARATIONS

void Draw();
void Interpolate1( float a, float b, vector<float>& result );
void Interpolate( vec3 a, vec3 b, vector<vec3>& result );
void Update();

// --------------------------------------------------------
// FUNCTION DEFINITIONS

int main( int argc, char* argv[] )
{
    /*
     //simple interpolation
     vector<float> result( 1 ); // Create a vector width 10 floats
     Interpolate1( 5, 14, result ); // Fill it with interpolated values
     for( int i=0; i<result.size(); ++i )
     cout << result[i] << " "; // Print the result to the terminal
     */
    
    //3D interpolation
    
    /*
     vector<vec3> result( 4 );
     vec3 a(1,4,9.2);
     vec3 b(4,1,9.8);
     Interpolate( a, b, result );
     for( int i=0; i<result.size(); ++i )
     {
     cout << "( "
     << result[i].x << ", "
     << result[i].y << ", "
     << result[i].z << " ) ";
     }
     */
    
    // initialising the positions of the stars
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    t = SDL_GetTicks();
    for(int i=0;i<stars.size();i++){
        float x=float (rand()) /(float (RAND_MAX/(2)))-1;
        float y=float (rand()) /(float (RAND_MAX/(2)))-1;
        float z=float(rand()) / float(RAND_MAX);
        //cout<<stars[i].x<<" " <<stars[i].y<<" "<<stars[i].z;
        stars[i]=vec3(x,y,z);
    }
    
    while( NoQuitMessageSDL() )
    {
        Update();
        Draw();
    }
    SDL_SaveBMP( screen, "screenshot.bmp" );
    return 0;
    
}

void Interpolate1( float a, float b, vector<float>& result ) // Interpolating between float values
{
    float x=a;
    if(result.size()==1){
        result[0]=a;
    }
    else{
        float step = ((b-a)*1.0)/(result.size()-1);
        for(int i=0; i<=result.size() ; i++){
            result[i]=x;
            x = x + step;
        }
    }
}
//Changing the Interpolate() function to take 3D vector arguments and do 3D interpolation:
void Interpolate( vec3 a, vec3 b, vector<vec3>& result ){
    vec3 x=a;
    if(result.size()==1){
        result[0]=a;
    }
    else{
        int count = result.size()-1;
        vec3 step = (b-a)* (1.0f/count);
        for(int i=0; i<result.size() ; i++){
            result[i]=x;
            x = x + step;
        }
    }
}

void Draw()
{
    
    
    //Color the entire screen black to erase contents of previous frame
    SDL_FillRect( screen, 0, 0 );
    
    if( SDL_MUSTLOCK(screen) )
        SDL_LockSurface(screen);
    
    //Task 1 of Lab 1 Rainbow Color Interpolation
    
    /*vec3 topLeft(1,0,0); // red
     vec3 topRight(0,0,1); // blue
     vec3 bottomLeft(0,1,0); // green
     vec3 bottomRight(1,1,0); // yellow
     
     vector<vec3> leftSide( SCREEN_HEIGHT );
     vector<vec3> rightSide( SCREEN_HEIGHT );
     Interpolate( topLeft, bottomLeft, leftSide );
     Interpolate( topRight, bottomRight, rightSide );
     
     
     //setting already defined/interpolated colors to every pixel of the row:
     for( int y=0; y<SCREEN_HEIGHT; ++y )
     {
     vector<vec3> row(SCREEN_WIDTH);
     Interpolate( leftSide[y], rightSide[y], row );
     for( int x=0; x<SCREEN_WIDTH; ++x )
     {
     PutPixelSDL( screen, x, y, row[x] );
     }
     }*/
    
    //Then the stars are progected to the screen
    
    for( size_t s=0; s<stars.size(); ++s )
    {
        // Projecting and drawing each star
        int u = (SCREEN_HEIGHT/2*(stars[s].x/stars[s].z))+SCREEN_WIDTH/2;
        int v = (SCREEN_HEIGHT/2*(stars[s].y/stars[s].z))+SCREEN_HEIGHT/2;
        vec3 color = 0.2f * vec3(1,1,1) / (stars[s].z*stars[s].z);
        PutPixelSDL( screen, u, v, color );
    }
    if( SDL_MUSTLOCK(screen) )
        SDL_UnlockSurface(screen);
    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

void Update() //Update()function is counted the time passed before the last call and then use it to calculate the new position of the z coordinate for each star

{
    int t2 = SDL_GetTicks();
    float dt = float(t2-t);// how much time has passed
    t = t2;
    for( size_t s=0; s<stars.size(); ++s )
    {
        // Updating the position of stars to move them in z-axis over time
        stars[s].z=stars[s].z-0.0003*dt;
        // Keep stars within the bounds of the projection
        if( stars[s].z <= 0 )// If the stars are too close push them away
            stars[s].z += 1;
        if( stars[s].z > 1 ) // If the stars are too far away move them closer
            stars[s].z -= 1;
    }
}
