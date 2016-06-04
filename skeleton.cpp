//LAB3 skeleton.cpp
//members: Anna Egorova <aegorova@kth.se> ID 0626 3000; Sheetal Borar <borar@kth.se> ID 0624 9020


#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <algorithm>
#define PI 3.14159265

using namespace std;
using glm::vec3;
using glm::ivec2;
using glm::mat3;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
vector<Triangle> triangles;
const int focal_length = 500;
vec3 cameraPos( 0, 0, -3.001 );
mat3 R;
float yaw=0;
vec3 currentColor;
struct Pixel
{
    int x;
    int y;
    float zinv;
    vec3 pos3d;
};
struct Vertex
{
    vec3 position;
};
const float delta = 0.01f;
vec3 currentNormal;
vec3 currentReflectance;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
vec3 lightPos(0,-0.5,-0.7);
vec3 lightPower = 10.1f*vec3( 1, 1, 1 );
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
//----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
void ComputePolygonRows(
                        const vector<Pixel>& vertexPixels,
                        vector<Pixel>& leftPixels,
                        vector<Pixel>& rightPixels
                        );
void DrawPolygonRows(
                     const vector<Pixel>& leftPixels,
                     const vector<Pixel>& rightPixels
                     );
void VertexShader( const Vertex& v, Pixel& p );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );

int main( int argc, char* argv[] )
{
    LoadTestModel( triangles ); //loads the cornell box in the triangles vector
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    t = SDL_GetTicks(); // Set start value for timer.
    
    while( NoQuitMessageSDL() )
    {
        Update();
        Draw();
    }
    
    SDL_SaveBMP( screen, "screenshot.bmp" );
    return 0;
}

//generate the rotation matrix
void rotation(float yaw){
    R[0][0] = glm::cos(yaw);
    R[2][0] = glm::sin(yaw);
    R[0][2] = -glm::sin(yaw);
    R[2][2] = glm::cos(yaw);
}

void Update()
{
    // Compute frame time:
    int t2 = SDL_GetTicks();
    float dt = float(t2-t);
    t = t2;
    cout << "Render time: " << dt << " ms." << endl;
    
    // How much to move every time a key is pressed
    const float STEP = 0.1f;
    const float ANGLE_STEP =  (5*PI/180);
    
    Uint8* keystate = SDL_GetKeyState(0);
    //change camera position to move front or back
    if( keystate[SDLK_UP] ){
        cameraPos.z=cameraPos.z+STEP;
    }
    if( keystate[SDLK_DOWN] ){
        cameraPos.z=cameraPos.z-STEP;
    }
    //change the rotation angle to rotate along y axis
    if( keystate[SDLK_LEFT] ){
        yaw=yaw +ANGLE_STEP;
        rotation(yaw);
    }
    if( keystate[SDLK_RIGHT] ){
        yaw=yaw -ANGLE_STEP;
        rotation(yaw);
    }
    //change light position to move the light source around
    if (keystate[SDLK_w])
        lightPos.z += delta * 2;
    
    if (keystate[SDLK_s])
        lightPos.z -= delta * 2;
    
    if (keystate[SDLK_a])
        lightPos.x -= delta * 2;
    
    if (keystate[SDLK_d])
        lightPos.x += delta * 2;
    
    if (keystate[SDLK_q])
        lightPos.y += delta * 2;
    
    if (keystate[SDLK_e])
        lightPos.y -= delta * 2;
}
//calculate max float
float max(float a, float b){
    if(a>=b){
        return a;
    }
    else if(b>a){
        return b;
    }
}
//shade each vertex
void VertexShader( const Vertex& v, Pixel& p )
{
    vec3 temp = R* (v.position - cameraPos) ;//translation and rotation (projection of 3d point in 2d)
    p.zinv = 1.0f/temp.z;
    p.x = (focal_length*temp.x/temp.z)+SCREEN_WIDTH/2;
    p.y = (focal_length*temp.y/temp.z)+SCREEN_HEIGHT/2;
    p.pos3d=v.position;//3d position of 2d pixel
}
//interpolate between two vec3 values
void Interpolate(vec3 a, vec3 b, vector<vec3>& result) {
    int N = result.size();
    vec3 step = vec3(b - a) / float(glm::max(N - 1, 1));
    vec3 current(a);
    for (int i = 0; i < N; ++i) {
        result[i] = current;
        current += step;
    }
}
//interpolate pixels(x,y,zinv)
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result )
{
    int N = result.size();
    float step_x = (float)(b.x-a.x)/ float(max(N-1,1));
    float step_y = (float)(b.y-a.y)/ float(max(N-1,1));
    float step_zinv = (b.zinv-a.zinv)/ float(max(N-1,1));
    
    vector<vec3> positions3D(N);
    Interpolate(a.pos3d * a.zinv, b.pos3d * b.zinv, positions3D);//interpolate pos3d/z
    
    float current_X = a.x;
    float current_Y = a.y;
    float current_ZINV = a.zinv;
    
    for( int i=0; i<N-1; ++i )
    {
        result[i].x = round(current_X);
        result[i].y = round(current_Y);
        result[i].zinv = current_ZINV;
        
        current_X += step_x;
        current_Y += step_y;
        current_ZINV +=step_zinv;
        result[i].pos3d = positions3D[i] / result[i].zinv;
    }
    result[N-1].x=b.x;
    result[N-1].y=b.y;
    result[N-1].zinv=b.zinv;
    result[N-1].pos3d = b.pos3d;
}
//shade each pixel
void PixelShader( const Pixel& p )
{
    int x = p.x;
    int y = p.y;
    
    //Check if inside the screen
    if( x >= 0 && y >= 0 &&  x < SCREEN_WIDTH && y < SCREEN_HEIGHT ){
        //if closest to the screen
        if( p.zinv > depthBuffer[y][x] ){
        depthBuffer[y][x] = p.zinv;//update depth buffer
            
        //illumination
        vec3 normal =glm::normalize(currentNormal);
        //light ray vector
        vec3 rVector =glm::normalize(lightPos-p.pos3d);
        float distance = glm::distance(lightPos, p.pos3d);
        //power calculation for each point
        float surfaceArea = 4 * PI * pow(distance,2);
        vec3 powerPerArea = lightPower / surfaceArea;
        float dotProduct = (rVector.x * normal.x + rVector.y * normal.y + rVector.z * normal.z);
        vec3 directIllumination = powerPerArea * max(0, dotProduct);
        
        vec3 illumination = currentReflectance*(directIllumination+indirectLightPowerPerArea);
        PutPixelSDL( screen, x, y, illumination );
        }
    }
}
void DrawLineSDL( SDL_Surface* surface, Pixel a, Pixel b, vec3 color )
{
    Pixel delta;
    delta.x = glm::abs( a.x - b.x );
    delta.y = glm::abs( a.y - b.y );
    
    int pixels = glm::max( delta.x, delta.y ) + 1;
    
    //interpolate between a and b and shade every pixel in between
    vector<Pixel> line( pixels );
    Interpolate( a, b, line );
    
    for (int i=0; i< pixels; ++i){
        PixelShader(line[i]);
    }
}
void ComputePolygonRows(
                        const vector<Pixel>& vertexPixels,
                        vector<Pixel>& leftPixels,
                        vector<Pixel>& rightPixels
                        ){
    // 1. Find max and min y-value of the polygon and compute the number of rows it occupies.
    int maxY = max(vertexPixels[0].y,max(vertexPixels[1].y,vertexPixels[2].y) );
    int minY = min(vertexPixels[0].y,min(vertexPixels[1].y,vertexPixels[2].y) );
    int Rows = maxY - minY + 1;
    
    // 2. Resize leftPixels and rightPixels so that they have an element for each row.
    leftPixels.resize(Rows);
    rightPixels.resize(Rows);
    
    // 3. Initialize the x-coordinates in leftPixels to some really large value and the x-coordinates
    // in rightPixels to some really small value.
    for( int i=0; i<Rows; ++i )
    {
        leftPixels[i].x = +numeric_limits<int>::max();
        rightPixels[i].x = -numeric_limits<int>::max();
    }
    
    // 4. Loop through all edges of the polygon and use linear interpolation to find the x-coordinate for
    // each row it occupies. Update the corresponding values in rightPixels and leftPixels.
    int size = vertexPixels.size();
    
    for( int i=0; i<size; ++i )
    {
        int j = (i+1)%size; // The next vertex
        ivec2 delta;
        delta.x = abs( vertexPixels[i].x - vertexPixels[j].x );
        delta.y = abs( vertexPixels[i].y - vertexPixels[j].y );
        int pixels = max( delta.x, delta.y ) + 1;
        vector<Pixel> line( pixels );
        Interpolate( vertexPixels[i], vertexPixels[j], line );
        
        for(int t = 0;t<line.size();t++){//update left pixels and right pixels
            int pos = line[t].y-minY;
            if((leftPixels[pos].x) > line[t].x){
                leftPixels[pos].x = line[t].x;
                leftPixels[pos].y = line[t].y;
                leftPixels[pos].zinv = line[t].zinv;
                leftPixels[pos].pos3d.x = line[t].pos3d.x;
                leftPixels[pos].pos3d.y = line[t].pos3d.y;
                leftPixels[pos].pos3d.z = line[t].pos3d.z;
            }
            if((rightPixels[pos].x) <line[t].x){
                rightPixels[pos].x = line[t].x;
                rightPixels[pos].y = line[t].y;
                rightPixels[pos].zinv = line[t].zinv;
                rightPixels[pos].pos3d.x = line[t].pos3d.x;
                rightPixels[pos].pos3d.y = line[t].pos3d.y;
                rightPixels[pos].pos3d.z = line[t].pos3d.z;
            }
        }
    }
}
void DrawPolygonRows(const vector<Pixel>& leftPixels,const vector<Pixel>& rightPixels)
{
    for(int i =0;i<leftPixels.size();i++){
        DrawLineSDL(screen, leftPixels[i], rightPixels[i], currentColor);
    }
}
void DrawPolygon( const vector<Vertex>& vertices )
{
    int V = vertices.size();
    vector<Pixel> vertexPixels( V );
    for( int i=0; i<V; ++i )
        VertexShader( vertices[i], vertexPixels[i] );
    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    DrawPolygonRows( leftPixels, rightPixels );
}
void Draw()
{
    SDL_FillRect( screen, 0, 0 );
    if( SDL_MUSTLOCK(screen) )
        SDL_LockSurface(screen);
    
    for( int y=0; y<SCREEN_HEIGHT; ++y )//initialize depth buffer
        for( int x=0; x<SCREEN_WIDTH; ++x )
            depthBuffer[y][x] = 0;
    
    for( int i=0; i<triangles.size(); ++i )
    {
        
        vector<Vertex> vertices(3);
        vertices[0].position = triangles[i].v0;
        vertices[1].position = triangles[i].v1;
        vertices[2].position = triangles[i].v2;
        
        currentNormal=triangles[i].normal;
        currentReflectance=triangles[i].color;
        DrawPolygon( vertices );
    }
    if ( SDL_MUSTLOCK(screen) )
        SDL_UnlockSurface(screen);
    SDL_UpdateRect( screen, 0, 0, 0, 0 );
}
