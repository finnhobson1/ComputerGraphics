#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <math.h>
#include <omp.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 400
#define SCREEN_HEIGHT 400
#define FULLSCREEN_MODE false
#define _USE_MATH_DEFINES

struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex = 0;
};


/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
vec4 cameraPos( 0.0, 0.0, -3.0, 1.0 );
mat4 yRotation = mat4(1.0f);
float yAngle;

vec4 lightPos( 0, -0.5, -0.8, 1.0 );
float intensity = 12.f;
float red = 1.0f;
float green = 1.0f;
float blue = 1.0f;
vec3 lightColor = intensity * vec3( red, green, blue );
vec3 indirectLight = 0.5f * vec3(red, green, blue);

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
bool Update();
void Draw(screen* screen, const vector<Triangle>& triangles);
bool ClosestIntersection( vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);
void UpdateRotation();
vec3 DirectLight( const Intersection& i, const vector<Triangle>& triangles );
void UpdateLightColour();


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  // Loads Cornell Box model
  vector<Triangle> triangles;
  LoadTestModel(triangles);

  int NUM_THREADS = omp_get_max_threads();
  omp_set_num_threads(NUM_THREADS);

  while( Update() )
    {
      Draw(screen, triangles);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

// Draws the current frame
void Draw(screen* screen, const vector<Triangle>& triangles)
{
  float focalLength = SCREEN_HEIGHT;
  Intersection intersection;

  // Clear buffer
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  #pragma omp parallel
  {
    #pragma omp for nowait collapse(2) private(intersection)
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      for (int y = 0; y < SCREEN_HEIGHT; y++) {
        vec3 color(0.0f);
        vec3 reflectColor(0.0f);
        //ANTI-ALIASING USING SUPER-SAMPLING
        for (float i = -0.004; i <= 0.004; i+=0.002) {
          for (float j = -0.004; j <= 0.004; j+=0.002) {
            vec4 dir(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength, 1.0);
            if (ClosestIntersection(cameraPos + vec4(i,j,0,0), dir, triangles, intersection))
            {
              // SINGLE COLOURS
              int index = intersection.triangleIndex;

              //REFLECTION
              if (triangles[index].reflects) {
                Intersection reflectIntersection;
                vec4 reflection = 2 * glm::dot(-dir, triangles[index].normal) * triangles[index].normal + dir;
                if (ClosestIntersection(intersection.position + vec4(0.0001, 0, 0, 0), reflection, triangles, reflectIntersection)) {
                  int reflectIndex = reflectIntersection.triangleIndex;
                  vec3 trueColor = triangles[reflectIndex].color;
                  // DIRECT LIGHTING
                  vec3 directLight = DirectLight(reflectIntersection, triangles);
                  reflectColor += trueColor * (directLight + indirectLight);

                  PutPixelSDL(screen, x, y, reflectColor/20.0f);
                }
              }

              else {
                vec3 trueColor = triangles[index].color;

                // DIRECT LIGHTING
                vec3 directLight = DirectLight(intersection, triangles);
                color += trueColor * (directLight + indirectLight);

                PutPixelSDL(screen, x, y, color/20.0f);
              }
            }
          }
        }
      }
    }
  }
}


// Updates parameters
bool Update()
{
  // Compute frame time
  static int t = SDL_GetTicks();
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  std::cout << "Render time: " << dt << " ms." << std::endl;

  /* Update variables*/
  SDL_Event e;
  while(SDL_PollEvent(&e))
  {
    if (e.type == SDL_QUIT)
    {
      return false;
    }
    else if (e.type == SDL_KEYDOWN)
    {
      int key_code = e.key.keysym.sym;
      switch(key_code)
      {
        case SDLK_UP:
          // Move camera forward
          cameraPos.z += 0.1;
          break;
        case SDLK_DOWN:
          // Move camera backwards
          cameraPos.z -= 0.1;
          break;
        case SDLK_LEFT:
          // Move camera left
          yAngle -= 0.1;
          UpdateRotation();
          break;
        case SDLK_RIGHT:
          // Move camera right
          yAngle += 0.1;
          UpdateRotation();
          break;
        case SDLK_w:
          // Move light forward
          lightPos.z += 0.1;
          break;
        case SDLK_s:
          // Move light forward
          lightPos.z -= 0.1;
          break;
        case SDLK_a:
          // Move light left
          lightPos.x -= 0.1;
          break;
        case SDLK_d:
          // Move light right
          lightPos.x += 0.1;
          break;
        case SDLK_q:
          // Move light up
          lightPos.y -= 0.1;
          break;
        case SDLK_e:
          // Move light down
          lightPos.y += 0.1;
          break;
        case SDLK_r:
          // Increase light intensity
          intensity += 1.0;
          UpdateLightColour();
          break;
        case SDLK_f:
          //Decrease light intensity
          if (intensity > 0) intensity -= 1.0;
          UpdateLightColour();
          break;
        case SDLK_t:
          // Increase light red value
          if (red < 1) red += 0.05;
          UpdateLightColour();
          break;
        case SDLK_g:
          // Decrease light red value
          if (red > 0) red -= 0.05;
          UpdateLightColour();
          break;
        case SDLK_y:
          // Increase light green value
          if (green < 1) green += 0.05;
          UpdateLightColour();
          break;
        case SDLK_h:
          // Decreae light green value
          if (green > 0) green -= 0.05;
          UpdateLightColour();
          break;
        case SDLK_u:
          // Increase light blue value
          if (blue < 1) blue += 0.05;
          UpdateLightColour();
          break;
        case SDLK_j:
          if (blue > 0) blue -= 0.05;
          UpdateLightColour();
          break;
        case SDLK_ESCAPE:
          // Quit
          return false;
        }
      }
    }
  return true;
}


// Calculates intersection with triangle closest to the camera, along a ray
// Returns true if an intersection occurs, false otherwise
bool ClosestIntersection( vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection)
{
  bool intersects = false;
  float m = std::numeric_limits<float>::max();
  closestIntersection.distance = m;

  for (unsigned int i = 0; i < triangles.size(); i++)
  {
    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    // Rotate points around the y-axis
    v0 = yRotation * v0;
    v1 = yRotation * v1;
    v2 = yRotation * v2;

    // Calculate intersection with the plane that the triangle lies on
    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);

    vec3 d = vec3(dir.x, dir.y, dir.z);

    mat3 A( -d, e1, e2 );
    vec3 x = glm::inverse( A ) * b;

    float t = x.x;
    float u = x.y;
    float v = x.z;

    //Check if intersection falls within triangle boundaries, and if intersection is closest to camera
    if (u >= 0 && v >= 0 && (u+v) <= 1 && t >= 0 && t < closestIntersection.distance)
    {
      closestIntersection.position.x = v0.x + u*e1.x + v*e2.x;
      closestIntersection.position.y = v0.y + u*e1.y + v*e2.y;
      closestIntersection.position.z = v0.z + u*e1.z + v*e2.z;
      closestIntersection.position.w = v0.w;

      closestIntersection.distance = t;
      closestIntersection.triangleIndex = i;

      intersects = true;
    }
  }

  return intersects;
}


// Updates rotation matrix R with new y rotation value
void UpdateRotation()
{
  yRotation[0][0] = cos(yAngle);
  yRotation[0][2] = sin(yAngle);
  yRotation[2][0] = -sin(yAngle);
  yRotation[2][2] = cos(yAngle);
}


// Calcalutes pixel colour value based on relationship with light source
vec3 DirectLight( const Intersection& i, const vector<Triangle>& triangles )
{
  int index = i.triangleIndex;
  vec4 position = i.position;
  vec4 normal = triangles[index].normal;
  vec3 D = vec3(0,0,0);

  //SOFT-SHADOWS
  for (float y = -0.05; y <= 0.05; y+=0.025f) {
    for (float x = -0.05; x <= 0.05; x+=0.025f) {
      // Vector from intersection point to light source
      vec4 lightDir = vec4((lightPos.x + x ) - position.x, lightPos.y  - position.y, (lightPos.z + y) - position.z , position.w);
      vec4 unitLightDir = glm::normalize(lightDir);

      float projection = glm::dot(unitLightDir, normal);
      //Distance from intersection point to light source
      float radius = glm::length(vec3(lightDir.x, lightDir.y, lightDir.z));

      Intersection shadowIntersection;
      if (ClosestIntersection(lightPos, -unitLightDir, triangles, shadowIntersection)) {
        if (shadowIntersection.distance >= radius) {
            D += lightColor * max(projection, 0.0f) / (4.0f * float(M_PI) * radius * radius);
        }
      }
    }
  }
  return D * 0.025f;
}


void UpdateLightColour()
{
  lightColor = intensity * vec3(red, green, blue);
  indirectLight = intensity/28.0f * vec3(red, green, blue);
}
