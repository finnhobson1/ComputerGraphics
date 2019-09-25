#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <bitset>

using namespace std;
using glm::vec2;
using glm::ivec2;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 500
#define SCREEN_HEIGHT 500
#define FULLSCREEN_MODE false

struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
};

struct Vertex
{
   vec4 position;
   bitset<4> outcode;
};

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
vector<Triangle> triangles;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

vec4 cameraPos( 0, 0, -3.001, 1 );
mat4 yRotation = mat4(1.0f);
float yAngle = 0;

vec4 lightPos( 0, -0.5, -0.7, 1.0 );
float intensity = 12.f;
float red = 1.0f;
float green = 1.0f;
float blue = 1.0f;
vec3 lightColor = intensity * vec3( red, green, blue );
vec3 indirectLight = 0.5f * vec3(red, green, blue);

vec3 currentColor;
vec4 currentNormal;

float umax = SCREEN_WIDTH/2;
float vmax = SCREEN_HEIGHT/2;


/* ---------------------------------------------------------------------------- */
/* FUNCTIONS                                                                    */
/* ---------------------------------------------------------------------------- */

bool Update();
void Draw(screen* screen);
void VertexShader( const Vertex& v, ivec2& p );
void UpdateRotation();
void ComputePolygonRows( const vector<Pixel>& vertexPixels,
    vector<Pixel>& leftPixels,
    vector<Pixel>& rightPixels );
void DrawRows( screen* screen, const vector<Pixel>& leftPixels,
    const vector<Pixel> rightPixels );
void PixelShader(screen* screen, const Pixel& p );
void DrawPolygon( screen* screen, const vector<Vertex>& vertices, vector<Pixel>& pixels );
void InterpolatePixel( Pixel a, Pixel b, vector<Pixel>& result );
vec3 DirectLight( const Vertex & v );
vector<Pixel> InterpolateLine( Pixel a, Pixel b );
void UpdateLightColour();
int ClipPolygon( vector<Vertex>& vertices, vector<Vertex>& inVertices );
void CalculateOutcode( Vertex& v );
Vertex ClipPolygonLeft(Vertex i, Vertex j);
Vertex ClipPolygonRight(Vertex i, Vertex j);
Vertex ClipPolygonBottom(Vertex i, Vertex j);
Vertex ClipPolygonTop(Vertex i, Vertex j);


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  LoadTestModel(triangles);

  while ( Update())
    {
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( int y=0; y<SCREEN_HEIGHT; ++y )
    for( int x=0; x<SCREEN_WIDTH; ++x )
      depthBuffer[y][x] = 0;

  for( uint32_t i=0; i<triangles.size(); ++i ){
    currentColor = triangles[i].color;
    currentNormal = triangles[i].normal;

    vector<Vertex> vertices(3);

    vertices[0].position = triangles[i].v0;
    vertices[1].position = triangles[i].v1;
    vertices[2].position = triangles[i].v2;

    for (int v = 0; v < 3; v++ ) {
      //Rotate and Translate Vertex
      vertices[v].position = yRotation * vertices[v].position;
      vertices[v].position = vertices[v].position - cameraPos;
      vertices[v].position.w = vertices[v].position.z / SCREEN_HEIGHT;
    }

    //CLIP VERTICES
    vector<Vertex> inVertices;
    int inView = ClipPolygon( vertices, inVertices );

    vector<Pixel> inPixels(inView);
    for (int v = 0; v < inView; v++ ) {
      inVertices[v].position.w = 1;
      inPixels[v].pos3d = glm::inverse(yRotation) * (inVertices[v].position + cameraPos);
    }

    //DIVIDE NEW VERTICES INTO TRIANGLES
    if (inView == 3) DrawPolygon( screen, inVertices, inPixels );

    if (inView == 4) {
      vector<Vertex> newVertices(3);
      vector<Pixel> newPixels(3);
      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[1];
      newVertices[2] = inVertices[2];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[1];
      newPixels[2] = inPixels[2];
      DrawPolygon( screen, newVertices, newPixels );

      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[2];
      newVertices[2] = inVertices[3];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[2];
      newPixels[2] = inPixels[3];
      DrawPolygon( screen, newVertices, newPixels );

    }
    if (inView == 5) {
      vector<Vertex> newVertices(3);
      vector<Pixel> newPixels(3);
      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[1];
      newVertices[2] = inVertices[2];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[1];
      newPixels[2] = inPixels[2];
      DrawPolygon( screen, newVertices, newPixels );

      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[2];
      newVertices[2] = inVertices[3];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[2];
      newPixels[2] = inPixels[3];
      DrawPolygon( screen, newVertices, newPixels );

      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[3];
      newVertices[2] = inVertices[4];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[3];
      newPixels[2] = inPixels[4];
      DrawPolygon( screen, newVertices, newPixels );
    }
    if (inView == 6) {
      vector<Vertex> newVertices(3);
      vector<Pixel> newPixels(3);
      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[1];
      newVertices[2] = inVertices[2];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[1];
      newPixels[2] = inPixels[2];
      DrawPolygon( screen, newVertices, newPixels );

      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[2];
      newVertices[2] = inVertices[3];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[2];
      newPixels[2] = inPixels[3];
      DrawPolygon( screen, newVertices, newPixels );

      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[3];
      newVertices[2] = inVertices[4];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[3];
      newPixels[2] = inPixels[4];
      DrawPolygon( screen, newVertices, newPixels );

      newVertices[0] = inVertices[0];
      newVertices[1] = inVertices[4];
      newVertices[2] = inVertices[5];
      newPixels[0] = inPixels[0];
      newPixels[1] = inPixels[3];
      newPixels[2] = inPixels[4];
      DrawPolygon( screen, newVertices, newPixels );
    }
  }
}

/*Place updates of parameters here*/
bool Update()
{
  /*static int t = SDL_GetTicks();
  // Compute frame time
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  std::cout << "Render time: " << dt << " ms." << std::endl;*/

  SDL_Event e;
  while(SDL_PollEvent(&e))
  {
    if (e.type == SDL_QUIT) return false;
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
          yAngle -= float(M_PI)/30.0f;
          UpdateRotation();
          break;
        case SDLK_RIGHT:
          // Move camera right
          yAngle += float(M_PI)/30.0f;
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


void VertexShader( const Vertex& v, Pixel& p ) {
  vec4 P = v.position;

  //Project points onto image plane
  p.x = (SCREEN_HEIGHT * P.x / P.z) + (SCREEN_WIDTH/2);
  p.y = (SCREEN_HEIGHT * P.y / P.z) + (SCREEN_HEIGHT/2);
  p.zinv = 1.0f / P.z;

  p.pos3d = p.pos3d * p.zinv;
}

vec3 DirectLight( const Pixel& p )
{
  vec4 position = p.pos3d / p.zinv;
  vec4 normal = currentNormal;

  // Vector from intersection point to light source
  vec4 lightDir = vec4(lightPos.x-position.x, lightPos.y-position.y, lightPos.z-position.z, position.w);
  vec4 unitLightDir = glm::normalize(lightDir);

  float projection = glm::dot(unitLightDir, normal);
  //Distance from intersection point to light source
  float radius = glm::length(vec3(lightDir.x, lightDir.y, lightDir.z));

  vec3 D = lightColor * max(projection, 0.0f) / (4.0f * float(M_PI) * radius * radius);
  return D;
}


void InterpolatePixel( Pixel a, Pixel b, vector<Pixel>& result )
{
  int N = result.size();
  vec3 vecA(a.x, a.y, a.zinv);
  vec3 vecB(b.x, b.y, b.zinv);
  vector<vec3> vecResult( N );

  vec3 step = (vecB - vecA) / float(max(N-1,1));
  vec4 posStep = (b.pos3d - a.pos3d) / float(max(N-1,1));

  vec3 current( vecA );
  vec4 currentPos( a.pos3d );
  for( int i=0; i<N; ++i )
  {
    vecResult[i] = current;
    result[i].x = round(vecResult[i].x);
    result[i].y = round(vecResult[i].y);
    result[i].zinv = vecResult[i].z;
    result[i].pos3d = currentPos;
    current += step;
    currentPos += posStep;
  }
}


vector<Pixel> InterpolateLine( Pixel a, Pixel b ) {
  int xDelta = glm::abs( a.x - b.x );
  int yDelta = glm::abs( a.y - b.y );
  int pixels = glm::max( xDelta, yDelta ) + 1;
  vector<Pixel> line(pixels);
  InterpolatePixel(a, b, line);
  return line;
}


// Updates rotation matrix R with new y rotation value
void UpdateRotation()
{
  yRotation[0][0] = cos(yAngle);
  yRotation[0][2] = sin(yAngle);
  yRotation[2][0] = -sin(yAngle);
  yRotation[2][2] = cos(yAngle);
}

void ComputePolygonRows( const vector<Pixel>& vertexPixels,
    vector<Pixel>& leftPixels,
    vector<Pixel>& rightPixels )
{
  vector<Pixel> edgePixels;
  int V = vertexPixels.size();
  for (int i = 0; i < V; i++) {
    int j = (i+1)%V;
    vector<Pixel> line = InterpolateLine( vertexPixels[i], vertexPixels[j] );
    edgePixels.insert(edgePixels.end(), line.begin(), line.end());
  }

  int yMax = -numeric_limits<int>::max();
  int yMin = numeric_limits<int>::max();

  for ( uint32_t i = 0; i < edgePixels.size(); i++) {
    if (edgePixels[i].y > yMax) yMax = edgePixels[i].y;
    if (edgePixels[i].y < yMin) yMin = edgePixels[i].y;
  }

  int ROWS = yMax - yMin + 1;

  leftPixels.resize(ROWS);
  rightPixels.resize(ROWS);

  for ( int i = 0; i < ROWS; i++ ) {
    leftPixels[i].x = numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
    leftPixels[i].y = yMin + i;
    rightPixels[i].y = yMin + i;
  }

  for ( uint32_t i = 0; i < edgePixels.size(); i++ ) {
    int row = edgePixels[i].y - yMin;
    if (edgePixels[i].x < leftPixels[row].x) {
      leftPixels[row].x = edgePixels[i].x;
      leftPixels[row].zinv = edgePixels[i].zinv;
      leftPixels[row].pos3d = edgePixels[i].pos3d;
    }
    if (edgePixels[i].x > rightPixels[row].x) {
      rightPixels[row].x = edgePixels[i].x;
      rightPixels[row].zinv = edgePixels[i].zinv;
      rightPixels[row].pos3d = edgePixels[i].pos3d;
    }
  }
}


void DrawRows( screen* screen, const vector<Pixel>& leftPixels,
               const vector<Pixel> rightPixels )
{
  int ROWS = leftPixels.size();
  for ( int i = 0; i < ROWS; i++ ) {
    //int pixels = 2;
    int pixels = rightPixels[i].x - leftPixels[i].x + 1;
    vector<Pixel> row(pixels);
    InterpolatePixel( leftPixels[i], rightPixels[i], row );
    for ( int j = 0; j < pixels; j++ ) {
      PixelShader(screen, row[j]);
      }
    }
  }

void PixelShader(screen* screen, const Pixel& p )
   {
       int x = p.x;
       int y = p.y;
       if( x > 0 && x < SCREEN_WIDTH && y > 0 && y < SCREEN_HEIGHT && p.zinv > depthBuffer[y][x] )
       {
           depthBuffer[y][x] = p.zinv;
           vec3 directLight = DirectLight(p);
           vec3 illumination = currentColor * (directLight + indirectLight);
           PutPixelSDL( screen, x, y, illumination );
       }
}

void DrawPolygon( screen* screen, const vector<Vertex>& vertices, vector<Pixel>& vertexPixels )
{
    int V = vertices.size();

    for (int i = 0; i < V; i++) {
      VertexShader( vertices[i], vertexPixels[i] );
    }

    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    DrawRows( screen, leftPixels, rightPixels );
}


int ClipPolygon( vector<Vertex>& vertices, vector<Vertex>& inVertices ) {
  int inView = 0;
  bitset<4> lastANDcode;

  for (uint32_t i = 0; i < vertices.size(); i++ ){
    CalculateOutcode(vertices[i]);
  }

  for (uint32_t i = 0; i < vertices.size(); i++ ) {
    int j = (i+1)%vertices.size();
    bitset<4> ORcode = (vertices[i].outcode | vertices[j].outcode);
    bitset<4> ANDcode = (vertices[i].outcode & vertices[j].outcode);

    //Both vertices on screen - add 2nd vertex to inVertices.
    if ( ORcode == 0 ) {
      inVertices.push_back( vertices[j] );
      inView++;
    }

    //1st vertex off screen - add intersection vertex (and 2nd vertex if on screen)
    if ( ANDcode == 0 && vertices[i].outcode != 0 ) {
      Vertex clippedVertex = vertices[i];
      if ( vertices[i].outcode[0] == 1 ) {
        //Clip Left
        clippedVertex = ClipPolygonLeft(vertices[j], clippedVertex);
      }
      else if ( vertices[i].outcode[1] == 1 ) {
        //Clip Right
        clippedVertex = ClipPolygonRight(vertices[j], clippedVertex);
      }
      if ( vertices[i].outcode[2] == 1 ) {
        //Clip Bottom
        clippedVertex = ClipPolygonBottom(vertices[j], clippedVertex);
      }
      else if ( vertices[i].outcode[3] == 1 ) {
        //Clip Top
        clippedVertex = ClipPolygonTop(vertices[j], clippedVertex);
      }
      inVertices.push_back(clippedVertex);
      inView++;
      if (vertices[j].outcode == 0)  {
        inVertices.push_back( vertices[j] );
        inView++;
      }
    }

    //2nd vertex off screen - add intersection vertex
    if ( ANDcode == 0 && vertices[j].outcode != 0 ) {
      Vertex clippedVertex = vertices[j];
      if ( vertices[j].outcode[0] == 1 ) {
        //Clip Left
        clippedVertex = ClipPolygonLeft(vertices[i], clippedVertex);
      }
      else if ( vertices[j].outcode[1] == 1 ) {
        //Clip Right
        clippedVertex = ClipPolygonRight(vertices[i], clippedVertex);
      }
      if ( vertices[j].outcode[2] == 1 ) {
        //Clip Bottom
        clippedVertex = ClipPolygonBottom(vertices[i], clippedVertex);
      }
      else if ( vertices[j].outcode[3] == 1 ) {
        //Clip Top
        clippedVertex = ClipPolygonTop(vertices[i], clippedVertex);
      }
      inVertices.push_back(clippedVertex);
      inView++;
    }
  }
  return inView;
}

Vertex ClipPolygonTop(Vertex i, Vertex j) {
  float iy = i.position.y;
  float iw = i.position.w;
  float jy = j.position.y;
  float jw = j.position.w;
  float t = (iw + 2*iy/SCREEN_WIDTH)/((iw + 2*iy/SCREEN_WIDTH)-(jw + 2*jy/SCREEN_WIDTH));

  Vertex intersect;
  intersect.position = i.position + t * (j.position - i.position);

  return intersect;
}

Vertex ClipPolygonLeft(Vertex i, Vertex j) {
  float ix = i.position.x;
  float iw = i.position.w;
  float jx = j.position.x;
  float jw = j.position.w;
  float t = (iw + 2*ix/SCREEN_WIDTH)/((iw + 2*ix/SCREEN_WIDTH)-(jw + 2*jx/SCREEN_WIDTH));

  Vertex intersect;
  intersect.position = i.position + t * (j.position - i.position);

  return intersect;
}

Vertex ClipPolygonBottom(Vertex i, Vertex j) {
    float iy = i.position.y;
    float iw = i.position.w;
    float jy = j.position.y;
    float jw = j.position.w;
    float t = (iw - 2*iy/SCREEN_WIDTH)/((iw - 2*iy/SCREEN_WIDTH)-(jw - 2*jy/SCREEN_WIDTH));

    Vertex intersect;
    intersect.position = i.position + t * (j.position - i.position);

    return intersect;
}

Vertex ClipPolygonRight(Vertex i, Vertex j) {
  float ix = i.position.x;
  float iw = i.position.w;
  float jx = j.position.x;
  float jw = j.position.w;
  float t = (iw - 2*ix/SCREEN_WIDTH)/((iw - 2*ix/SCREEN_WIDTH)-(jw - 2*jx/SCREEN_WIDTH));

  Vertex intersect;
  intersect.position = i.position + t * (j.position - i.position);

  return intersect;
}

void CalculateOutcode( Vertex& v ) {
  float xmax = umax *  v.position.w;
  float xmin = -xmax;
  float ymax = vmax * v.position.w;
  float ymin = -ymax;
  float x = v.position.x;
  float y = v.position.y;
  //float z = vertices[i].position.z;
  if (y < ymin) v.outcode[3] = 1;
  if (y > ymax) v.outcode[2] = 1;
  if (x > xmax) v.outcode[1] = 1;
  if (x < xmin) v.outcode[0] = 1;
}


void UpdateLightColour()
{
  lightColor = intensity * vec3(red, green, blue);
  indirectLight = intensity/28.0f * vec3(red, green, blue);
}
