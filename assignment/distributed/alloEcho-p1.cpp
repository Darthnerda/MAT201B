// Starter code, slurp function, and much advice by Karl Yerkes
// Kinect calibration matrices from libfreenect glpcview.cpp example

// includes
#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)
#include "al/ui/al_Parameter.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "al/spatial/al_HashSpace.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "libfreenect.h"
#include "libfreenect_sync.h"
#include <vector>
using namespace al;
using namespace std;

// tweak zone
const float  minDistance = -10;
const float scalingFactor = 0.00021;
const int trailLength = 4;
const int NBoids = 8000;
const int NSpheres = 10;
const int sphereRingR = 35;
const int kinectW = 640;
const int kinectH = 480;
const int maxBoidNeighbors = 100;
const bool kinectInitiallyStreaming = false;
const unsigned initialKinectSphere = 5;
const float initialShellTransparency = 0.079;
//const al::Color shellFactions[] = { RGB(1,0,0), RGB(0,1,0), RGB(0,0,1) };
// end tweak zone

// calculate some constants
const int NKinect = kinectW * kinectH;
const int boidsPerSphere = NBoids / NSpheres;
const int boidifyStride = ceil(NKinect / boidsPerSphere);
HashSpace::Query qmany(maxBoidNeighbors);

// early globals declarations
unsigned maxradius;
float floatDim;
Vec3f hashCenter;
bool kinectStreaming = kinectInitiallyStreaming;
unsigned trailIdx = 0;
//vector<Colori> shellPix;
//int allegiances[265*265];
//unsigned kinectSphere = initialKinectSphere;

// early function declarations
string slurp(string fileName);

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
}

bool isBoid(unsigned id) {
	return false;
}

// possibly unused calibration matrices
const Matrix4f vertexCalibrate(
        1/ 594.21f,     0,  0, 0,
        0,    -1/ 591.04f,  0, 0,
        0,       0,  0, -0.0030711f,
        -339.5f/594.21f, 242.7f/ 591.04f, -1,3.3309495f
);
const Matrix4f rgbCalibrate(
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
);

// data structures
enum sphereType {BOID, THREEDVIDEO};

struct ShareVoxel {
  Vec3f pos;
  al::Color col;
};

struct SharedState {
  ShareVoxel voxels[NKinect];
};

struct Agent {
  Vec3f velocity;
  Vec3f acceleration;
  double mass;
};

struct Bubble {
  Vec3f center;
  Vec3f origin;
  HashSpace space;
  sphereType type;
  Agent agents[boidsPerSphere];
};

struct MyApp : public DistributedAppWithState<SharedState> {
  // declare meshes and textures
  Mesh boids;
  Mesh trails[NBoids];
  Mesh spheres[NSpheres];
  Mesh kinectVoxels;
  //Texture shellTex;

  // declare simulation structs
  //Agent agents[NBoids];
  Bubble bubbles[NSpheres];
  //HashSpace shellSpace;

  // initialize some parameters
  Parameter timeStep{ "/timeStep", "", 0.1, "", 0.0000001, 0.1 };
  Parameter pointSize{ "/pointSize", "", 1.0, "", 0.0, 40.0 };
  Parameter KDist{ "/KDist","",0.23,"",0.000001,1.0 };
  Parameter cohesionModifier{ "/cohesionModifier","",0.60,"",0.1,10 };
  Parameter separationModifier{ "/separationModifier","",0.8,"",0.1,10 };
  Parameter alignmentModifier{ "/alignmentModifier","",0.96,"",0,10 };
  Parameter minimumDisparity{ "/minimumDisparity", "", 500, "", 0, 1000 };
  Parameter maximumDisparity{ "/maximumDisparity", "", 1100, "", 0, 5000 };
  ParameterInt kinectSphere{ "/kinectSphere", "", initialKinectSphere, "", 0, NSpheres - 1 };
  Parameter shellTransparency{ "/shellTransparency", "", initialShellTransparency, "", 0, 1 };

  // declare the gui
  ControlGUI gui;

  // declare the point shader
  ShaderProgram pointShader; 

  // declare a pointer that is shared between the simulation machine and the renderers
  std::shared_ptr<CuttleboneStateSimulationDomain<SharedState>> cuttleboneDomain;

  void onCreate() override {
	// load cuttlebone
	cuttleboneDomain = CuttleboneStateSimulationDomain<SharedState>::enableCuttlebone(this);
	if(!cuttleboneDomain){
		std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
		quit();
	}

	// compile shaders
	pointShader.compile(slurp("../point-vertex.glsl"),
		slurp("../point-fragment.glsl"),
		slurp("../point-geometry.glsl"));

	// pipe parameters into the gui
	gui << pointSize << KDist << cohesionModifier << separationModifier << alignmentModifier << timeStep << minimumDisparity << maximumDisparity << kinectSphere << shellTransparency;
	gui.init();

	// pipe parameters to the shared parameter server
	parameterServer() << pointSize << KDist << cohesionModifier << separationModifier << alignmentModifier << timeStep << minimumDisparity << maximumDisparity << kinectSphere << shellTransparency;

	// prep navigation stuff
	navControl().useMouse(false);
	nav().pos(Vec3f(0,0,25));

	/*
	// create semi transparent texture for the bubble shells
	shellTex.create2D(256, 256, Texture::RGBA8);
	int Nx = shellTex.width();
	int Ny = shellTex.height();
	//vector<Colori> pix;
	shellPix.resize(Nx * Ny);
	shellSpace = HashSpace(4, Nx * Ny);
	int n, m;
	n = m = 0;
	for (unsigned j = 0; j < Ny; ++j) {
		for (unsigned i = 0; i < Nx; ++i) {
			Color c = RGB( 0, 0, 0 );	
			unsigned idx = j * Nx + i;	
			allegiances[idx] = -1; // meaning black
		
			if (al::rnd::prob(0.01)) { // its a seed!	
				shellSpace.move( idx, Vec3f(j, i, 0) );
				unsigned faction = floor( rnd::uniform() * (sizeof(shellFactions)/sizeof(*shellFactions)) );
				allegiances[idx] = faction;
				c = shellFactions[faction];
				//cout << shellFactions[faction].components << endl;
			}   //RGB(rnd::uniform(),rnd::uniform(),rnd::uniform());
	
			c.a = 0.5;
			shellPix[j * Nx + i] = c;
		}
	}
	shellTex.submit(shellPix.data());
	*/

	// initialize the boid spheres
	for (unsigned i = 0; i < NSpheres; i++) {
		// initialize the bubble object stuff
		bubbles[i].space = HashSpace(4, boidsPerSphere);
		bubbles[i].center = Vec3f(bubbles[i].space.dim()/2, bubbles[i].space.dim()/2, bubbles[i].space.dim()/2);
		bubbles[i].type = i == 0 ? sphereType::THREEDVIDEO : sphereType::BOID;
		bubbles[i].origin = Vec3f(sphereRingR * cos(i * 360/NSpheres), 0, sphereRingR * sin(i * 360/NSpheres));		

		// create the sphere shell
		spheres[i].primitive(Mesh::TRIANGLES);
		addIcosphere(spheres[i], bubbles[i].space.maxRadius(), 4);
		spheres[i].decompress();
		spheres[i].generateNormals();
	
		/*
		// add texture coordinates
		spheres[i].texCoord(0, 1);
		spheres[i].texCoord(0, 0);
		spheres[i].texCoord(1, 1);
		spheres[i].texCoord(1, 0);
		*/

		// move the sphere to its origin point
		vector<Vec3f>& verts(spheres[i].vertices());
		for (unsigned j = 0; j < verts.size(); j++) {		
			verts[j] += bubbles[i].origin + bubbles[i].center;
			spheres[i].color((float)j / verts.size(), 0, 0, shellTransparency);
		}
	}

	// Figure out some hashspace related globals
	maxradius = bubbles[0].space.maxRadius();
	floatDim = (float)bubbles[0].space.dim();
	hashCenter = Vec3f(floatDim/2, floatDim/2, floatDim/2);

	// initialize all the points in the mesh
	boids.primitive(Mesh::POINTS);
	if(cuttleboneDomain->isSender()){
		for (unsigned i = 0; i < NSpheres; i++) {
			for (unsigned id = 0; id < boidsPerSphere; id++) {
				// put the boid position in the hash space
				Vec3f initialPos = Vec3f(rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1).normalize(rnd::uniform() * maxradius) + bubbles[i].center;	
				bubbles[i].space.move(id, initialPos);
				HashSpace::Object& o = bubbles[i].space.object(id);	
				
				// store some simulation state for the agent
				bubbles[i].agents[id].mass = rnd::uniform() * 10 + 1;
				bubbles[i].agents[id].velocity = Vec3f(0,0,0);
				bubbles[i].agents[id].acceleration = Vec3f(0,0,0);
				
				// put the boid position, color, and size in the mesh
				boids.vertex(o.pos+bubbles[i].origin);
				boids.color(0, 1, 0);
				boids.texCoord(pow(bubbles[i].agents[id].mass,1/3), 0);

				// create the trail mesh for this boid
				for (unsigned j = 0; j < trailLength; j++) {
					trails[i * boidsPerSphere + id].vertex(o.pos + bubbles[i].origin);
				}
			}
		}
		kinectVoxels.primitive(Mesh::POINTS);
		for (unsigned i = 0; i < NKinect; i++) {
			// fill the kinect voxels with default x and y coordinates which won't really change
			kinectVoxels.vertex(floor(i / kinectW),i % kinectW,0);
			kinectVoxels.color(0,0,0);
		}
	}else{
		// on the receiving end we will need a place for all the 300,000 or so kinect voxels
		for (unsigned i = 0; i < NBoids; i++) {
			boids.vertex(0,0,0);
			boids.color(0,0,0);
			boids.texCoord(5,0);
		}
		for (unsigned i = 0; i < NKinect; i++) {
			kinectVoxels.vertex(0,0,0);
			kinectVoxels.color(0,1,0);
			kinectVoxels.texCoord(2,0);
		}
	}

  }

  void onAnimate(double dt) override {
	  dt = timeStep;
	  if(cuttleboneDomain->isSender()){
		  // update the boids mesh
		  vector<Vec3f>& position(boids.vertices());
		  vector<al::Color>& colors(boids.colors());
		  for (unsigned i = 0; i < NSpheres; i++){
			  // skip this sphere's update if the kinect is streaming and there's no kinect data there
			  if (kinectStreaming && i != kinectSphere) {
				continue;
			  }
			  for (unsigned id = 0; id < boidsPerSphere; id++) {		
				// collect boid from hashspace
				HashSpace::Object& o = bubbles[i].space.object(id);

				// find flock mates
				qmany.clear();
				int flockMates = qmany(bubbles[i].space, o.pos, KDist * floatDim);
				
				// accumulate boid forces
				Vec3f boidForce = Vec3f(0,0,0);
				for(unsigned j = 1; j < flockMates; j++) {			
					Vec3f vec2other = qmany[j]->pos - o.pos;
					Vec3f vec2mid = vec2other / 2;
					boidForce += vec2mid * cohesionModifier; // cohesion
					boidForce -= vec2other * separationModifier; // separation
					Agent& B = bubbles[i].agents[qmany[j]->id];
					Agent& A = bubbles[i].agents[id];
					boidForce += Vec3f((B.velocity - A.velocity).normalize(alignmentModifier)); // alignment
				}
				
				// integrate forces into mesh position
				bubbles[i].agents[id].acceleration += boidForce / bubbles[i].agents[id].mass;
				bubbles[i].agents[id].velocity += bubbles[i].agents[id].acceleration * dt;
				bubbles[i].space.move(id, o.pos + (bubbles[i].agents[id].velocity * dt));

				// clearing acceleration
				bubbles[i].agents[id].acceleration.zero();

				// modulo space check
				Vec3f vec2Origin = hashCenter - o.pos;
				if(vec2Origin.mag() >= maxradius) { bubbles[i].space.move(id, o.pos + vec2Origin * 1.95); }

				// get index of boid in mesh
				unsigned idx = i * boidsPerSphere + id;

				// get world position
				Vec3f worldPos(o.pos + bubbles[i].origin);

				// update mesh position to hashspace + origin
				position[idx] = worldPos;	
				
	
				// updating shareState agents
				state().voxels[id].pos = position[idx];
				state().voxels[id].col = colors[idx];

				// update trail
				//Vec3f& trailPosition(trails[idx].vertices()[trailIdx]);
				//trailPosition = worldPos;
			  }
			  // update the texture for the shells
			  vector<al::Color>& sphereColors(spheres[i].colors()); 
			  for (unsigned i = 0; i < sphereColors.size(); i++) {
			  	if(rnd::prob(0.1)) { sphereColors[i] += rnd::uniform() * 2 - 1; sphereColors[i].a = shellTransparency; }
			  }
		}
		
		

		/*
		// update the texture for the shells
		shellTex.invalidate();
		shellTex.create2D(256, 256, Texture::RGBA8);
		int Nx = shellTex.width();
		int Ny = shellTex.height();
		shellPix.resize(Nx * Ny);
		for (unsigned j = 0; j < Ny; ++j) {
			for (unsigned i = 0; i < Nx; ++i) {
				unsigned idx = j * Nx + i;
				Color c = shellPix[idx];
				cout << "foo " << allegiances[idx] << endl;
				if ( allegiances[idx] == -1 ) {
					//cout << 1 << endl; 
					if (al::rnd::prob(0.1)) { // if we pass this check then we are spawning a thing 
						HashSpace::Object& o = shellSpace.object(idx);
						int numNearby = qmany(shellSpace, o.pos, 0.001 * shellSpace.dim());
						if (numNearby > 1) { allegiances[idx] = allegiances[qmany[1]->id]; c = shellPix[qmany[1]->id]; } // if it spawns close to something, add it to its faction
					}
				}

				c.a = 0.5;
				//cout << (int)shellPix[idx].r << (int)shellPix[idx].g << (int)shellPix[idx].b << endl;
				shellPix[idx] = c;
			}
		}
		shellTex.submit(shellPix.data());
		*/


	  }
	  trailIdx += 1;
	  trailIdx = trailIdx > trailLength * NBoids ? 0 : trailIdx;
	  //cout << trailIdx << endl;
  }

  void onDraw(Graphics& g) override {
	g.clear(0.0);

	gl::blending(true);
	gl::blendTrans();

	static Light light;
	g.lighting(true);
	g.light(light);

	//vector<Vec3f>& positions(kinectVoxels.vertices());
	//vector<al::Color>& colors(kinectVoxels.colors());
	for (unsigned i = 0; i < NSpheres; i++) {
		// draw the sphere shell meshes
		//shellTex.bind();
		//g.texture();
		gl::depthTesting(true);
		g.meshColor();
		g.draw(spheres[i]);
		//shellTex.unbind();		
	}

	
	gl::depthTesting(false);
	if (cuttleboneDomain->isSender()) {
		// if any of the spheres are marked for kinect data, change the mesh data according to that
		if (kinectStreaming) {
			// grab depth and rgb data from kinect
			kinectVoxels.reset();
			short *depth = 0;
			char *rgb = 0;
			uint32_t ts;
			if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
				no_kinect_quit();
			if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
				no_kinect_quit();

			// update mesh to reflect kinect point cloud
			int j,k;
			int n = 0;
			unsigned m = 0;
			for (j = 0; j < kinectH; j++) {
				for (k = 0; k < kinectW; k++) {	
					//double z = 0.1236 * tan((double)depth[i*kinectW+j] / 2842.5 + 1.1863);
					double z = ((double)depth[j*kinectW+k] - minimumDisparity) * (floatDim/(maximumDisparity - minimumDisparity));
					Vec3f pos(k / (float)kinectW * floatDim, j / (float)kinectW * floatDim, z);
				
					// if the voxel is outside the sphere or is a boid then just put this kinect data at the hashspace center
					if ((pos - hashCenter).mag() > maxradius || isBoid(m/boidifyStride)) { pos = hashCenter; }
					else {
						// if in the sphere, and not a boid, then update a corresponding point in the boid space
						if (m % boidifyStride == 0) { bubbles[kinectSphere].space.move(m / boidifyStride, pos); }
					}
	
					Vec3f worldPos(pos + bubbles[kinectSphere].origin);
					//if (isBoidified(m)) { worldPos = Vec3f(0,0,0); }
				
					kinectVoxels.vertex(worldPos);
					kinectVoxels.color(((float)rgb[n++]+128)/255, ((float)rgb[n++]+128)/255, ((float)rgb[n++]+128)/255, 1.0);
					kinectVoxels.texCoord(5, 0);

					// update state
					//Vec3f oldPos = position[i];
					state().voxels[m].pos = kinectVoxels.vertices()[m];
					state().voxels[m].col = kinectVoxels.colors()[m];
					
					// update m, which is the voxel index
					m = m + 1;
				}
			}
		}	
	}

	// TO DO: Add a state retrieval point here in an else statement so that the renderers will be able to pull the voxels from state.

	
	g.shader(pointShader);
	g.shader().uniform("pointSize", pointSize.get() / 100);
	if (kinectStreaming) { /*g.meshColor();*/ g.draw(kinectVoxels); } else { g.draw(boids); }

	
	
	for (unsigned id = 0; id < NBoids; id++) {
		//g.draw(trails[id]);
	}
	
	
	
	if(isPrimary()){
		gui.draw(g);
	}
  }

  bool onKeyDown(const Keyboard& k) override {
    //
    //printf("got here\n");
    return false;
  }

  bool onMouseMove(const Mouse& m) override {
    //
    //printf("%d,%d\n", m.x(), m.y());
    return false;
  }
};

int main() {
  MyApp app;
  app.start();
}

string slurp(string fileName) {
	fstream file(fileName);
	string returnValue = "";
	while (file.good()) {
		string line;
		getline(file, line);
		returnValue += line + "\n";
	}
	return returnValue;
}
