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
const int trailLength = 40;
unsigned trailIdx = 0;
const int NBoids = 6000;
const int NSpheres = 10;
const int sphereRingR = 25;
const int kinectW = 640;
const int kinectH = 480;
const int maxBoidNeighbors = 100;
// end tweak zone

// calculate some constants
const int NKinect = kinectW * kinectH;
const int boidifyStride = floor(NKinect / NBoids);
const int boidsPerSphere = NBoids / NSpheres;
HashSpace::Query qmany(maxBoidNeighbors);

// early globals declarations
unsigned maxradius;
float floatDim;
Vec3f hashCenter;

// early function declarations
string slurp(string fileName);

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
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

struct ShareAgent {
  Vec3f pos;
  al::Color col;
};

struct SharedState {
  ShareAgent agents[NKinect];
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
  Mesh mesh;
  Mesh trails[NBoids];
  Mesh spheres[NSpheres];
  Texture tex;

  // declare simulation structs
  //Agent agents[NBoids];
  Bubble bubbles[NSpheres];

  // initialize some parameters
  Parameter timeStep{ "/timeStep", "", 0.1, "", 0.0000001, 0.1 };
  Parameter pointSize{ "/pointSize", "", 1.0, "", 0.0, 40.0 };
  Parameter KDist{ "/KDist","",0.13,"",0.000001,1.0 };
  Parameter cohesionModifier{ "/cohesionModifier","",0.60,"",0.1,10 };
  Parameter separationModifier{ "/separationModifier","",0.7,"",0.1,10 };
  Parameter alignmentModifier{ "/alignmentModifier","",0.96,"",0,10 };
  
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

	// create semi transparent texture for the bubble shells
	tex.create2D(256, 256, Texture::RGBA8);
	int Nx = tex.width();
	int Ny = tex.height();
	vector<Colori> pix;
	pix.resize(Nx * Ny);
	for (unsigned j = 0; j < Ny; ++j) {
		for (unsigned i = 0; i < Nx; ++i) {
			Color c = RGB(1,0,0);
			c.a = 0.5;
			pix[j * Nx + i] = c;
		}
	}
	tex.submit(pix.data());

	// compile shaders
	pointShader.compile(slurp("../point-vertex.glsl"),
		slurp("../point-fragment.glsl"),
		slurp("../point-geometry.glsl"));

	// pipe parameters into the gui
	gui << pointSize << KDist << cohesionModifier << separationModifier << alignmentModifier << timeStep;
	gui.init();

	// pipe parameters to the shared parameter server
	parameterServer() << pointSize << KDist << cohesionModifier << separationModifier << alignmentModifier << timeStep;

	// prep navigation stuff
	navControl().useMouse(false);
	nav().pos(Vec3f(0,0,25));

	// initialize the boid spheres
	for (unsigned i = 0; i < NSpheres; i++) {
		// initialize the bubble object stuff
		bubbles[i].space = HashSpace(4, boidsPerSphere);
		bubbles[i].center = Vec3f(bubbles[i].space.dim()/2, bubbles[i].space.dim()/2, bubbles[i].space.dim()/2);
		bubbles[i].type = sphereType::BOID;
		bubbles[i].origin = Vec3f(sphereRingR * cos(i * 360/NSpheres), 0, sphereRingR * sin(i * 360/NSpheres));		

		// create the sphere shell
		spheres[i].primitive(Mesh::TRIANGLES);
		addIcosphere(spheres[i], bubbles[i].space.maxRadius(), 4);
		spheres[i].decompress();
		spheres[i].generateNormals();
		vector<Vec3f>& verts(spheres[i].vertices());
		for (unsigned j = 0; j < verts.size(); j++) {		
			verts[j] += bubbles[i].origin + bubbles[i].center;
		}
	}

	// Generate some hashspace related constants
	maxradius = bubbles[0].space.maxRadius();
	floatDim = (float)bubbles[0].space.dim();
	const Vec3f hashCenter(floatDim/2, floatDim/2, floatDim/2);

	// initialize all the points in the mesh
	mesh.primitive(Mesh::POINTS);
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
				mesh.vertex(o.pos+bubbles[i].origin);
				mesh.color(0, 1, 0);
				mesh.texCoord(pow(bubbles[i].agents[id].mass,1/3), 0);

				// create the trail mesh for this boid
				for (unsigned j = 0; j < trailLength; j++) {
					trails[id].vertex(0,0,0);
				}
			}
		}
	}else{
		// on the receiving end we will need a place for all the 300,000 or so kinect voxels
		for (unsigned i = 0; i < NKinect; i++) {
			mesh.vertex(0,0,0);
			mesh.color(0,1,0);
			mesh.texCoord(5,0);
		}
	}

  }

  void onAnimate(double dt) override {
	  dt = timeStep;
	  if(cuttleboneDomain->isSender()){
		  // update the meshes
		  vector<Vec3f>& position(mesh.vertices());
		  vector<al::Color>& colors(mesh.colors());
		  for (unsigned i = 0; i < NSpheres; i++){
			  // skip this sphere's update if there's supposed to be kinect data there
			  if (bubbles[i].type == THREEDVIDEO) {
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

				// get index of boid in mesh
				unsigned idx = i * boidsPerSphere + id;
				
				// integrate forces into mesh position
				bubbles[i].agents[id].acceleration += boidForce / bubbles[i].agents[id].mass;
				bubbles[i].agents[id].velocity += bubbles[i].agents[id].acceleration * dt;
				bubbles[i].space.move(id, o.pos + (bubbles[i].agents[id].velocity * dt));

				// clearing acceleration
				bubbles[i].agents[id].acceleration.zero();

				// modulo space check
				Vec3f vec2Origin = hashCenter - o.pos;
				if(vec2Origin.mag() >= maxradius) { bubbles[i].space.move(id, o.pos + vec2Origin * 1.95); }

				// update mesh position to hashspace + origin
				position[idx] = o.pos + bubbles[i].origin;	
				
				// sync hashspace position with mesh position; Subtracting from origin puts it into the space particular to the bubble's hashspace.
				//bubbles[i].space.move(id, position[idx] - bubbles[i].origin);
				
				// updating shareState agents
				state().agents[id].pos = position[idx];
				state().agents[id].col = colors[idx];

				// update trail
				//Vec3f& trailPosition(trails[idx].vertices()[trailIdx]);
				//trailPosition = o.pos;
			  }
		}
	  }else{
		// record boid position and color data from shared state
		vector<Vec3f>& position(mesh.vertices());
		vector<al::Color>& colors(mesh.colors());
		for(int i = 0; i < NKinect; i++){
			//Vec3f oldPos = position[i];
			position[i] = state().agents[i].pos;
			colors[i] = state().agents[i].col;
		}
	  }
	  trailIdx += 1;
  }

  void onDraw(Graphics& g) override {
	g.clear(0.0);

	gl::blending(true);
	gl::blendTrans();

	static Light light;
	g.lighting(true);
	g.light(light);

	for (unsigned i = 0; i < NSpheres; i++) {
		// draw the sphere shell meshes
		tex.bind();
		g.texture();
		g.draw(spheres[i]);
		tex.unbind();	

		// if any of the spheres are marked for kinect data, change the mesh data according to that
		if (bubbles[i].type == THREEDVIDEO) {
			// grab depth and rgb data from kinect
			mesh.reset();
			short *depth = 0;
			char *rgb = 0;
			uint32_t ts;
			if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
				no_kinect_quit();
			if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
				no_kinect_quit();

			// update mesh to reflect kinect point cloud
			int i,j;
			int n = 0;
			int m = 0;
			for (i = 0; i < kinectH; i++) {
				for (j = 0; j < kinectW; j++) {	
					//double z = 0.1236 * tan((double)depth[i*kinectW+j] / 2842.5 + 1.1863);
					double z = ((double)depth[i*kinectW+j] - 500) * (floatDim/(1100 - 500));
					Vec3f pos(j / (float)kinectW * floatDim, i / (float)kinectW * floatDim, z);
					mesh.vertex(pos + bubbles[i].origin);
					mesh.color(Color(((float)rgb[n++]+128)/255, ((float)rgb[n++]+128)/255, ((float)rgb[n++]+128)/255, 1.0));
					mesh.texCoord(1, 0);
				}
			}	
		}
	}
	
	g.shader(pointShader);
	g.shader().uniform("pointSize", pointSize.get() / 100);
	//gl::depthTesting(true);
	g.draw(mesh);

	
	/*
	for (unsigned id = 0; id < N; id++) {
		g.draw(trails[id]);
	}
	*/
	
	
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