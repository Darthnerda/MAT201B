#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)
#include "al/ui/al_Parameter.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "al/spatial/al_HashSpace.hpp"
using namespace al;
using namespace std;
#include <vector>

string slurp(string fileName);

struct ShareAgent {
  Vec3f pos;
  al::Color col;
};

HashSpace space(6, 10000);
unsigned maxradius = space.maxRadius();
HashSpace::Query qmany(500);

const int N = 10000;
struct SharedState {
  ShareAgent agent[N];
};

enum AgentType {BOID, NOTBOID};

struct Agent {
	Vec3f velocity;
	Vec3f acceleration;
	double mass;
	AgentType agentType;
};

Vec3f boidForce(auto context, float distance, Vec3f vec2mid, Agent A, Agent B);

struct MyApp : public DistributedAppWithState<SharedState> {
  Mesh mesh;

  // simulation state
  Agent agents[N];

  Parameter timeStep{ "/timeStep", "", 0.01, "", 0.0000001, 1.0 };
  Parameter pointSize{ "/pointSize", "", 1.0, "", 0.0, 40.0 };
  Parameter sepDistThresh{ "/sepDistThresh","",0.3,"",0.01,0.9 };
  Parameter cohDistThresh{ "/cohDistThresh","",0.5,"",0.01,0.9 };
  Parameter alignDistThresh{ "/alignDistThresh","",0.5,"",0.01,0.9 };
  Parameter cohesionModifier{ "/cohesionModifier","",0.6,"",0.1,10 };
  Parameter separationModifier{ "/separationModifier","",0.6,"",0.1,10 };
  Parameter alignmentModifier{ "/alignmentModifier","",1.6,"",0,10 };
  
  ControlGUI gui;
  ShaderProgram pointShader; 

  std::shared_ptr<CuttleboneStateSimulationDomain<SharedState>> cuttleboneDomain;

  void onCreate() override {
        // load cuttleboan
	cuttleboneDomain = CuttleboneStateSimulationDomain<SharedState>::enableCuttlebone(this);
	if(!cuttleboneDomain){
		std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
		quit();
	}	
	
	// compile shaders
	pointShader.compile(slurp("../point-vertex.glsl"),
		slurp("../point-fragment.glsl"),
		slurp("../point-geometry.glsl"));

	// pipe in gui stuff
	gui << pointSize << sepDistThresh << cohDistThresh << alignDistThresh << cohesionModifier << separationModifier << alignmentModifier << timeStep;
	gui.init();

	// pipe in gui stuff to the parameter server
	parameterServer()  << pointSize << sepDistThresh << cohDistThresh << alignDistThresh << cohesionModifier << separationModifier << alignmentModifier << timeStep;

	// prep navigation stuff
	navControl().useMouse(false);
	nav().pos(0, 0, 10);

	// initialize all the agents
	mesh.primitive(Mesh::POINTS);
	for (unsigned id = 0; id < N; id++) {
		if(cuttleboneDomain->isSender()){
			space.move(id, space.dim() * rnd::uniform(), space.dim() * rnd::uniform(), space.dim() * rnd::uniform());
			HashSpace::Object& o = space.object(id);
			mesh.vertex(o.pos);
			mesh.color(0, 1, 0);
			
			agents[id].mass = rnd::uniform() * 10 + 1;
			agents[id].velocity = Vec3f(0,0,0);
			agents[id].acceleration = Vec3f(0,0,0);
			agents[id].agentType = BOID;

			mesh.texCoord(pow(agents[id].mass,1/3), 0);	}
		else{
			mesh.vertex(0,0,0);
			mesh.color(0,1,0);
		}
	}

  }

  void onAnimate(double dt) override {
	  dt = timeStep;
	  if(cuttleboneDomain->isSender()){
		  // calculate physics
		  vector<Vec3f>& position(mesh.vertices());
		  vector<al::Color>& colors(mesh.colors());
		  for (unsigned id = 0; id < N; id++) {
		  	HashSpace::Object& o = space.object(id);
			qmany.clear();
			int cohesionMates = qmany(space, o.pos, cohDistThresh);
			Vec3f boidForce = Vec3f(0,0,0);
			for(unsigned i = 0; i < cohesionMates; i++) {			
				Vec3f vec2mid = (qmany[i]->pos - o.pos) / 2;
				boidForce += vec2mid * cohesionModifier;
			}
			qmany.clear();
			int separationMates = qmany(space, o.pos, cohDistThresh);
			for(unsigned i = 0; i < separationMates; i++) {	
				Vec3f vec2mid = (qmany[i]->pos - o.pos) / 2;
				boidForce -= vec2mid * separationModifier;
			}
			qmany.clear();
			int alignmentMates = qmany(space, o.pos, alignDistThresh);
			for(unsigned i = 0; i < alignmentMates; i++) {	
				Agent& B = agents[qmany[i]->id];
				Agent& A = agents[id];
				boidForce += (B.velocity.normalized() - A.velocity.normalized()).normalize(alignmentModifier);
			}
			agents[id].acceleration += boidForce / agents[id].mass;
			// integration
			agents[id].velocity += agents[id].acceleration * dt;
			position[id] += agents[id].velocity * dt;
			// clearing acceleration
			agents[id].acceleration.zero();
			// updating shareState agents
			state().agent[id].pos = position[id];
			state().agent[id].col = colors[id];

		  }

		  /*
		  for (int i = 0; i < N; i++) {
			// calculate interactions
			for (int j = i + 1; j < N; j++) {
				double distance = dist(mesh.vertices()[i], mesh.vertices()[j]);
				Vec3f force = Vec3f(0,0,0); 
				if(agents[i].agentType == BOID){
					Vec3f vec2mid = (position[j] - position[i]) / 2;
					force += boidForce(this, distance, vec2mid, agents[i], agents[j]);	
				}
				agents[i].acceleration += force / agents[i].mass;
				agents[j].acceleration -= force / agents[j].mass;
			}
			// integration
			agents[i].velocity += agents[i].acceleration * dt;
			position[i] += agents[i].velocity * dt;
			// clearing acceleration
			agents[i].acceleration.zero();
			// updating shareState agents
			state().agent[i].pos = position[i];
			state().agent[i].col = colors[i];
		  }
		  */

		  // clear all accelerations
		  //for (auto& a : acceleration) a.zero();
		  
	  }else{
		// record boid position and color data from shared state
		vector<Vec3f>& position(mesh.vertices());
		vector<al::Color>& colors(mesh.colors());
		for(int i = 0; i < N; i++){
			//Vec3f oldPos = position[i];
			position[i] = state().agent[i].pos;
			colors[i] = state().agent[i].col;
		} 	
	  }

  }

  void onDraw(Graphics& g) override {
    //
	g.clear(0.0);
	//g.shader(pointShader);
	//g.shader().uniform("pointSize", pointSize.get() / 100);
	//g.blending(true);
	//g.blendModeTrans();
	//g.depthTesting(true);
	
	//cout << mesh.vertices()[0] << endl;
	//cout << nav().uf() << endl;
	g.draw(mesh);
	
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

Vec3f boidForce(auto context, float distance, Vec3f vec2mid, Agent A, Agent B) {
	Vec3f boidForce = Vec3f(0,0,0);
	//cout << avgResultant.normalized() << endl;
	//cout << context->cohesionModifier << endl;
	if (distance < context->cohDistThresh) { boidForce += vec2mid * context->cohesionModifier;	}

	if (distance < context->sepDistThresh) { boidForce -= vec2mid * context->separationModifier; }

	if (distance < context->alignDistThresh) { boidForce += (B.velocity.normalized() - A.velocity.normalized()).normalize(context->alignmentModifier); }

	return boidForce;
}
