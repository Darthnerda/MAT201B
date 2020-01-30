#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)

using namespace al;

#include <fstream>
#include <vector>
using namespace std;

Vec3d rv(float scale) {
  //return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
	return Vec3d(rnd::normal(), rnd::normal(), rnd::normal()) * scale;
}

string slurp(string fileName);  // forward declaration

struct AlloApp : App {
  // add more GUI here
  Parameter pointSize{"/pointSize", "", 10.0, "", 0.0, 40.0};
  Parameter timeStep{"/timeStep", "", 0.1, "", 0.01, 20.0};
  Parameter gravityMultiplier{ "/gravityMultiplier", "", 1.0, "", 0.0, 6.0 };
  Parameter relaxedDistance{ "/relaxedDistance", "", 0.005, "", 0.001, 0.009 };
  Parameter stiffness{ "/stiffness", "", 6.0, "", 0.0, 20.0 };
  Parameter attachmentThreshold{ "/attachmentThreshold", "", 0.05, "", 0.01, 0.09 };
  
  ControlGUI gui;

  ShaderProgram pointShader;
  Mesh mesh;  // vector<Vec3f> position is inside mesh

  // typedef al::Vec<float, 3> Vec3f;
  // typedef std::vector<Vec3f> Vertices;

  // simulation constants
  const double GRAV_CONST = 1.6718705213575e+7; // in AU ^ 3 / solar mass * day ^ 2

  //  simulation state
  vector<Vec3f> velocity;
  vector<Vec3f> acceleration;
  vector<double> mass;
  vector<string> types;
  // attachments: first indice corresponds to the source entity, second indice corresponds to the target entity, both together return a bool of whether or not they connect
  //vector<vector<bool>> attachments;

  // simulation options
  bool drag = false;

  void onCreate() override {
    // add more GUI here
    gui << pointSize << timeStep << gravityMultiplier << relaxedDistance << stiffness << attachmentThreshold;
    gui.init();
    navControl().useMouse(false);

    // compile shaders
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    // set initial conditions of the simulation
    //

    // c++11 "lambda" function
    auto rc = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };

    mesh.primitive(Mesh::POINTS);
    
	/*
	// does 1000 work on your system? how many can you make before you get a low
    // frame rate? do you need to use <1000?
    for (int _ = 0; _ < 200; _++) {
	  // add entity type
	  float r = rnd::uniform();
	  string t;
	  float m;
	  if (r < 0.01) { // stars
		  t = "star";
		  mesh.vertex(rv(40));
		  mesh.color(HSV(rnd::uniform() * 0.10f + (rnd::uniform() > 0.5 ? 0.5 : 0), rnd::uniform() * 0.5f + 0.2f, rnd::uniform() * 0.3f + 0.7f));
		  m = 1 + rnd::normal() * 1000.0f; // so lets say that m is in decisuns
		  if (m < 0.5) m = 0.5;

		  velocity.push_back(rv(0.05));
		  acceleration.push_back(rv(0.05));
	  }
	  if (r >= 0.01 && r < 0.96) { // planets
		  t = "planet";
		  mesh.vertex(rv(20));
		  mesh.color(HSV(rnd::uniform() * 0.023f + 0.027, rnd::uniform() * 0.2f + 0.1f, rnd::uniform() * 0.30f + 0.25f));
		  m = 3 + rnd::normal(); // so lets say that m is in decisuns
		  if (m < 0.5) m = 0.5;

		  velocity.push_back(rv(0.1));
		  acceleration.push_back(rv(0.1));
	  }
	  if (r >= 0.96) { // life
		  t = "life";
		  mesh.vertex(rv(20));
		  mesh.color(HSV(0.27f, rnd::uniform() * 0.4 + 0.4, 1.0f));
		  m = 0.05 + rnd::normal() / 4; // so lets say that m is in decisuns
		  if (m < 0.1) m = 0.1;

		  velocity.push_back(rv(0.1));
		  acceleration.push_back(rv(0.1));
	  }
	  types.push_back(t);
      // float m = rnd::uniform(3.0, 0.5);
      mass.push_back(m);


      // using a simplified volume/size relationship
      //mesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t
	  // modified volume/size relationship
	  mesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t

      // separate state arrays
      
    }
	*/

	double m;

	// spawn sun
	mesh.vertex(Vec3f(0,0,0));
	mesh.color(HSV(rnd::uniform() * 0.10f + (rnd::uniform() > 0.5 ? 0.5 : 0), rnd::uniform() * 0.5f + 0.2f, rnd::uniform() * 0.3f + 0.7f));
	types.push_back("star");
	m = 1;
	mass.push_back(m);
	//mesh.texCoord(pow(m, 1.0f / 3), 0);
	mesh.texCoord(1, 0);
	velocity.push_back(Vec3f(0,0,0));
	acceleration.push_back(Vec3f(0,0,0));


	// spawn earth
	mesh.vertex(Vec3f(1, 0, 0)); // in astronomical units
	mesh.color(HSV(rnd::uniform() * 0.023f + 0.027, rnd::uniform() * 0.2f + 0.1f, rnd::uniform() * 0.30f + 0.25f));
	types.push_back("planet");
	m = 3.00348959632e-6; // in solar mass
	mass.push_back(m); 
	//mesh.texCoord(pow(m, 1.0f / 3), 0);
	mesh.texCoord(1, 0);
	velocity.push_back(Vec3f(0, 1.719914438502673e-2, 0)); // in AU / day
	acceleration.push_back(Vec3f(0, 1, 0));

	// spawn many earth-like planets
	for (int _ = 0; _ < 20; _++) {
		mesh.vertex(Vec3f(rnd::uniform() * 0.1 + 1, 0, 0)); // in astronomical units
		mesh.color(HSV(rnd::uniform() * 0.023f + 0.027, rnd::uniform() * 0.2f + 0.1f, rnd::uniform() * 0.30f + 0.25f));
		types.push_back("planet");
		m = rnd::uniform() * 1e-6 + 3.00348959632e-6; // in solar mass
		mass.push_back(m);
		//mesh.texCoord(pow(m, 1.0f / 3), 0);
		mesh.texCoord(1, 0);
		velocity.push_back(Vec3f(0, rnd::uniform() * 0.3e-2 + 1.719914438502673e-2, 0)); // in AU / day
		acceleration.push_back(Vec3f(0, 1, 0));
	}

	// spawn moon
	mesh.vertex(Vec3f(1.00256951871657754, 0, 0));
	mesh.color(HSV(rnd::uniform() * 0.023f + 0.027, rnd::uniform() * 0.2f + 0.1f, rnd::uniform() * 0.30f + 0.25f));
	types.push_back("planet");
	m = 3.694329684197e-8; // in solar mass
	mass.push_back(m);
	//mesh.texCoord(pow(m, 1.0f / 3), 0);
	mesh.texCoord(1, 0);
	velocity.push_back(Vec3f(0, 5.902459893048128342e-4, 0));
	acceleration.push_back(Vec3f(0, 1, 0));

    nav().pos(0, 0, 10);
  }

  bool freeze = false;
  void onAnimate(double dt) override {
    if (freeze) return;

    // ignore the real dt and set the time step;
    dt = timeStep;

    // Generic force modifier lambda
	auto getForce = [](vector<Vec3f>::iterator &A, vector<Vec3f>::iterator &B, int &IndA, int &IndB, double &force, string &typeA, string &typeB, string &forceType) {
		int magnifier = 1;
		// there are special magnifier rules for life
		if (typeA.compare("life") == 0) {
			if (typeB.compare("life") == 0) {
				if (forceType.compare("gravity") == 0) {
					magnifier = 0;
				}
				if (forceType.compare("hookes") == 0) {
					magnifier = -50;
				}
			}
			if (typeB.compare("planet") == 0) {
				if (forceType.compare("gravity") == 0) {
					magnifier = 0;
				}
				if (forceType.compare("hookes") == 0) {
					magnifier = 100;
				}
			}
			if (typeB.compare("star") == 0) {
				if (forceType.compare("gravity") == 0) {
					magnifier = 1;
				}
				if (forceType.compare("hookes") == 0) {
					magnifier = 1;
				}
			}
		}
		else {
			if (typeB.compare("life") == 0) {
				if (forceType.compare("gravity") == 0) {
					magnifier = 0;
				}
				if (forceType.compare("hookes") == 0) {
					magnifier = 0;
				}
			}
			else {
				if (forceType.compare("gravity") == 0) {
					magnifier = 1;
				}
				if (forceType.compare("hookes") == 0) {
					magnifier = 0;
				}
			}
		}

		return (*B - *A).normalize(magnifier * force);
	};
	
	// apply interactions
	for (vector<Vec3f>::iterator it = mesh.vertices().begin(); it != mesh.vertices().end(); ++it) {
		int itInd = it - mesh.vertices().begin();
		string itType = types[itInd];
		for (vector<Vec3f>::iterator that = mesh.vertices().begin() + itInd + 1; that != mesh.vertices().end(); ++that) {
			int thatInd = that - mesh.vertices().begin();
			// apply all interactivities
			double distance = dist(*it, *that);
			string thatType = types[thatInd];

			// apply hooke's force
			if (distance > relaxedDistance && distance < attachmentThreshold) { // if things are close enough to attach to one another
				double hookesForce = stiffness * (distance - relaxedDistance);// calc hook's force
				acceleration[itInd] += getForce(it, that, itInd, thatInd, hookesForce, itType, thatType, string("hookes")) / mass[itInd]; // and apply it ( * 1000 for kiloSeconds)
				acceleration[thatInd] += getForce(that, it, thatInd, itInd, hookesForce, thatType, itType, string("hookes")) / mass[thatInd];
			}

			// apply gravity force
			double gravForce = gravityMultiplier * GRAV_CONST * ((mass[itInd] * mass[thatInd]) / pow(distance, 1.5)); // calc gravity force
			acceleration[itInd] += getForce(it, that, itInd, thatInd, gravForce, itType, thatType, string("gravity")) / (mass[itInd] * 1000000000);
			acceleration[thatInd] += getForce(that, it, thatInd, itInd, gravForce, thatType, thatType, string("gravity")) / (mass[thatInd] * 1000000000);
		}

		// apply all global forces
		if (drag) {
			acceleration[itInd] -= velocity[itInd] * 0.1;
		}
	}
	

    // Vec3f has
    // • +=
    // • -=
    // • .normalize()
    // • .normalize(float scale)
    // • .mag()
    // • .magSqr()
    // • .dot(Vec3f f)
    // • .cross(Vec3f f)

    // Integration
    //
    vector<Vec3f>& position(mesh.vertices());
    for (int i = 0; i < velocity.size(); i++) {
      // "backward" Euler integration
      velocity[i] += acceleration[i] * dt;
      position[i] += velocity[i] * dt;

      // Explicit (or "forward") Euler integration would look like this:
      // position[i] += velocity[i] * dt;
      // velocity[i] += acceleration[i] / mass[i] * dt;
    }

	cout << velocity[1] << " " << acceleration[1] << " " << mesh.vertices()[1] << endl;

    // clear all accelerations (IMPORTANT!!)
    for (auto& a : acceleration) a.zero();
  }

  bool onKeyDown(const Keyboard& k) override {
    if (k.key() == ' ') {
      freeze = !freeze;
    }

    if (k.key() == '1') {
      // introduce some "random" forces
      for (int i = 0; i < velocity.size(); i++) {
        // F = ma
        acceleration[i] = rv(1) / mass[i];
      }
    }

	if (k.key() == '2') {
		drag = true;
	}

    return true;
  }

  void onDraw(Graphics& g) override {
    g.clear(0.0);
    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100);
    g.blending(true);
    g.blendModeTrans();
    g.depthTesting(true);
    g.draw(mesh);
    gui.draw(g);
  }
};

int main() { AlloApp().start(); }

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