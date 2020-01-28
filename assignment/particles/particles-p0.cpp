#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)

using namespace al;

#include <fstream>
#include <vector>
using namespace std;

Vec3f rv(float scale) {
  return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
}

string slurp(string fileName);  // forward declaration

struct AlloApp : App {
  // add more GUI here
  Parameter pointSize{"/pointSize", "", 1.0, "", 0.0, 2.0};
  Parameter timeStep{"/timeStep", "", 0.1, "", 0.01, 0.6};
  Parameter gravityMultiplier{ "/gravityConstant", "", 1.0, "", 0.0, 6.0 };
  Parameter relaxedDistance{ "/relaxedDistance", "", 0.005, "", 0.001, 0.009 };
  Parameter stiffness{ "/stiffness", "", 6.0, "", 0.0, 20.0 };
  Parameter attachmentThreshold{ "/attachmentThreshold", "", 0.05, "", 0.01, 0.09 };
  
  ControlGUI gui;

  ShaderProgram pointShader;
  Mesh mesh;  // vector<Vec3f> position is inside mesh

  // typedef al::Vec<float, 3> Vec3f;
  // typedef std::vector<Vec3f> Vertices;

  // simulation constants
  const float GRAV_CONST = 0.00006674301; // removing 6 orders of magnitude to adjust for sun mass

  //  simulation state
  vector<Vec3f> velocity;
  vector<Vec3f> acceleration;
  vector<float> mass;
  vector<string> types;
  // attachments: first indice corresponds to the source entity, second indice corresponds to the target entity, both together return a bool of whether or not they connect
  //vector<vector<bool>> attachments;

  // simulation options
  bool drag = false;

  // state pool
  vector<vector<float>> distances;

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
    // does 1000 work on your system? how many can you make before you get a low
    // frame rate? do you need to use <1000?
    for (int _ = 0; _ < 1000; _++) {
      mesh.vertex(rv(5));
	  // add entity type
	  float r = rnd::uniform();
	  string t;
	  float m;
	  if (r < 0.96) {
		  t = "celestial";
		  mesh.color(HSV(rnd::uniform() * 0.10f, rnd::uniform() * 0.6f + 0.1f, rnd::uniform() * 0.8f + 0.2f));
		  m = 10 + rnd::normal() / 1.5f; // so lets say that m is in suns
		  if (m < 0.5) m = 0.5;
	  }
	  else {
		  t = "life";
		  mesh.color(HSV(0.27f, rnd::uniform(), 1.0f));
		  m = 1 + rnd::normal() / 2; // so lets say that m is in suns
		  if (m < 0.1) m = 0.1;
	  }
	  types.push_back(t);
      // float m = rnd::uniform(3.0, 0.5);
      mass.push_back(m);


      // using a simplified volume/size relationship
      //mesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t
	  // modified volume/size relationship
	  mesh.texCoord(pow(m, 1.0f / 2), 0);  // s, t

      // separate state arrays
      velocity.push_back(rv(0.1));
      acceleration.push_back(rv(1));
    }

	// allocate space for the attachments truth pool's first indice
	//attachments.resize(mesh.vertices().size());

	// initialize the attachments truth pool to false 
	cout << "foo" << endl;
	//for (vector<Vec3f>::iterator it = mesh.vertices().begin(); it != mesh.vertices().end(); ++it) {
	//	int itInd = it - mesh.vertices().begin();
	//	// allocate space for the attachments truth pool's second indice
	//	attachments[itInd].resize(mesh.vertices().size()-itInd);
	//	for (vector<Vec3f>::iterator that = mesh.vertices().begin(); that != mesh.vertices().end(); ++that) {
	//		int thatInd = that - mesh.vertices().begin();
	//		if (thatInd > itInd) {
	//			attachments[itInd][thatInd] = false;
	//		}
	//	}
	//}

	cout << "bar" << endl;

    nav().pos(0, 0, 10);
  }

  bool freeze = false;
  void onAnimate(double dt) override {
    if (freeze) return;

    // ignore the real dt and set the time step;
    dt = timeStep;

    // Calculate forces

    // pair-wise and equal but opposite
    // nested for loop to visit each pair once
    // O(n*n)
    //



    // drag

	auto getForce = [](vector<Vec3f>::iterator &A, vector<Vec3f>::iterator &B, int &IndA, int &IndB, float &force, string &type, string &forceType) {
		int magnifier = 1;
		// there are special magnifier rules for life
		if (type.compare("life") == 0) {
			if (forceType.compare("gravity") == 0) {
				magnifier = 5;
			}
			if (forceType.compare("hookes") == 0) {
				magnifier = 10;
			}
		}
		return (*B - *A).normalize(magnifier * force);
	};
	
	// apply interactions
	for (vector<Vec3f>::iterator it = mesh.vertices().begin(); it != mesh.vertices().end(); ++it) {
		int itInd = it - mesh.vertices().begin();
		string itType = types[itInd];
		for (vector<Vec3f>::iterator that = mesh.vertices().begin(); that != mesh.vertices().end(); ++that) {
			int thatInd = that - mesh.vertices().begin();
			// apply all interactivities
			if (thatInd > itInd) {
				//cout << to_string(itInd) << " " << to_string(thatInd) << endl;
				//cout << to_string(itInd) << " " << to_string(thatInd) << endl;
				float distance = dist(*it, *that);
				//cout << "buzz" << endl;
				//cout << "calculated distance for " << to_string(itInd) << " and " << to_string(thatInd) << endl;
				string thatType = types[thatInd];

				//if(!attachments[itInd][thatInd]) { // if they're not attached
				//	if (distance < attachmentThreshold && distance > relaxedDistance) { // and they're within a threshold of distance
				//		if (rnd::uniform() > 0.65) { // and they pass a randomness check
				//			//cout << "attached " << to_string(itInd) << " and " << to_string(thatInd) << endl;
				//			attachments[itInd][thatInd] = true; // set them to be attached
				//		}
				//	}
				//}

				// apply the hook force if applicable
				//if (attachments[itInd][thatInd]) { // if they are attached
				//	float relaxedDistance = 0.005;
				//	if (distance < relaxedDistance) { // and they are within the relaxed distance
				//		attachments[itInd][thatInd] = false; // set them to be detached
				//	}
				//	else { // otherwise
				//		float hookesForce = stiffness * (distance - relaxedDistance);// calc hook's force
				//		acceleration[itInd] += getForce(it, that, itInd, thatInd, hookesForce, itType, string("hookes")); // and apply it
				//		acceleration[thatInd] += getForce(that, it, thatInd, itInd, hookesForce, thatType, string("hookes"));
				//	}
				//}

				if (distance > relaxedDistance && distance < attachmentThreshold) {
					float hookesForce = stiffness * (distance - relaxedDistance);// calc hook's force
					acceleration[itInd] += getForce(it, that, itInd, thatInd, hookesForce, itType, string("hookes")); // and apply it
					acceleration[thatInd] += getForce(that, it, thatInd, itInd, hookesForce, thatType, string("hookes"));
				}


				// apply gravity force
				float gravForce = gravityMultiplier * GRAV_CONST * ((mass[itInd] * mass[thatInd]) / pow(distance, 2)); // calc gravity force
				acceleration[itInd] += getForce(it, that, itInd, thatInd, gravForce, itType, string("gravity")) / mass[itInd];
				acceleration[thatInd] += getForce(that, it, thatInd, itInd, gravForce, thatType, string("gravity")) / mass[thatInd];
			}
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
      velocity[i] += acceleration[i] / mass[i] * dt;
      position[i] += velocity[i] * dt;

      // Explicit (or "forward") Euler integration would look like this:
      // position[i] += velocity[i] * dt;
      // velocity[i] += acceleration[i] / mass[i] * dt;
    }

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