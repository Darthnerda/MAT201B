#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)

using namespace al;

#include <fstream>
#include <vector>
#include <deque>
using namespace std;

enum ForceType { GRAVITY, HOOKES };
enum BodyType { STAR, PLANET, LIFE };

string slurp(string fileName);  // forward declaration
float getMagnifier(BodyType, BodyType, ForceType);

struct AlloApp : App {
	// add more GUI here
	Parameter pointSize{ "/pointSize", "", 10.0, "", 0.0, 40.0 };
	Parameter timeStep{ "/timeStep", "", 0.1, "", 0.01, 20.0 };
	Parameter gravityMultiplier{ "/gravityMultiplier", "", 1.0, "", 0.0, 6.0 };
	Parameter relaxedDistance{ "/relaxedDistance", "", 0.005, "", 0.001, 0.009 };
	Parameter stiffness{ "/stiffness", "", 6.0, "", 0.0, 20.0 };
	Parameter attachmentThreshold{ "/attachmentThreshold", "", 0.05, "", 0.01, 0.09 };
	Parameter sepDistThresh{ "/sepDistThresh","",0.1,"",0.01,0.9 };
	Parameter cohDistThresh{ "/cohDistThresh","",0.1,"",0.01,0.9 };
	Parameter alignDistThresh{ "/alignDistThresh","",0.1,"",0.01,0.9 };

	ControlGUI gui;

	ShaderProgram pointShader;
	Mesh mesh;  // vector<Vec3f> position is inside mesh
	Mesh trails;

	// simulation constants
	const double GRAV_CONST = 1.6718705213575e+7; // in AU ^ 3 / solar mass * day ^ 2

	//  simulation state
	vector<Vec3f> velocity;
	vector<Vec3f> acceleration;
	vector<double> mass;
	vector<BodyType> types;
	vector<Vec3f> direction;

	//
	int nextTrailHead = 0;

	// simulation options
	bool drag = false;

	void onCreate() override {
		// add more GUI here
		gui << pointSize << timeStep << gravityMultiplier << relaxedDistance << stiffness << attachmentThreshold << sepDistThresh << cohDistThresh << alignDistThresh;
		gui.init();
		navControl().useMouse(false);

		// compile shaders
		pointShader.compile(slurp("../point-vertex.glsl"),
			slurp("../point-fragment.glsl"),
			slurp("../point-geometry.glsl"));

		// set initial conditions of the simulation
		//

		mesh.primitive(Mesh::POINTS);
		trails.primitive(Mesh::POINTS);

		double m;

		// spawn sun
		mesh.vertex(Vec3f(0, 0, 0));
		mesh.color(HSV(rnd::uniform() * 0.10f + (rnd::uniform() > 0.5 ? 0.5 : 0), rnd::uniform() * 0.5f + 0.2f, rnd::uniform() * 0.3f + 0.7f));
		types.push_back(BodyType::STAR);
		m = 1;
		mass.push_back(m);
		//mesh.texCoord(pow(m, 1.0f / 3), 0);
		mesh.texCoord(1, 0);
		velocity.push_back(Vec3f(0, 0, 0));
		direction.push_back(velocity.back().normalized());
		acceleration.push_back(Vec3f(0, 0, 0));

		// spawn many earth-like planets
		for (int _ = 0; _ < 20; _++) {
			mesh.vertex(Vec3f(rnd::uniform() * 0.1 + 1, 0, 0)); // in astronomical units
			mesh.color(HSV(rnd::uniform() * 0.023f + 0.027, rnd::uniform() * 0.2f + 0.1f, rnd::uniform() * 0.30f + 0.25f));
			types.push_back(BodyType::PLANET);
			m = rnd::uniform() * 1e-6 + 3.00348959632e-6; // in solar mass
			mass.push_back(m);
			//mesh.texCoord(pow(m, 1.0f / 3), 0);
			mesh.texCoord(0.6, 0);
			velocity.push_back(Vec3f(0, rnd::uniform() * 0.3e-2 + 1.719914438502673e-2, 0)); // in AU / day
			direction.push_back(velocity.back().normalized());
			acceleration.push_back(Vec3f(0, 1, 0));
		}

		// spawn many moon-like planets
		for (int _ = 0; _ < 200; _++) {
			mesh.vertex(Vec3f(rnd::uniform() * 0.1 + 1.00256951871657754, 0, 0)); // in astronomical units
			mesh.color(HSV(rnd::uniform() * 0.023f + 0.027, rnd::uniform() * 0.2f + 0.1f, rnd::uniform() * 0.30f + 0.25f));
			types.push_back(BodyType::PLANET);
			m = rnd::uniform() * 1e-6 + 3.694329684197e-8; // in solar mass
			mass.push_back(m);
			//mesh.texCoord(pow(m, 1.0f / 3), 0);
			mesh.texCoord(0.3, 0);
			velocity.push_back(Vec3f(0, rnd::uniform() * 0.3e-4 + 5.902459893048128342e-4, 0)); // in AU / day
			direction.push_back(velocity.back().normalized());
			acceleration.push_back(Vec3f(0, 1, 0));
		}

		// spawn life!
		for (int _ = 0; _ < 200; _++) {
			mesh.vertex(Vec3f(rnd::uniform(), rnd::uniform(), rnd::uniform()));
			mesh.color(HSV(0.33, rnd::uniform()*0.1f + 0.9f, 1.0f));
			types.push_back(BodyType::LIFE);
			m = 2.0e-8; // in solar mass (an incredibly massive spaceship)
			mass.push_back(m);
			mesh.texCoord(0.1, 0);
			velocity.push_back(Vec3f(0, rnd::uniform() * 0.3e-4 + 5.902459893048128342e-4, 0));
			direction.push_back(velocity.back().normalized());
			acceleration.push_back(Vec3f(0, 1, 0));
		}

		// prepare the trails mesh
		int historySize = 2000;
		for (int i = 0; i < mesh.vertices().size(); i++) {
			Vec3f pos(mesh.vertices()[i]);
			Color c(mesh.colors()[i]);
			for (int _ = 0; _ < historySize; _++) {
				trails.vertex(pos);
				trails.color(c);
				trails.texCoord(0.04, 0);
			}
		}

		nav().pos(0, 0, 10);
	}

	bool freeze = false;
	void onAnimate(double dt) override {
		if (freeze) return;

		// ignore the real dt and set the time step;
		dt = timeStep;

		// Generic force modifier lambda

		// apply interactions
		for (vector<Vec3f>::iterator it = mesh.vertices().begin(); it != mesh.vertices().end(); ++it) {
			int itInd = it - mesh.vertices().begin();
			BodyType itType = types[itInd];
			for (vector<Vec3f>::iterator that = mesh.vertices().begin() + itInd + 1; that != mesh.vertices().end(); ++that) {
				int thatInd = that - mesh.vertices().begin();
				// apply all interactivities
				double distance = dist(*it, *that);
				BodyType thatType = types[thatInd];

				// apply life forces
				if (itType == BodyType::LIFE && thatType == BodyType::LIFE) {
					double hookesForce = stiffness * (distance - relaxedDistance);
					// apply separation conditionally
					if (distance < sepDistThresh) {
						acceleration[itInd] -= (*that - *it).normalize(hookesForce) / (mass[itInd] * 1000000000);
						acceleration[thatInd] -= (*it - *that).normalize(hookesForce) / (mass[itInd] * 1000000000);
					}

					// apply cohesion conditionally
					if (distance < cohDistThresh) {
						acceleration[itInd] += (*that - *it).normalize(hookesForce) / (mass[itInd] * 1000000000);
						acceleration[thatInd] += (*it - *that).normalize(hookesForce) / (mass[itInd] * 1000000000);
					}

					// apply alignment conditionally
					if (distance < alignDistThresh) {
						acceleration[itInd] += (*that).normalize(hookesForce) / (mass[itInd] * 1000000000);
						acceleration[thatInd] += (*it).normalize(hookesForce) / (mass[itInd] * 1000000000);
					}
					
				}

				// apply hooke's force
				if (distance > relaxedDistance && distance < attachmentThreshold) { // if things are close enough to attach to one another
					double hookesForce = stiffness * (distance - relaxedDistance);// calc hook's force
					acceleration[itInd] += (*that - *it).normalize(getMagnifier(itType, thatType, ForceType::HOOKES) * hookesForce) / mass[itInd];
					acceleration[thatInd] += (*it - *that).normalize(getMagnifier(thatType, itType, ForceType::HOOKES) * hookesForce) / mass[thatInd];
				}

				// apply gravity force
				double gravForce = gravityMultiplier * GRAV_CONST * ((mass[itInd] * mass[thatInd]) / pow(distance, 1.5)); // calc gravity force
				acceleration[itInd] += (*that - *it).normalize(getMagnifier(itType, thatType, ForceType::GRAVITY) * gravForce) / (mass[itInd] * 1000000000);
				acceleration[thatInd] += (*it - *that).normalize(getMagnifier(thatType, itType, ForceType::GRAVITY) * gravForce) / (mass[thatInd] * 1000000000);
			}

			// apply all global forces
			if (drag) {
				acceleration[itInd] -= velocity[itInd] * 0.1;
			}
		}

		// Update trails before physics integration
		vector<Vec3f>& trailPositions(trails.vertices());
		for (int i = 0; i < mesh.vertices().size(); i++) {
			trailPositions[i + nextTrailHead] = mesh.vertices()[i];
		}
		nextTrailHead = nextTrailHead + mesh.vertices().size();
		if (nextTrailHead >= trails.vertices().size()) {
			nextTrailHead = 0;
		}

		// Integration
		//
		vector<Vec3f>& position(mesh.vertices());
		for (int i = 0; i < velocity.size(); i++) {
			// "backward" Euler integration
			velocity[i] += acceleration[i] * dt;
			position[i] += velocity[i] * dt;
		}

		// clear all accelerations (IMPORTANT!!)
		for (auto& a : acceleration) a.zero();
	}

	bool onKeyDown(const Keyboard& k) override {
		if (k.key() == ' ') {
			freeze = !freeze;
		}

		if (k.key() == '1') {
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
		g.draw(trails);
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

float getMagnifier(BodyType typeA, BodyType typeB, ForceType forceType) {
	float magnifier = 1;
	// there are special magnifier rules for life
	if (typeA == BodyType::LIFE) {
		if (typeB == BodyType::LIFE) {
			if (forceType == ForceType::GRAVITY) {
				magnifier = 0;
			}
			if (forceType == ForceType::HOOKES) {
				//magnifier = -50;
			}
		}
		if (typeB == BodyType::PLANET) {
			if (forceType == ForceType::GRAVITY) {
				magnifier = 1;
			}
			if (forceType == ForceType::HOOKES) {
				//magnifier = 100;
			}
		}
		if (typeB == BodyType::STAR) {
			if (forceType == ForceType::GRAVITY) {
				//magnifier = 1;
				magnifier = 1;
			}
			if (forceType == ForceType::HOOKES) {
				//magnifier = 1;
			}
		}
	}
	else { // if you're celestial
		if (typeB == BodyType::LIFE) { // and you're interacting with life
			if (forceType == ForceType::GRAVITY) { // make gravity 0
				magnifier = 0;
			}
			if (forceType == ForceType::HOOKES) { // make hookes 0
				magnifier = 0;
			}
		}
		else {
			if (forceType == ForceType::GRAVITY) {
				magnifier = 1;
			}
			if (forceType == ForceType::HOOKES) {
				magnifier = 0;
			}
		}
	}

	return magnifier;
}