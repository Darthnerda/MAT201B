// original code by Rodney
// modifications by Josh

#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp" // gui.draw(g)
#include "al/sound/al_SoundFile.hpp"

using namespace al;

#include <fstream>
#include <vector>
using namespace std;

string slurp(string fileName); // forward declaration

struct AlloApp : App {
  // add more GUI here
  Parameter pointSize{"/pointSize", "", 2.0, "", 0.0, 2.0};
  Parameter timeStep{"/timeStep", "", 0.01, "", 0.001, 0.1};
  Parameter dragFactor{"/drag", "", 0.01, "", 0, 0.5};
  ParameterBool drawTrails{"/drawTrails", "", 0};
  Parameter asymmetry{"/asymmetry", "", 0, "", 0, 1};

  ControlGUI gui;

  ShaderProgram pointShader;
  Mesh mesh; // vector<Vec3f> position is inside mesh
  Texture trail;

  SoundFilePlayerTS playerTS;
  std::vector<float> buffer;

  //  simulation state
  vector<Vec3f> velocity;
  vector<Vec3f> acceleration;
  vector<float> mass;
  const float GM =
      1.190588106 * pow(10, -5); // Geocentric gravitational constant
                                 // (gravity per earth mass) in AU^3/s^2
  const float largeBodies[34] = {
      // Geocentric masses of 34 largest bodies
      333000,   317.83,   95.162,   14.536,   17.147,  1,       0.815,
      0.107,    0.0248,   0.0225,   0.0553,   0.018,   0.015,   0.0123,
      0.008035, 0.003599, 0.0022,   0.0028,   0.00059, 0.00066, 0.00039,
      0.0005,   0.00033,  0.00053,  0.00029,  0.00025, 0.0002,  0.000226,
      0.000183, 0.0002,   0.000103, 0.000157, 0.0001,  0.000082};

  const float planetDistance[10] = {0, 5.20,  9.58, 19.2,  30.1,
                                    1, 0.722, 1.52, 0.387, 39.5};

  float value = 0;

  void onInit() override {
	  const char name[] = "../data/Carmichael_Alleyway.wav";
	  if (!playerTS.open(name)) {
		  std::cerr << "File not found: " << name << std::endl;
		  quit();
	  }
	  std::cout << "sampleRate: " << playerTS.soundFile.sampleRate << std::endl;
	  std::cout << "channels: " << playerTS.soundFile.channels << std::endl;
	  std::cout << "frameCount: " << playerTS.soundFile.frameCount << std::endl;
	  playerTS.setLoop();
	  playerTS.setPlay();
  }

  void onCreate() override {
    // add more GUI here
    gui << pointSize << timeStep << dragFactor << drawTrails;
    gui.init();
    navControl().useMouse(false);
    // compile shaders
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    trail.filter(Texture::LINEAR);

    // set initial conditions of the simulation
    //

    mesh.primitive(Mesh::POINTS);
    // does 1000 work on your system? how many can you make before you get a low
    // frame rate? do you need to use <1000?
    // void reset() {

    reset();

    nav().pos(0, 0, 15);
  }

  void reset() {

    mesh.reset();
    velocity.clear();
    acceleration.clear();

    rnd::Random<> rng;
    rng.seed(42);

    auto rc = [&]() { return HSV(rng.uniform(), 1.0f, 1.0f); };
    auto rv = [&](float scale) -> Vec3f {
      return Vec3f(rng.uniformS(), rng.uniformS(), rng.uniformS()) * scale;
    };

    for (int i = 0; i < 1000; i++) {
      if (i == 0) {
        mesh.color(HSV(0.125, 1, 1));
      } else if (i < 10) {
        mesh.color(rc());

      } else if (i > 10) {
        mesh.color(HSV(0, 0, 0.7));
      }

      // float m = rnd::uniform(3.0, 0.5);
      //   float m = 3 + rnd::normal() / 2;
      //   if (m < 0.5) m = 0.5;
      float m = 0;

      if (i < 34) {
        m = largeBodies[i];
      } else if ((34 <= i) && (i < 100)) {
        m = rnd::uniform(0.00001) + 0.00001;
      } else if ((100 <= i) && (i < 500)) {
        m = rnd::uniform(0.0000077) + 0.0000001;
      } else if ((500 <= i) && (i < 1000)) {
        m = rnd::uniform(0.00000001) + 0.000000000001;
      }

      mass.push_back(m);

      // separate state arrays
      if (i == 0) {
        mesh.vertex(0);
        velocity.push_back(rv(0));
        acceleration.push_back(0);
      } else {
        float initialDistance =
            sqrt(i < 10 ? planetDistance[i] : rnd::uniform(30.0) + 1);
        mesh.vertex(initialDistance, 0, 0);
        float initialVelocity = pow(GM * mass[0] / initialDistance, 1.0 / 2);
        velocity.push_back(Vec3f(0, initialVelocity, 0));
        acceleration.push_back(0);
      }

      // using a simplified volume/size relationship
      if (i == 0) {
        mesh.texCoord(20, 0); // s, t
      } else {
        float s = pow(m, 1.0 / 3);
        mesh.texCoord(s > 0.4 ? s : 0.4, 0); // s, t
      }
    }
  }

  bool freeze = false;
  void onAnimate(double dt) override {
    if (freeze)
      return;

    // ignore the real dt and set the time step;
    dt = timeStep;
    // Calculate forces
    for (int i = 0; i < velocity.size(); i++) {
      if (mass[i] == 0)
        continue;
      // drag
      // acceleration[i] -= velocity[i] * dragFactor;
      // Inter-particle gravitation
      for (int j = i + 1; j < velocity.size(); j++) {
        Vec3f r = mesh.vertices()[i] - mesh.vertices()[j];
        if (mass[j] == 0)
          continue;
        if (r.mag() < 0.01) {
          mass[i] += mass[j];
          mass[j] = 0;
          mesh.vertices()[j] = 0;
          float s = pow(mass[i], 1.0 / 3);
          mesh.texCoord2s()[i] = Vec2f(s > 0.4 ? s : 0.4, 0);
        }
        float distcubed = pow(r.mag(), 3.0);
        if (distcubed == 0)
          continue;
        Vec3f F = GM * mass[j] * mass[i] * r / distcubed;
        acceleration[i] -= F * (1 + float(asymmetry));
        acceleration[j] += F / (1 + float(asymmetry));
      }
    }

    // Integration
    //
    vector<Vec3f> &position(mesh.vertices());
    for (int i = 0; i < velocity.size(); i++) {
      // "backward" Euler integration
      velocity[i] += acceleration[i] / mass[i] * dt;
      position[i] += velocity[i] * dt;

      // Explicit (or "forward") Euler integration would look like this:
      // position[i] += velocity[i] * dt;
      // velocity[i] += acceleration[i] / mass[i] * dt;
    }

    // clear all accelerations (IMPORTANT!!)
    for (auto &a : acceleration)
      a.zero();
  }

  bool onKeyDown(const Keyboard &k) override {
    if (k.key() == ' ') {
      freeze = !freeze;
    }

    if (k.key() == 'r') {
      //
      reset();
    }

    return true;
  }

  void onSound(AudioIOData& io) override {
	  int frames = (int)io.framesPerBuffer();
	  int channels = playerTS.soundFile.channels;
	  int bufferLength = frames * channels;
	  if ((int)buffer.size() < bufferLength) {
		  buffer.resize(bufferLength);
	  }
	  playerTS.getFrames(frames, buffer.data(), (int)buffer.size());
	  int second = (channels < 2) ? 0 : 1;
	  while (io()) {
		  int frame = (int)io.frame();
		  int idx = frame * channels;
		  io.out(0) = value = buffer[idx];
		  io.out(1) = buffer[idx + second];
	  }
  }

  void onDraw(Graphics &g) override {
    g.clear(0);
    if (drawTrails) {
      // 1. Match texture dimensions to window
      trail.resize(fbWidth(), fbHeight());
      // 2. Draw feedback texture.
      g.tint(0.98);
      g.quadViewport(trail, -1, -1, 2, 2); // non-transformed
      g.tint(1);                           // set tint back to 1
    }
    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100 + (value / 10));
    g.blending(true);
    g.blendModeTrans();
    g.depthTesting(true);
    g.draw(mesh);

    if (drawTrails) {
      g.depthTesting(false);
      trail.copyFrameBuffer(); // 3. Copy current (read) frame buffer to texture
    }

    imguiBeginFrame();
    ParameterGUI::beginPanel("Control");
    ParameterGUI::drawParameter(&pointSize);
    ParameterGUI::drawParameter(&timeStep);
    ParameterGUI::drawParameter(&dragFactor);
    ParameterGUI::drawParameterBool(&drawTrails);
    ParameterGUI::drawParameter(&asymmetry);

    ImGui::Text("Framerate %.3f", ImGui::GetIO().Framerate);

    ParameterGUI::endPanel();
    imguiEndFrame();
    imguiDraw();
  }
};

int main() { 
	AlloApp().configureAudio(44100, 512, 2, 0);
	AlloApp().start();
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