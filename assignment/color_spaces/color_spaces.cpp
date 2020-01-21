#include "al/app/al_App.hpp"
#include "al/graphics/al_Image.hpp"  // al::Image
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)

using namespace al;

#include <fstream>
#include <vector>
#include <functional> // std::divides 
#include <map>  // unordered_map
using namespace std;

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

void rgb2hsv(Vec3f& hsvPixel, Vec3f pixel) {
	double mi = *min_element(begin(pixel),end(pixel));
	double ma = *max_element(begin(pixel), end(pixel));
	float delta = ma - mi;
	// calculate hue
	float h;
	if (ma == pixel.x) {
		hsvPixel.x = 60 * (fmod(((pixel.y - pixel.z) / delta), 6));
	}
	else if (ma == pixel.y) {
		hsvPixel.x = 60 * (((pixel.z - pixel.x) / delta) + 2);
	}
	else if (ma == pixel.z) {
		hsvPixel.x = 60 * (((pixel.x - pixel.y) / delta) + 4);
	}
	// calculate saturation
	float s;
	if (ma == 0) {
		hsvPixel.y = 0;
	}
	else {
		hsvPixel.y = delta / ma;
	}
	// calculate value
	hsvPixel.z = ma;
}

typedef vector<Vec3f> Matrix;

struct AlloApp : App {
  Parameter pointSize{"/pointSize", "", 0.21, "", 0.0, 1.0};
  ControlGUI gui;
  ShaderProgram pointShader;
  Mesh mesh;

  Matrix original;
  Matrix cube;
  Matrix cyl;

  Matrix* A = &original;
  Matrix* B = &original;

  string currentStyle = "original";
  string targetStyle = "original";

  Matrix* D = &original;

  bool animating = false;
  float alpha = 0;

  void onCreate() override {
    gui << pointSize;
    gui.init();
    navControl().useMouse(false);
	cout << "Initialized GUI and Nav.\n";

    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));
	cout << "Slurped shaders.\n";

    mesh.primitive(Mesh::POINTS);

    const char* filename = "../content1.png";
    auto imageData = Image(filename);
    if (imageData.array().size() == 0) {
      std::cout << "failed to load image" << std::endl;
      exit(1);
    }
	cout << "Loaded image.\n";

    Image::RGBAPix pixel;
    const int W = imageData.width();
    const int H = imageData.height();
	float cubeSideLength = H;
	int count = 0;
	Vec3f normPix;
	cout << "Prepared setup stuff.\n";
	for (int c = 0; c < W; c++) {
		for (int r = 0; r < H; r++) {
			imageData.read(pixel, c, r);
			normPix.set(pixel.r / 255.0, pixel.g / 255.0, pixel.b / 255.0); //x = r, y = g, z = b

			// image as plane
			Vec3f oriPosition((c - W / 2) * 0.005, (r - H / 2) * 0.005, 0);
			original.push_back(oriPosition);

			// image as RGB cube
			Vec3f cubePosition((normPix.x * 2 - 1) * cubeSideLength / 2 * 0.005, (normPix.y * 2 - 1) * cubeSideLength / 2 * 0.005, (normPix.z * 2 - 1) * cubeSideLength / 2 * 0.005);
			cube.push_back(cubePosition);

			// image as HSV cylinder
			Vec3f hsv; // x = hue, y = sat, z = val
			rgb2hsv(hsv,normPix);

			Vec3f cylPosition(hsv.y * (W / 2) * cos(hsv.x) * 0.005, hsv.z * (H / 2) * 0.005, hsv.y * (W / 2) * sin(hsv.x) * 0.005);
			cyl.push_back(cylPosition);

			// set the mesh to the plane shape and assign colors based on that.
			mesh.vertex(oriPosition);
			mesh.color(normPix.x, normPix.y, normPix.z);
		}
	}
	cout << "Loaded styles.\n";

	/*
	// collect the various style combos' pixel-by-pixel deltas into a dictionary
	for (pair<string,Matrix&> styleA : styleDict) {
		Matrix& matrixA = styleA.second;
		for (pair<string, Matrix&> styleB : styleDict) {
			string styleCombo = styleA.first + styleB.first;
			Matrix& matrixB = styleB.second;

			// calculate the pixel vector by pixel vector deltas going from styleA to styleB
			Matrix d;
			for (int i = 0; i < styleA.second.size(); i++) {
				d.push_back(matrixA[i] - matrixB[i]);
			}
			deltaDict.insert({ styleCombo, d });
		}
	}
	cout << "Pre-Calculated Style Animation Deltas.\n";
	*/

    nav().pos(0, 0, 10);
  }

  bool freeze = false;
  void onAnimate(double dt) override {

	if (freeze) return;

	// animate from one mesh shape to another
	if (animating) {
		alpha += dt;
		for (int i = 0; i < mesh.vertices().size();  i++) {
			mesh.vertices()[i] = (1 - alpha) * (*A)[i] + alpha * (*B)[i];
		}
		if (alpha >= 1) {
			mesh.vertices() = *B;
			alpha = 0;
			animating = false;
			A = B;
			cout << "animated.\n";
		}
	}
  }

  bool onKeyDown(const Keyboard& k) override {
    if (k.key() == ' ') {
      freeze = !freeze;
    }

    if (k.key() == '1') {
		targetStyle = "original";
		if (targetStyle != currentStyle) {
			B = &original;
			animating = true;
			currentStyle = "original";
		}
    }

	if (k.key() == '2') {
		targetStyle = "cube";
		if (targetStyle != currentStyle) {
			B = &cube;
			animating = true;
			currentStyle = "cube";
		}
	}

	if (k.key() == '3') {
		targetStyle = "cylinder";
		if (targetStyle != currentStyle) {
			B = &cyl;
			animating = true;
			currentStyle = "cylinder";
		}
	}

    return true;
  }

  void onDraw(Graphics& g) override {
    g.clear(0.01);
    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100);
    g.depthTesting(true);
    g.draw(mesh);
    gui.draw(g);
  }
};

int main() { AlloApp().start(); }