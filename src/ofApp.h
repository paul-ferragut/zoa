#pragma once

#include "ofMain.h"

#include "ofxMPMFluid.h"
#include "ofxSvg.h"
#include "ofxGui.h"  /* for the control panel */
//#include "ofxVectorGraphics.h"    /* for PostScript output */

//#include "ofxKinect.h"
#include "ofxBloom.h"
#include "ofxGaussianBlur.h"
#include "ofxBlur.h"
#include "ofxBokeh.h"
#include "ofxGlow.h"
#include "ofxInverse.h"
#include "ofxKinectCommonBridge.h"

#include "ofxGpuLut.h"

#define EMPTY1 0
#define LOGO1 1
#define EMPTY2 2
#define LOGO2 3


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void exit();

		ofxMPMFluid fluid;

		bool  bFullscreen;
		bool  bShowControlPanel;
		ofxFloatSlider scaleFactor;
		ofxPanel gui;
		void setupControlPanel();
		ofxFloatSlider N_PARTICLES;
		ofxFloatSlider DENSITY;
		ofxFloatSlider STIFFNESS;
		ofxFloatSlider BULK_VISCOSITY;
		ofxFloatSlider ELASTICITY;
		ofxFloatSlider VISCOSITY;
		ofxFloatSlider YIELD_RATE;
		ofxFloatSlider GRAVITY;
		ofxFloatSlider SMOOTHING;
		ofxFloatSlider LINEWIDTH;
		ofxToggle DO_OBSTACLES[OBSTACLESGROUPS];
		ofxToggle DENSITY_GRADIENT;
		ofxToggle VERTICAL_SYNC;

		ofxFloatSlider LETTER_REPULSION;
		ofxFloatSlider LETTER_ATTRACTION;


		ofxIntSlider effects;

		
		ofxToggle debugKinect;
		vector < ofPoint>kinectPts;
		vector < ofPoint>kinectPtsPrev;

		ofxBloom    bloom;
		ofxToggle bloomB;
		ofxFloatSlider bloomIntensity;

		ofxGaussianBlur gaussianBlur;
		ofxToggle blurB;
		ofxFloatSlider blurIntensity;
		//ofxBlur     blur;
		ofxBokeh    bokeh;
		ofxToggle bokehB;
		ofxFloatSlider bokehIntensity;
		//ofxGlow     glow;

		ofxBloom    inverse;

		ofFbo fbo;
		
		ofxKinectCommonBridge kinect;
		ofVec3f head, lHand, rHand;
		bool hasSkeleton;

		ofxFloatSlider scaleKinectPts;
		float jointDistance;
		ofxFloatSlider p1, p2, p3, p4, p5, p6;

		float scalePts;
		//ofVec2f translatePts(0, (((ofGetHeight() / 2) - (480 / 2))/10)*scaleFactor);
		//cout << scalePts << endl;
		ofVec2f translatePts;

		/*
		ofColor lightColor;
		float colorHue;
		ofColor materialColor;
		ofLight pointLight;
		ofMaterial material;	ofVec3f center;
		*/

		ofShader shader;


		ofImage brushImage;

		ofFbo fboBg;
		ofFbo maskFbo;

		ofxFloatSlider fluidity1;
		ofxFloatSlider fluidity2;
		ofxFloatSlider fluidity3;
		ofxFloatSlider speed;
		ofxFloatSlider scale;
		ofxFloatSlider resolutionMesh;

		ofxToggle showNoiseDebug;

		ofShader shaderNoise;
		ofFbo fboNoise;

		int counterTimer;
		ofxIntSlider delay;
		int timerLimit;
		int sequence;
		ofxToggle autoSequence;
		/*
		void loadLUT(string path);
		void applyLUT(ofPixelsRef pix);

		bool doLUT;
		ofVideoGrabber 		vidGrabber;
	
		ofPoint lutPos;
		ofPoint thumbPos;

		bool LUTloaded;
		ofVec3f lut[32][32][32];

		ofImage lutImg;
	*/
		bool showGui;

		ofxGpuLut luts[9];
		ofImage lutImg;
		ofVideoGrabber video;
		bool isThumbnailView;
		int lutIndex;
		string lutNames[9];
		string description;
		ofxIntSlider lutSelection;

		ofxFloatSlider kinectForce;

		int w;
		int h;

		ofxSVG svgA[2];
};
