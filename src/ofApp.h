#pragma once

#include "ofMain.h"

#include "ofxMPMFluid.h"
#include "ofxSvg.h"
#include "ofxGui.h"  /* for the control panel */
//#include "ofxVectorGraphics.h"    /* for PostScript output */

//#include "ofxKinect.h"
#include "ofxBlur.h"
#include "ofxKinectCommonBridge.h"

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

		
		ofxToggle debugKinect;
		vector < ofPoint>kinectPts;
		vector < ofPoint>kinectPtsPrev;

		ofxBlur             blur;

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

};
