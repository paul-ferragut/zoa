#pragma once

#include "ofMain.h"

#include "ofxMPMFluid.h"
#include "ofxSvg.h"
#include "ofxGui.h"  /* for the control panel */
//#include "ofxVectorGraphics.h"    /* for PostScript output */
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBlur.h"

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
		float scaleFactor;
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

		ofxToggle debugKinect;

	//	ofxToggle bThreshWithOpenCV;
		
		ofxIntSlider nearThreshold;
		ofxIntSlider farThreshold;

		ofxCvGrayscaleImage grayImage; // grayscale depth image
		ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
		ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

		ofxCvContourFinder contourFinder;

		ofxIntSlider blobSize1;
		ofxIntSlider blobSize2;
		vector < ofPoint>kinectPts;
		vector < ofPoint>kinectPtsPrev;



		ofxKinect kinect;

		ofxBlur             blur;

		ofFbo fbo;
		
};
