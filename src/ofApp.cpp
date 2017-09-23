#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetWindowTitle("ZOA");
	ofBackground(0, 0, 0);

	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();
	//colorImg.allocate(kinect.width, kinect.height);
	cout << kinect.width << " height" << kinect.height << endl;
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	// zero the tilt on startup
	float angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	bFullscreen = false;
	bShowControlPanel = true;
	ofSetFullscreen(bFullscreen);

	fluid.setup(100000);
	scaleFactor = 6.4;	// FYI: this is computed from (screensize/gridSizeX)



	for (int k = 0; k < 3; k++) {
		vector<ofPolyline> outlines;
		ofxSVG svg;
		svg.load("zoa" + ofToString(k + 1) + ".svg");
		
		
		for (int i = 0; i < svg.getNumPath(); i++) {
			ofPath p = svg.getPathAt(i);
			// svg defaults to non zero winding which doesn't look so good as contours
			p.setPolyWindingMode(OF_POLY_WINDING_ODD);
			vector<ofPolyline>& lines = const_cast<vector<ofPolyline>&>(p.getOutline());
			for (int j = 0; j<(int)lines.size(); j++) {
				outlines.push_back(lines[j]);
			}
		}
		fluid.addObstacles(outlines, k);
	}
	setupControlPanel();

	fbo.allocate(ofGetWidth(), ofGetHeight());
	blur.allocate(ofGetWidth(), ofGetHeight());
	blur.setPasses(10);
}

//--------------------------------------------------------------
void ofApp::update(){
	ofSetWindowTitle("Modern Meadow - ZOA - FPS: "+ofToString(ofGetFrameRate()));
	//-------------------------
	// Update the OpenGL vertical sync, based on the control panel.
	bool bSync = VERTICAL_SYNC;
	ofSetVerticalSync(bSync);

	fluid.numParticles = N_PARTICLES;
	fluid.densitySetting = DENSITY;
	fluid.stiffness = STIFFNESS;
	fluid.bulkViscosity = BULK_VISCOSITY;
	fluid.elasticity = ELASTICITY;
	fluid.viscosity = VISCOSITY;
	fluid.yieldRate = YIELD_RATE;
	fluid.bGradient = DENSITY_GRADIENT;
	fluid.bDoObstacles[0] = DO_OBSTACLES[0];
	fluid.bDoObstacles[1] = DO_OBSTACLES[1];
	fluid.bDoObstacles[2] = DO_OBSTACLES[2];
	fluid.gravity = GRAVITY;
	fluid.smoothing = SMOOTHING;
	fluid.lineWidth = LINEWIDTH;

	fluid.scaleFactor = scaleFactor;

	
	kinect.update();

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
	
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		


		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height) / 2, 20, false);

		float scaleBlob=2;
		ofVec2f offSetBlob(0,500);
		ofxCvBlob biggerBlob;
		kinectPts.clear();
		float area = 0;
		for (int i = 0; i < contourFinder.blobs.size(); i++) {
			if (contourFinder.blobs[i].area > area) {
				//cout << "area" << contourFinder.blobs[i].area << endl;
				biggerBlob = contourFinder.blobs[i];
				area = contourFinder.blobs[i].area;
			}
		}
		if (biggerBlob.area > blobSize1) {
			kinectPts.push_back(biggerBlob.centroid);
		}
		else if (biggerBlob.area > blobSize2) {
			kinectPts.push_back((ofVec2f(biggerBlob.boundingRect.x, biggerBlob.centroid.y)*scaleBlob)+offSetBlob);
			kinectPts.push_back((ofVec2f(biggerBlob.boundingRect.x+ biggerBlob.boundingRect.width, biggerBlob.centroid.y)*scaleBlob) + offSetBlob);
		}
		else {
		
		}

	}
	//kinectPts
	
	
	
	fluid.update(kinectPts, kinectPtsPrev);

	kinectPtsPrev = kinectPts;
	blur.setTexture(fbo.getTexture());
	blur.update();
}

//--------------------------------------------------------------
void ofApp::draw(){

	
	

	fbo.begin();
	//ofClear(255);
	ofBackgroundGradient( ofColor(50, 50, 50),ofColor(0, 0, 0), OF_GRADIENT_CIRCULAR);
	fluid.draw();
	fbo.end();
	blur.draw(0, 0);


	ofSetColor(255, 255, 255);
	if (debugKinect) {
		kinect.draw(420, 10, 400, 300);
		kinect.drawDepth(10, 10, 400, 300);

		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
		for (int i = 0; i < kinectPts.size(); i++) {
			ofDrawCircle(kinectPts[i], 20);
		}
	}
	gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'f') { // fullscreen
		bFullscreen = !bFullscreen;
		ofSetFullscreen(bFullscreen);

	}
	else if (key == 'h') { // help
		bShowControlPanel = !bShowControlPanel;
		if (bShowControlPanel) {
			//	gui.show();
			ofShowCursor();
		}
		else {
			//gui.hide();
			ofShowCursor();
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
void ofApp::setupControlPanel() {

	gui.setup("ZOA GUI", "settings.xml");

	gui.add(N_PARTICLES.setup("Particles", 100000 / 4, 1000, 100000));
	gui.add(DENSITY.setup("Density", 5.0, 0, 30.0));
	gui.add(STIFFNESS.setup("Stiffness", 0.5, 0, 2.0));
	gui.add(BULK_VISCOSITY.setup("Bulk Viscosity", 3.0, 0, 10.0));
	gui.add(ELASTICITY.setup("Elasticity", 1.0, 0, 4.0));
	gui.add(VISCOSITY.setup("Viscosity", 1.0, 0, 4.0));
	gui.add(YIELD_RATE.setup("Yield Rate", 1.0, 0, 2.0));
	gui.add(GRAVITY.setup("Gravity", 0.002, 0, 0.02));
	gui.add(SMOOTHING.setup("Smoothing", 1.0, 0, 3.0));

	gui.add(DO_OBSTACLES[0].setup("Do Obstacles1", false));
	gui.add(DO_OBSTACLES[1].setup("Do Obstacles2", false));
	gui.add(DO_OBSTACLES[2].setup("Do Obstacles3", false));
	// Now, this horizontal gradient in the Density parameter is just for yuks.
	// It demonstrates that spatial variations in the Density parameter can yield interesting results. 
	// For an interesting experiment, try making Density proportional to the luminance of a photograph.
	gui.add(DENSITY_GRADIENT.setup("Horizontal Density Gradient", true));

	// This can eliminate screen tearing. 
	gui.add(VERTICAL_SYNC.setup("Vertical Sync", true));

	gui.add(LINEWIDTH.setup("LINEWIDTH", 1.0, 0, 7.0));

	gui.add(debugKinect.setup("debugKinect", false));
	gui.add(nearThreshold.setup("nearThreshold", 0, 0,255));
	gui.add(farThreshold.setup("farThreshold", 0, 0, 255));


	gui.add(blobSize1.setup("blobSize1", 0, 0, 150000));
	gui.add(blobSize2.setup("blobSize2", 0, 0, 150000));
	gui.loadFromFile("settings.xml");
}


void ofApp::exit() {
	
	gui.saveToFile("settings.xml");
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

	
}
