#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	//ofEnableDepthTest();
	ofSetWindowTitle("ZOA");
	ofBackground(0, 0, 0);

	
	kinect.initSensor( 0 );

	kinect.initColorStream(640, 480);
	kinect.initDepthStream(320, 240, true);
	kinect.initSkeletonStream(true);

	kinect.start();
	
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
			p.setPolyWindingMode(OF_POLY_WINDING_ODD);//MATTERS
			vector<ofPolyline>& lines = const_cast<vector<ofPolyline>&>(p.getOutline());
			for (int j = 0; j<(int)lines.size(); j++) {
				outlines.push_back(lines[j]);
			}
		}
	//	cout << "ADD OBSTACLES" << endl;
		fluid.addObstacles(outlines, k);
	}
	
	for (int k = 0; k < 3; k++) {
		vector<ofPolyline> outlines;
		ofxSVG svg;
		svg.load("zoai" + ofToString(k + 1) + ".svg");


		for (int i = 0; i < svg.getNumPath(); i++) {
			ofPath p = svg.getPathAt(i);
			// svg defaults to non zero winding which doesn't look so good as contours
			p.setPolyWindingMode(OF_POLY_WINDING_ODD);//MATTERS
			vector<ofPolyline>& lines = const_cast<vector<ofPolyline>&>(p.getOutline());
			for (int j = 0; j<(int)lines.size(); j++) {
				outlines.push_back(lines[j]);
			}
		}
		//	cout << "ADD OBSTACLES" << endl;
		fluid.addObstacles(outlines, k+3);
	}
	setupControlPanel();

	fbo.allocate(ofGetWidth(), ofGetHeight(),GL_RGBA,4);
	bloom.allocate(ofGetWidth(), ofGetHeight());
	bokeh.allocate(ofGetWidth(), ofGetHeight());
	gaussianBlur.allocate(ofGetWidth(), ofGetHeight());
	inverse.allocate(ofGetWidth(), ofGetHeight());


	/*
	pointLight.setDiffuseColor(ofColor(255.f, 255.f, 255.f));

	// specular color, the highlight/shininess color //
	pointLight.setSpecularColor(ofColor(255.f, 255.f, 200.f));
	pointLight.setPosition(center.x, center.y, 0);

	// shininess is a value between 0 - 128, 128 being the most shiny //
	material.setShininess(64);
	//colorHue = ofRandom(0, 250);

	lightColor.setBrightness(255.f);
	lightColor.setSaturation(0.f);

	materialColor.setBrightness(255.f);
	materialColor.setSaturation(00.f);
	*/

	shader.load("shader");
	brushImage.loadImage("brush.png");
	maskFbo.allocate(ofGetWidth(), ofGetHeight());
	fboBg.allocate(ofGetWidth(), ofGetHeight());
	fboNoise.allocate(ofGetWidth()/2, ofGetHeight()/2);

	maskFbo.begin();
	ofClear(0, 0, 0, 255);
	maskFbo.end();

	fboBg.begin();
	ofClear(0, 0, 0, 255);
	fboBg.end();

	sequence = 0;
	timerLimit = 0;
	shaderNoise.load("noise");
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

	fluid.letterAttract = LETTER_ATTRACTION;
	fluid.letterRepel = LETTER_REPULSION;

	fluid.scaleFactor = scaleFactor;

	kinect.update();
		 scalePts =  ((((1080.0 / (640.0)))*scaleKinectPts) / 10)*scaleFactor;
		 

		 translatePts.set( (((1080 / 10)*scaleFactor) / 2)-((640 * scalePts)/ 2) ,  (((1920 / 10)*scaleFactor) / 2)- ((480 * scalePts)/ 2) );
		
	if (kinect.isNewSkeleton()) {


		kinectPts.clear();
		for (int i = 0; i < kinect.getSkeletons().size(); i++)
		{

			if (kinect.getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD) != kinect.getSkeletons().at(i).end())
			{

				// just get the first one
				SkeletonBone headBone = kinect.getSkeletons().at(i).find(NUI_SKELETON_POSITION_HEAD)->second;
				SkeletonBone lHandBone = kinect.getSkeletons().at(i).find(NUI_SKELETON_POSITION_HAND_LEFT)->second;
				SkeletonBone rHandBone = kinect.getSkeletons().at(i).find(NUI_SKELETON_POSITION_HAND_RIGHT)->second;
				ofVec3f hb(headBone.getScreenPosition().x, headBone.getScreenPosition().y, 0);
				head = head.getInterpolated(hb, 0.5);
				head.z = ofInterpolateCosine(head.z, headBone.getStartPosition().x, 0.5) + 0.1;
				ofVec3f lhb(lHandBone.getScreenPosition().x, lHandBone.getScreenPosition().y, 0);
				lHand = lHand.getInterpolated(lhb, 0.5);
				lHand.z = ofInterpolateCosine(lHand.z, lHandBone.getStartPosition().x, 0.5);
				ofVec3f rhb(rHandBone.getScreenPosition().x, rHandBone.getScreenPosition().y, 0);
				rHand = rHand.getInterpolated(rhb, 0.5);
				rHand.z = ofInterpolateCosine(rHand.z, rHandBone.getStartPosition().x, 0.5);

				//cout << headBone.getScreenPosition() << endl;
				//cout << rHandBone.getScreenPosition() << endl;
				//cout << lHandBone.getScreenPosition() << endl;

				kinectPts.push_back((rHandBone.getScreenPosition()*scalePts) + translatePts);
				kinectPts.push_back((lHandBone.getScreenPosition()*scalePts)+ translatePts);

				jointDistance = head.distance(rHand);
				jointDistance += lHand.distance(rHand);
				jointDistance += lHand.distance(head);

				hasSkeleton = true;

				//return;
			}
		}
	}
	
	if (kinect.getSkeletons().size() <= 0) {
	kinectPts.clear();
	kinectPtsPrev.clear();
	}
	

	
	fluid.update(kinectPts, kinectPtsPrev);

	kinectPtsPrev = kinectPts;


	if (bloomB) {



		bloom << fbo.getTexture();
		//bloom.setPasses(bloomIntensity);
		bloom.update();
	}

	if (bokehB) {
		bokeh.setRadius(bokehIntensity);
		
		if (bloomB) {
			bokeh << bloom;
		}
		else {
		bokeh << fbo.getTexture();
		}
		
		bokeh.update();
	}
	if (blurB) {
		gaussianBlur.setRadius(blurIntensity);
		gaussianBlur << bokeh;
		gaussianBlur.update();
	}



	/*

	colorHue += .1f;
	if (colorHue >= 255) colorHue = 0.f;
	lightColor.setHue(colorHue);


	//pointLight.setDiffuseColor(lightColor);


	materialColor.setHue(colorHue);
	// the light highlight of the material //
	material.setSpecularColor(materialColor);
	*/
	fboNoise.begin();
	ofClear(0, 0, 0, 0);
	shaderNoise.begin();
	shaderNoise.setUniform1f("fluidity1", fluidity1);
	shaderNoise.setUniform1f("fluidity2", fluidity2);
	shaderNoise.setUniform1i("fluidity3", fluidity3);

	shaderNoise.setUniform1f("scaleWidth", scale);
	shaderNoise.setUniform1f("scaleHeight", scale);
	shaderNoise.setUniform1f("time", ofGetFrameNum() *speed);
	shaderNoise.setUniform1i("width", fboNoise.getWidth()); //* 10
	shaderNoise.setUniform1i("height", fboNoise.getHeight()); //* 10

	shaderNoise.setUniform1f("time", ofGetFrameNum() *speed);

	ofDrawRectangle(0, 0, fboNoise.getWidth(), fboNoise.getHeight());

	shaderNoise.end();
	fboNoise.end();


	if (autoSequence) {
	//	cout << "ofGetFrameNum() - counterTimer" << ofGetFrameNum() - counterTimer << endl;	cout << "timerLimit" << timerLimit << endl;
		if (ofGetFrameNum() - counterTimer >= timerLimit) {
			sequence++;
			//cout << "SEQUENCE" << sequence << endl;
			if (sequence > 5) {
				sequence = 0;
			}
			counterTimer = ofGetFrameNum();
			DO_OBSTACLES[0] = false;
			DO_OBSTACLES[1] = false;
			DO_OBSTACLES[2] = false;
			switch (sequence){
			case EMPTY1:
				timerLimit = ofRandom(delay * 5, delay * 7);
				break;
			case EMPTY2:
				timerLimit = ofRandom(delay *0.4, delay *0.6);
				break;
			case EMPTY3:
				timerLimit = ofRandom(delay *0.2, delay *0.3);
				break;
			case LOGO1:
				//GRAVITY = 0.0028;
				timerLimit = ofRandom(delay * 2, delay * 6);
				DO_OBSTACLES[0] = true;
				break;
			case LOGO2:
				timerLimit = ofRandom(delay * 1, delay * 1.5);
				DO_OBSTACLES[1] = true;
				break;
			case LOGO3:
				timerLimit = ofRandom(delay * 9, delay *12);
				DO_OBSTACLES[2] = true;
				break;
			
			}
		
		}
	
	
	}
	else {
		counterTimer = ofGetFrameNum();
	}
	if (sequence == EMPTY1) {
		//GRAVITY = GRAVITY+cos(ofGetElapsedTimeMillis()*0.0001)*0.002;//GET RESET ABOVE TO 0.0028
	}
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofEnableAlphaBlending();

	maskFbo.begin();
	ofClear(0, 0, 0, 0);
	//int brushImageSize = 50;
	//int brushImageX = mouseX - brushImageSize * 0.5;
	//int brushImageY = mouseY - brushImageSize * 0.5;
	//brushImage.draw(brushImageX, brushImageY, brushImageSize, brushImageSize);
	fboNoise.draw(0, 0, ofGetWidth(),ofGetHeight());
	maskFbo.end();


	fboNoise.begin();
	ofClear(0, 0, 0, 0);
	shaderNoise.begin();
	shaderNoise.setUniform1f("fluidity1", fluidity1);
	shaderNoise.setUniform1f("fluidity2", fluidity2);
	shaderNoise.setUniform1i("fluidity3", fluidity3);

	shaderNoise.setUniform1f("scaleWidth", scale);
	shaderNoise.setUniform1f("scaleHeight", scale);
	shaderNoise.setUniform1f("time", ofGetFrameNum() *speed);
	shaderNoise.setUniform1i("width", fboNoise.getWidth()); //* 10
	shaderNoise.setUniform1i("height", fboNoise.getHeight()); //* 10

	shaderNoise.setUniform1f("time", 1000+ofGetFrameNum() *speed);

	ofDrawRectangle(0, 0, fboNoise.getWidth(), fboNoise.getHeight());

	shaderNoise.end();
	fboNoise.end();

//	
	//ofClear(255);
	/*
	ofEnableDepthTest();
	pointLight.setPosition(mouseX, mouseY, 200);
	//ofBackgroundGradient(ofColor(50, 50, 50), ofColor(0, 0, 0), OF_GRADIENT_CIRCULAR);
	

	// enable lighting //
	ofEnableLighting();
	// the position of the light must be updated every frame, 
	// call enable() so that it can update itself //
	pointLight.enable();
	material.begin();
	lightColor.setBrightness(20.f);
	ofDrawPlane(0, 0, -100, ofGetWidth()*4, ofGetHeight()*4);
	lightColor.setBrightness(255.f);


	material.end();
	// turn off lighting //
	ofDisableLighting();
	*/
//	
	fbo.begin();
	ofSetColor(0, 0, 0);
	ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
	ofSetColor(255, 5);
	fboNoise.draw(0,0, ofGetWidth(), ofGetHeight());
	ofSetColor(255,255);
//	ofClearAlpha();
	fluid.draw();

	fbo.end();
	//ofSetColor(255, 255, 255);




	//fbo.draw(0,0);

	fboBg.begin();
	// Cleaning everthing with alpha mask on 0 in order to make it transparent by default
	ofClear(0, 0, 0, 0);

	shader.begin();
	// here is where the fbo is passed to the shader
	shader.setUniformTexture("maskTex", maskFbo.getTextureReference(), 1);
	ofSetColor(255, 255, 255);
	//fbo.draw(0, 0);
	//ofSetColor(0, 0, 0);
	//ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
	//	ofClearAlpha();
	if (blurB) {
		gaussianBlur.draw();
	}
	else {
		bokeh.draw();
	}
	

	//ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
	shader.end();
	fboBg.end();


	
	//
	if (bloomB) {
		bloom.draw();
	}
	else {
		fbo.draw(0,0);
	}



		fboBg.draw(0,0);

	/*
	ofDisableDepthTest();
	if (bloomB) {

		
	}
	if (blurB) {
	
		gaussianBlur.draw();
		ofEnableAlphaBlending();
		ofSetColor(255,50);
		inverse.draw();
		
	}

	if (bokehB) {

		bokeh.draw();
	}
//	fluid.draw();
	//blur.draw(0, 0);
	*/

	ofSetColor(255, 255, 255);
	
	
	if (debugKinect) {		
		kinect.drawDepth(0, 0, 640, 480);
		
		for (int i = 0; i < kinect.getSkeletons().size(); i++)
		{
			kinect.drawSkeleton(i);
		}
		for (int i = 0; i < kinectPts.size(); i++)
		{
			ofDrawCircle(kinectPts[i],20);
		}
		ofNoFill();
		ofDrawRectangle(0 + translatePts.x, 0 + translatePts.y, 640 * scalePts, 480 * scalePts);
		ofFill();

	}
	if (showNoiseDebug) {
		fboNoise.draw(0, 0);
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


	gui.add(LETTER_REPULSION.setup("letter repulsion", 0.12, 0.0, 0.3));
	gui.add(LETTER_ATTRACTION.setup("letter attraction", 0.05,0.0, 0.3));

	gui.add(scaleFactor.setup("scale factor", 6.4, 1, 10));

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
	// For an interesting experiment, try making Density proportional to the luminance of a photograph.
	gui.add(scaleKinectPts.setup("scaleKinectPts", 1.0, 0.5, 7.0));

	gui.add(bloomB.setup("Bloom", false));
	gui.add(bloomIntensity.setup("bloomIntensity", 1.0, 0.0, 5.0));
	
	gui.add(blurB.setup("Blur", false));
	gui.add(blurIntensity.setup("blurIntensity", 1.0, 0.0, 20.0));

	gui.add(bokehB.setup("Bokeh", false));
	gui.add(bokehIntensity.setup("bokehIntensity", 1.0, 0.0, 5.0));
	

	gui.add(showNoiseDebug.setup("showNoiseDebug", false));

	gui.add(p1.setup("head", 0.0, -1.0, 1.0));
	gui.add(p2.setup("leftHand", 0.0, -1.0, 1.0));
	gui.add(p3.setup("rightHand", 0.0, -1.0, 1.0));
	gui.add(p4.setup("blobDensity", 0.8, 0.0, 2.0));
	gui.add(p5.setup("frequency", 0.18, 0.0, 2.0));
	gui.add(p6.setup("scalar", 2.5, -1.0, 3.0));

	jointDistance = 1.f;


	gui.add(fluidity1.setup("fluidity 1", 2.3f, 0.001f, 5));
	gui.add(fluidity2.setup("fluidity 2", 0.4f, 0.001f, 1));
	gui.add(fluidity3.setup("fluidity 3", 3, 1, 16));
	gui.add(speed.setup("speed", 0.00001f, 0.0f, 0.0028f));
	gui.add(scale.setup("scale", 2000, 0, 40000.0f));
	gui.add(delay.setup("delay", 400, 10, 400));
	gui.add(autoSequence.setup("autoSequence",false));


	
	gui.loadFromFile("settings.xml");
}


void ofApp::exit() {
	
	gui.saveToFile("settings.xml");
//	kinect.setCameraTiltAngle(0); // zero the tilt on exit
//	kinect.close();

	
}
