#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
    //execute ssh shell script




    //logging levels are verbose print
	ofSetLogLevel(OF_LOG_SILENT);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

	// print the intrinsic IR sensor values to console
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

    //colorImg represents the color of every pixel in rgb
    //allocate determines the width and height of the kinect's view for the colorImg
    //class to calculate for
	colorImg.allocate(kinect.width, kinect.height);
	//same as above but for depth
	grayImage.allocate(kinect.width, kinect.height);
	//determines near and far thresholds
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

    //manipulate these in order to determine how much kinect considers the users
	nearThreshold = 255;
	farThreshold = 200;
	//bool
	bThreshWithOpenCV = true;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
    counter = 0;
    sum = 0;
    boolCounter = 0;
}

//--------------------------------------------------------------
void ofApp::update() {

	ofBackground(190, 100, 100);

    //in order to read new upcoming frames
	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
                //arrays determining depth of individual pixels
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			//sets the standard threshhold which can be manipulated
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {

			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();

			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255; //if in between two thresholds its white
				} else {
					pix[i] = 0; //else black
				}
			}
		}

		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		//grabs current image, determines minimum box size, max box size is size of kinect, number of boxes allowed, leave
		//last false (detects contours holes)
		contourFinder.findContours(grayImage, 20, (kinect.width*kinect.height), 1, false, true);
		for(int i = 0; i < contourFinder.nBlobs; i++) {
        ofxCvBlob maincontour = contourFinder.blobs.at(i);
        blobArea = int (maincontour.area);

        sum += blobArea;
        //Area of the detected image
        //counter counts for 50 input values, then we output the average
        counter++;
        if(counter >= 50)
        {
            //send output to shell script
            cout << sum/50 << endl;
            if((sum/50) > 40000)
            {
                boolCounter = 0;
                cout << false << endl;
            }
            else {
                cout << true << endl;
                boolCounter++;
            }

            if (boolCounter >= 5)
            {
                //execute shell script to sleep
                cout << "RETURNED 5 TRUE" << endl;
                system("./screen 2");
            }

            //reset counter and sum for next 50 set of calculations
            counter = 0;
            sum = 0;
        }

		}


	}

}

//--------------------------------------------------------------
void ofApp::draw() {

	ofSetColor(255, 255, 255);

		// draw from the live kinect
		//1st video feed
		kinect.drawDepth(10, 10, 400, 300);
		//2nd video feed
		kinect.draw(420, 10, 400, 300);

        //3rd video feed
        //x,y,w,h
		grayImage.draw(840, 10, 400, 300);
		contourFinder.draw(840, 10, 400, 300);

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;


	reportStream << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }

	ofDrawBitmapString(reportStream.str(), 20, 340);

}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;

		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;

		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;

		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;

		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;

		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;

		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;

		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;

		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;

		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
