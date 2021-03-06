#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    if( !ofFile::doesFileExist("11to16.bin") ){
        ofSystemAlertDialog("Make sure you have 11to16.bin, xTable.bin and zTable.bin in your data folder!");
        ofExit();
    }
    ofBackground(0, 0, 0);
    
    // kinect
    kinectIsReady = false;
    kinect.open();
    kinectWidth = 512;
    kinectHeight = 424;
    kinectDepth = (int)kinect.maxDistance.getMax();
    
    // listener
    reset.addListener(this, &ofApp::resetPressed);
    enableSmoothLighting.addListener(this, &ofApp::enableSmoothLightingChanged);
    enableScanPeople.addListener(this, &ofApp::enableScanPeopleChanged);
    saveReferenceDepthPixels.addListener(this, &ofApp::saveReferenceDepthPixelsPressed);
    ofAddListener(ofxSimpleTimer::TIMER_COMPLETE, this, &ofApp::timerComplete);
    
    // gui
    showPanel = true;
    panel.setup("distance in mm", "settings.xml", 0, 0);
    // - kinect
    panel.add(kinect.minDistance);
    panel.add(kinect.maxDistance);
    panel.add(step.set("step", 5, 3, 30));
    panel.add(stopUpdatingKinectBullet.set("stopUpdatingKinectBullet", false));
    // - debug
    panel.add(enableDrawDebug.set("enableDrawDebug", true));
    panel.add(enableDrawKinectWireFrame.set("enableDrawKinectWireFrame", true));
    panel.add(enableDrawAssimpModelWireFrame.set("enableDrawAssimpModelWireFrame", false));
    panel.add(hideKinectMesh.set("hideKinectMesh", false));
    panel.add(enableDrawGuideLine.set("enableDrawGuideLine", false));
    panel.add(enableMouseInput.set("enableMouseInput", true));
    panel.add(enableDrawDebugSpheres.set("enableDrawDebugSpheres", false));
    panel.add(enableScanPeople.set("enableScanPeople", false));
    panel.add(saveReferenceDepthPixels.setup("saveReferenceDepth"));
    panel.add(probabilityFactor.set("probabilityFactor", kinect.maxDistance, 1*PROBABILITY_FACTOR_MIN_FACTOR, kinect.maxDistance.getMax()*PROBABILITY_FACTOR_MAX_FACTOR));
    panel.add(reset.setup("reset"));
    // - dmx
    panel.add(enableDmx.set("enableDmx", false));
    for (int i = 0; i < DMX_CHANNEL_NUMBER; i++) {
        panel.add(dmxChannels[i].set("DMX Channel "+ofToString(i+1), 127, 0, 255));
    }
    // - light
    panel.add(lightSpecularColor.set("lightSpecularColor", ofFloatColor::red, ofFloatColor::black, ofFloatColor::white));
    panel.add(lightDissuseColor.set("lightDiffuseColor", ofFloatColor::green, ofFloatColor::black, ofFloatColor::white));
    panel.add(lightAmbientColor.set("lightAmbientColor", ofFloatColor::blue, ofFloatColor::black, ofFloatColor::white));
    panel.add(lightAttenuation.set("lightAttenuation", ofVec3f(1.0, 0.0, 0.0), ofVec3f(0.0, 0.0, 0.0), ofVec3f(5.0, 0.01, 0.0001)));
    panel.add(enableSmoothLighting.set("enableSmoothLighting", true));
    panel.add(enableSeparateSpecularLight.set("enableSeparateSpecularLight", false));
    panel.add(lightPosition.set("lightPosition", ofVec3f(kinectWidth/2.0, kinectHeight/2.0, kinect.minDistance/2.0), ofVec3f(0, 0, -kinectDepth), ofVec3f(kinectWidth, kinectHeight, kinectDepth)));
    // - material
    panel.add(materialSpecularColor.set("materialSpecularColor", ofFloatColor::red, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialDiffuseColor.set("materialDiffuseColor", ofFloatColor::green, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialAmbientColor.set("materialAmbientColor", ofFloatColor::blue, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialEmissiveColor.set("materialEmissiveColor", ofFloatColor::black, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialShininess.set("materialShininess", 64, 0, 128));
    // camera
    panel.add(cameraFov.set("cameraFov", 60, 1, 180));
    panel.add(cameraNearDist.set("cameraNearDist", 6.65107, 0, 100));
    panel.add(cameraFarDist.set("cameraFarDist", 6651.07, 0, kinectDepth));
    panel.add(cameraPosition.set("cameraPosition", ofVec3f(kinectWidth/2.0, kinectHeight/2.0, 0), ofVec3f(0, 0, -kinectDepth), ofVec3f(kinectWidth, kinectHeight, kinectDepth)));
    panel.add(cameraLookAt.set("cameraLookAt", ofVec3f(kinectWidth/2.0, kinectHeight/2.0, kinect.minDistance), ofVec3f(0, 0, -kinectDepth), ofVec3f(kinectWidth, kinectHeight, kinectDepth)));
    // world
    panel.add(modelStartPosition.set("modelStartPosition", ofVec3f(cameraPosition), cameraPosition.getMin(), cameraPosition.getMax()));
    panel.add(worldGravity.set("worldGravity", ofVec3f(0, 0, 15.0), ofVec3f(-100, -100, -100), ofVec3f(30, 30, 30)));
    panel.add(modelMass.set("modelMass", 0.000005, 0.000005, 1)); // 1 is 1 kg
    panel.add(enableAddModel.set("enableAddModel", false));
    panel.add(enableAddModelRandom.set("enableAddModelRandom", false));
    // load saved settings data
    panel.loadFromFile("settings.xml");
    // minimize all guis
    panel.minimizeAll();
    
    // dmx
    dmx.connect("tty.usbserial-ENY46L4I"); // use the name
    //dmx.connect(0); // or use a number
    
    // caemera
    camera.setAutoDistance(false);
    if (!enableMouseInput) camera.disableMouseInput();
    camera.setPosition(cameraPosition);
    ofVec3f upVector(0, -1, 0);
    camera.lookAt(ofVec3f(cameraLookAt), upVector);
    
    // bullet
    world.setup();
    world.enableGrabbing();
    world.setCamera(&camera);
    world.setGravity(worldGravity);
    
    // model
    // assimpModelLoaders
    // - sakura
    assimpModelLoaders[0].loadModel("models/sakura/sakura2/sakura2.3ds", true);
    assimpModelLoaders[0].setPosition(ofGetWidth()/2, ofGetHeight()/2, 0);
    assimpModelLoaders[0].setScale(1.0, 1.0, 1.0);
    // - bitcoin
    assimpModelLoaders[1].loadModel("models/bitcoin/bitcoin4/bitcoin_v02.3ds", true);
    assimpModelLoaders[1].setPosition(ofGetWidth()/2, ofGetHeight()/2, 0);
    assimpModelLoaders[1].setScale(1.0, 1.0, 1.0);
    // - dna
    assimpModelLoaders[2].loadModel("models/dna/dna6/dna_low_green.3ds", true);
    assimpModelLoaders[2].setPosition(ofGetWidth()/2, ofGetHeight()/2, 0);
    assimpModelLoaders[2].setScale(2.0, 2.0, 2.0);
    // - dgcoin
    assimpModelLoaders[3].loadModel("models/bitcoin/bitcoin6/DGbitcoin_low_blue_0_180_235.3ds", true);
    assimpModelLoaders[3].setPosition(ofGetWidth()/2, ofGetHeight()/2, 0);
    assimpModelLoaders[3].setScale(2.0, 2.0, 2.0);
    // - maple
    assimpModelLoaders[4].loadModel("models/maple/maple1/maple_orange.3ds", true);
    assimpModelLoaders[4].setPosition(ofGetWidth()/2, ofGetHeight()/2, 0);
    assimpModelLoaders[4].setScale(0.65, 0.65, 0.65);
    
    // modelSetVector
    modelSetVector.resize(4);
    // - sakura
    modelSetVector[0].push_back(0);
    // - bitcoin & dgcoin
    modelSetVector[1].push_back(1);
    modelSetVector[1].push_back(3);
    // - dna
    modelSetVector[2].push_back(2);
    // - maple
    modelSetVector[3].push_back(4);
    
    // referenceAssimpModelBulletShapes
    ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
    for (int i = 0; i < MODEL_NUMBER; i++) {
        for (int j = 0; j < assimpModelLoaders[i].getNumMeshes(); j++) {
            referenceAssimpModelBulletShapes[i] = new ofxBulletCustomShape;
            referenceAssimpModelBulletShapes[i]->addMesh(assimpModelLoaders[i].getMesh(j), assimpModelLoaders[i].getScale(), true);
            ofVec3f startLoc = ofVec3f( ofRandom(-5, 5), ofRandom(0, -10), ofRandom(-5, 5) );
            referenceAssimpModelBulletShapes[i]->create(world.world, startLoc, startRot, 3.);
            referenceAssimpModelBulletShapes[i]->add();
        }
    }
    currentModelSetId = 0;
    
    // light
    light.setSpecularColor(lightSpecularColor);
    light.setDiffuseColor(lightDissuseColor);
    light.setAmbientColor(lightAmbientColor);
    light.setAttenuation(lightAttenuation->x, lightAttenuation->y, lightAttenuation->z);
    ofSetSmoothLighting(enableSmoothLighting);
    light.setPosition(lightPosition);
    light.setPointLight();
    
    // timer
    timer.setName("play");
    timer.setTime(MODEL_PLAY_SECONDS*1000, 1);
    timer.start();
    
    // debug
    ofSetVerticalSync(false);
    ofSetFrameRate(0);
    // - camera target
    debugSphereCameraTarget.set(10, 3);
    // - debug spheres
    float debugSphereRadius = 5;
    int debugSphereResolution = 3;
    float samplingNumber = 100;
    ofVec3f debugSphereNumber(kinectWidth/samplingNumber, kinectHeight/samplingNumber, kinectDepth/samplingNumber);
    ofVec3f gapBetweenSpheres(kinectWidth/debugSphereNumber.x, kinectHeight/debugSphereNumber.y, kinectDepth/debugSphereNumber.z);
    for (int x = 0; x < debugSphereNumber.x; x++) {
        for (int y = 0; y < debugSphereNumber.y; y++) {
            for (int z = 0; z < debugSphereNumber.z; z++) {
                ofSpherePrimitive instantSphere;
                instantSphere.set(debugSphereRadius, debugSphereResolution);
                instantSphere.setPosition(x*gapBetweenSpheres.x, y*gapBetweenSpheres.y, z*gapBetweenSpheres.z);
                debugSpheres.push_back(instantSphere);
            }
        }
    }
}

void ofApp::setupWhenKinectIsReady(){
    kinectWidth = rawDepthPixels.getWidth();
    kinectHeight = rawDepthPixels.getHeight();
    
    kinectDepth = (int)kinect.maxDistance.getMax();
    savedReferenceDepthPixels = rawDepthPixels;
    savedReferenceDepthTexture.loadData(kinect.getDepthPixels()); // be careful, this is not raw depth pixels
    
    // init kinectBulletShape
    if (kinectBulletShape == NULL) {
        kinectBulletShape = shared_ptr< ofxBulletTriMeshShape >( new ofxBulletTriMeshShape() );
    }
    kinectBulletShape->create( world.world, kinectMesh, ofVec3f(0,0,0), 0.f, ofVec3f(0, 0, 0), ofVec3f(kinectWidth, kinectHeight, kinect.maxDistance.getMax()) );
    kinectBulletShape->add();
    kinectBulletShape->enableKinematic();
    kinectBulletShape->setActivationState( DISABLE_DEACTIVATION );
    
    kinectIsReady = true;
    ofLogNotice("Kinect is ready!");
}

//--------------------------------------------------------------
void ofApp::update(){
    // timer update
    timer.update();
    
    // dmx
    if (enableDmx) {
        for (int i = 0; i < DMX_CHANNEL_NUMBER; i++) {
            dmxChannels[i].set(tween.update());
            dmx.setLevel(i+1, dmxChannels[i]);// be careful, dmx channel starts from 1, not 0.
        }
        dmx.update();
        if ((int)ofGetElapsedTimef() % DMX_TIMER_PER_SECONDS == 0){
            // FIXME: hard code and use same tween to each dmx channel
            for (int i = 0; i < DMX_CHANNEL_NUMBER; i++) {
                doEase(dmxChannels[i], 10000, 0);
            }
        }
    }
    
    // kinect
    kinect.update();
    if( kinect.isFrameNew() ){
        if (showPanel) {
            texDepth.loadData(kinect.getDepthPixels());
            texRGB.loadData(kinect.getRgbPixels());
        }
        rawDepthPixels = kinect.getRawDepthPixels();
        // bullet
        if (kinectBulletShape == NULL ) {
            updateKinectMesh();
            setupWhenKinectIsReady();
        }else if(enableScanPeople) {
            // scanning people feature
            if (!diffDepthTexture.isAllocated()) {
                diffDepthTexture.allocate(rawDepthPixels.getWidth(), rawDepthPixels.getHeight(), GL_RGB);// color texture
                diffDepthTexturePixels = new unsigned char[rawDepthPixels.getWidth()*rawDepthPixels.getHeight()*3];
            }
            diffDepthPixels = rawDepthPixels; // copy
            float * diffDepthPixelsPointer = diffDepthPixels.getPixels();
            modelStartPositions.clear();
            for (int x = 0; x < rawDepthPixels.getWidth(); x++) {
                for (int y = 0; y < rawDepthPixels.getHeight(); y++) {
                    diffDepthPixelsPointer[rawDepthPixels.getWidth()*y+x] = savedReferenceDepthPixels[rawDepthPixels.getWidth()*y+x]-rawDepthPixels[rawDepthPixels.getWidth()*y+x];
                    if (/* savedReferenceDepthPixels[rawDepthPixels.getWidth()*y+x] > kinect.minDistance && savedReferenceDepthPixels[rawDepthPixels.getWidth()*y+x] < kinect.maxDistance && */
                        rawDepthPixels[rawDepthPixels.getWidth()*y+x] > kinect.minDistance && rawDepthPixels[rawDepthPixels.getWidth()*y+x] < kinect.maxDistance) {
                        if (diffDepthPixelsPointer[rawDepthPixels.getWidth()*y+x] > 0) {
                            // calculate probability
                            float probability = diffDepthPixelsPointer[rawDepthPixels.getWidth()*y+x] / probabilityFactor;
                            // judge add or not
                            bool judgeAdd = (ofRandomuf() < probability);
                            if (judgeAdd) {
                                modelStartPositions.push_back(ofVec3f(x, y, modelStartPosition->z));
                            }
                            
                            // set diffDepthTexturePixels with red color
                            if (showPanel) {
                                diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+0] = ofMap(diffDepthPixelsPointer[rawDepthPixels.getWidth()*y+x],
                                                                                                    0, kinect.maxDistance-kinect.minDistance, 0, 255, true) ; // r
                                diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+1] = 0; // g
                                diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+2] = 0; // b
                            }
                        }else{
                            // set diffDepthTexturePixels with blue color
                            if (showPanel) {
                                diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+0] = 0; // r
                                diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+1] = 0; // g
                                diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+2] = ofMap(-1*diffDepthPixelsPointer[rawDepthPixels.getWidth()*y+x],
                                                                                                    0, kinect.maxDistance-kinect.minDistance, 0, 255, true); // b
                            }
                        }
                    }else{
                        // black
                        if (showPanel) {
                            diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+0] = 0; // r
                            diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+1] = 0; // g
                            diffDepthTexturePixels[(rawDepthPixels.getWidth()*y+x)*3+2] = 0; // b
                        }
                    }
                }
            }
            // grayscale diff texture
            // diffDepthTexture.loadData(diffDepthPixels);
            
            // draw RGB diff texture
            if (showPanel) {
                diffDepthTexture.loadData(diffDepthTexturePixels, rawDepthPixels.getWidth(), rawDepthPixels.getHeight(), GL_RGB);
            }
        }else{
            updateKinectMesh();
        }
    }
    
    // bullet
    if (!stopUpdatingKinectBullet && !enableScanPeople) {
        if (kinectHeight != 0 && kinectBulletShape != NULL) {
            kinectBulletShape->remove();
            kinectBulletShape->create( world.world, kinectMesh, ofVec3f(0,0,0), 0.f, ofVec3f(0, 0, 0), ofVec3f(kinectWidth, kinectHeight, kinect.maxDistance.getMax()) );
            kinectBulletShape->add();
            kinectBulletShape->enableKinematic();
            kinectBulletShape->setActivationState( DISABLE_DEACTIVATION );
        }
    }
    
    // model
    if (enableAddModel) {
        // add model in modelStartPosition
        ofxBulletAssimpShape *bulletCustomShape;
        bulletCustomShape = new ofxBulletAssimpShape(currentModelSetId, rand()%(int)modelSetVector[currentModelSetId].size());
        ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
        int modelId = modelSetVector[bulletCustomShape->getSetId()][bulletCustomShape->getIndex()];
        bulletCustomShape->init((btCompoundShape*)referenceAssimpModelBulletShapes[modelId]->getCollisionShape(), referenceAssimpModelBulletShapes[modelId]->getCentroid());
        bulletCustomShape->create(world.world, modelStartPosition, startRot, modelMass);
        bulletCustomShape->add();
        ofVec3f frc(camera.getLookAtDir());
        frc.normalize();
        bulletCustomShape->applyCentralForce(frc*0.005);
        assimpModelBulletShapes.push_back(bulletCustomShape);
    }
    if (enableAddModelRandom && timer.getName() != "interval") {
        // add model in random x, random y and modelStartPosition.z
        ofxBulletAssimpShape *bulletCustomShape;
        bulletCustomShape = new ofxBulletAssimpShape(currentModelSetId, rand()%(int)modelSetVector[currentModelSetId].size());
        ofQuaternion startRot = ofQuaternion(ofRandom(0, 1), ofRandom(0, 1), ofRandom(0, 1), ofRandom(-1*PI, PI));
        int modelId = modelSetVector[bulletCustomShape->getSetId()][bulletCustomShape->getIndex()];
        bulletCustomShape->init((btCompoundShape*)referenceAssimpModelBulletShapes[modelId]->getCollisionShape(), referenceAssimpModelBulletShapes[modelId]->getCentroid());
        // random square
        //        ofVec3f instantModelStartPosition(ofRandom(0, w), ofRandom(0, h), modelStartPosition->z);
        
        // random circle
        float instantRadius = ofRandom(0, 500);
        //        float instantRadius = (ofRandom(-500, 500)+ofRandom(-500, 500)+ofRandom(-500, 500)+ofRandom(-500, 500)+ofRandom(-500, 500))/5.0;
        float instantTheta = ofRandom(-PI, PI);
        ofVec3f instantModelStartPosition(kinectWidth/2.0+instantRadius*cos(instantTheta), kinectHeight/2.0+instantRadius*sin(instantTheta), modelStartPosition->z);
        bulletCustomShape->create(world.world, instantModelStartPosition, startRot, modelMass);
        bulletCustomShape->add();
        ofVec3f frc(camera.getLookAtDir());
        frc.normalize();
        bulletCustomShape->applyCentralForce(frc*ofRandom(-0.01, 0.01));
        ofVec3f instantTorque(ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005));
        bulletCustomShape->applyTorque(instantTorque);
        assimpModelBulletShapes.push_back(bulletCustomShape);
    }
    if (enableScanPeople) {
        // add models in modelStartPositioins
        // ofLogNotice("modelStartPositions"+ofToString(modelStartPositions.size()));
        for (int i = 0; i < modelStartPositions.size(); i++) {
            // add model in random x, random y and modelStartPosition.z
            ofxBulletAssimpShape *bulletCustomShape;
            bulletCustomShape = new ofxBulletAssimpShape(currentModelSetId, rand()%(int)modelSetVector[currentModelSetId].size());
            ofQuaternion startRot = ofQuaternion(ofRandom(0, 1), ofRandom(0, 1), ofRandom(0, 1), ofRandom(-1*PI, PI));
            int modelId = modelSetVector[currentModelSetId][rand()%(int)modelSetVector[currentModelSetId].size()];
            bulletCustomShape->init((btCompoundShape*)referenceAssimpModelBulletShapes[modelId]->getCollisionShape(), referenceAssimpModelBulletShapes[modelId]->getCentroid());
            bulletCustomShape->create(world.world, modelStartPositions[i], startRot, modelMass);
            bulletCustomShape->add();
            ofVec3f frc(camera.getLookAtDir());
            frc.normalize();
            bulletCustomShape->applyCentralForce(frc*ofRandom(-0.01, 0.01));
            ofVec3f instantTorque;
            switch (currentModelSetId) {
                case 0:
                    instantTorque.set(ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005));
                    break;
                case 1:
                    instantTorque.set(ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005));
                    break;
                case 2:
                    instantTorque.set(ofRandom(-0.0001, 0.0001), ofRandom(-0.0001, 0.0001), ofRandom(-0.0001, 0.0001));
                    break;
                case 3:
                    instantTorque.set(ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005));
                    break;
                case 4:
                    instantTorque.set(ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005));
                    break;
                default:
                    break;
            }
            bulletCustomShape->applyTorque(instantTorque);
            assimpModelBulletShapes.push_back(bulletCustomShape);
        }
    }
    
    // world
    world.setGravity(worldGravity);
    world.update(ofGetLastFrameTime()*2, 20);
    
    ofRectangle kinectRange(0, 0, kinectWidth, kinectHeight);
    for( int i = 0; i < assimpModelBulletShapes.size(); i++ ) {
        // erase model which is in out range(z < 0)
        if(assimpModelBulletShapes[i]->getPosition().z < 0) {
            assimpModelBulletShapes[i]->remove();
            assimpModelBulletShapes.erase(assimpModelBulletShapes.begin() + i);
            // erase only one bullet shape
            // break;
        }else if(kinectIsReady && kinectRange.inside(assimpModelBulletShapes[i]->getPosition().x, assimpModelBulletShapes[i]->getPosition().y)){
            // apply force to bullet shape inside the
            float instantKinectDepth = rawDepthPixels[(int)assimpModelBulletShapes[i]->getPosition().x + (int)assimpModelBulletShapes[i]->getPosition().y*kinectWidth];
            if (instantKinectDepth > kinect.minDistance && instantKinectDepth < kinect.maxDistance && assimpModelBulletShapes[i]->getPosition().z > instantKinectDepth) {
                // add force to the model which is above the kinect mesh
                if (ofGetMousePressed()) {
                    ofVec3f wind(0, 0, 0.001); // FIXME: hard code
                    assimpModelBulletShapes[i]->applyCentralForce(wind);
                }
            }
        }
    }
    
    // camera
    if (enableMouseInput) {
        camera.enableMouseInput();
    }else{
        camera.disableMouseInput();
        camera.setupPerspective(false, cameraFov, cameraNearDist, cameraFarDist, ofVec2f(0.0f, 0.0f));
        camera.setPosition(cameraPosition);
        camera.lookAt(ofVec3f(cameraLookAt), ofVec3f(0, -1, 0));
    }
    
    // camera target
    debugSphereCameraTarget.setPosition(cameraLookAt);
    
    // light
    light.setPosition(lightPosition);
    light.setSpecularColor(lightSpecularColor);
    light.setDiffuseColor(lightDissuseColor);
    light.setAmbientColor(lightAmbientColor);
    light.setAttenuation(lightAttenuation->x, lightAttenuation->y, lightAttenuation->z);
    
    // material
    material.setSpecularColor(materialSpecularColor);
    material.setDiffuseColor(materialDiffuseColor);
    material.setAmbientColor(materialAmbientColor);
    material.setEmissiveColor(materialEmissiveColor);
    material.setShininess(materialShininess);
}

void ofApp::updateKinectMesh(){
    kinectMesh.clear();
    
    // add vertex to mesh and save indexes
    vector< vector<int> > indexes;
    int id = 0;
    for (int y = 0; y < kinectHeight; y += step) {
        vector<int> tempVector;
        for (int x = 0; x < kinectWidth; x += step) {
            float distance = rawDepthPixels[x+y*kinectWidth];
            kinectMesh.addVertex(ofVec3f(x, y, distance));
            if (distance > kinect.minDistance && distance < kinect.maxDistance) {
                tempVector.push_back(id);
            }else{
                tempVector.push_back(-1); // set -1 for out of range
            }
            id++;
        }
        indexes.push_back(tempVector);
    }
    
    // set triangle
    for (int y = 0; y < kinectHeight-step; y += step) {
        for (int x = 0; x < kinectWidth-step; x += step) {
            if (indexes[y/step][x/step] != -1 &&
                indexes[y/step][x/step+1] !=  -1 &&
                indexes[y/step+1][x/step+1] != -1 &&
                indexes[y/step+1][x/step] != -1) {
                kinectMesh.addTriangle(indexes[y/step][x/step], indexes[y/step][x/step+1], indexes[y/step+1][x/step+1]);
                kinectMesh.addTriangle(indexes[y/step][x/step], indexes[y/step+1][x/step+1], indexes[y/step+1][x/step]);
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    // draw mesh
    ofEnableDepthTest();{
        // ofPushMatrix();{
        camera.begin();{
            // ofTranslate(-rawDepthPixels.getWidth()/2.0, -rawDepthPixels.getHeight()/2.0, 0);
            ofSetLineWidth(1.f);
            if(enableDrawDebug) world.drawDebug();
            // light
            light.draw();
            ofEnableLighting();{
                light.enable();{
                    if (enableSeparateSpecularLight) {
                        ofEnableSeparateSpecularLight();
                    }
                    glDisable(GL_COLOR_MATERIAL);
                    material.begin();{
                        // kinect mesh
                        if (!hideKinectMesh) {
                            kinectMesh.setMode(OF_PRIMITIVE_TRIANGLES);
                            glLineWidth(int(1));
                            enableDrawKinectWireFrame ? kinectMesh.drawWireframe() : kinectMesh.drawFaces();
                        }
                        
                        // spheres
                        for( int i = 0; i < spheres.size(); i++ ) {
                            spheres[i]->draw();
                        }
                        
                        // assimp models
                        for (int i = 0; i < assimpModelBulletShapes.size(); i++) {
                            int assimpModelId = modelSetVector[assimpModelBulletShapes[i]->getSetId()][assimpModelBulletShapes[i]->getIndex()];
                            ofPoint scale = assimpModelLoaders[assimpModelId].getScale();
                            ofxAssimpMeshHelper & meshHelper = assimpModelLoaders[assimpModelId].getMeshHelper(0);
                            // ofMaterial & assimpModelMaterial = meshHelper.material;
                            meshHelper.getTextureRef().bind();{
                                // assimpModelMaterial.begin();{
                                assimpModelBulletShapes[i]->transformGL();{
                                    ofScale(scale.x, scale.y, scale.z);
                                    enableDrawAssimpModelWireFrame ? assimpModelLoaders[assimpModelId].getCurrentAnimatedMesh(0).drawWireframe() : assimpModelLoaders[assimpModelId].getCurrentAnimatedMesh(0).drawFaces();
                                } assimpModelBulletShapes[i]->restoreTransformGL();
                                // }assimpModelMaterial.end();
                            }meshHelper.getTextureRef().unbind();
                        }
                        
                        // debug
                        // - camera target
                        debugSphereCameraTarget.draw();
                        
                        // - debugSpheres
                        if (enableDrawDebugSpheres) {
                            for (int i = 0; i < (int)debugSpheres.size(); i++) {
                                debugSpheres[i].draw();
                            }
                        }
                    }material.end();
                    if (enableSeparateSpecularLight) {
                        ofDisableSeparateSpecularLight();
                    }
                }light.disable();
            }ofDisableLighting();
            if(enableDrawDebug) ofDrawAxis(10000);
            // }ofPopMatrix();
        }camera.end();
    }ofDisableDepthTest();
    
    // gui
    if (showPanel) {
        panel.draw();
    }
    
    // debug
    ofSetColor(255);
    // - fps
    ofSetWindowTitle(ofToString(ofGetFrameRate(), 0)+" assimpModelBulletShapes:"+ofToString((int)assimpModelBulletShapes.size(), 0));
    
    // - center guide line
    if (enableDrawGuideLine) {
        ofSetColor(ofColor::lightBlue);
        ofLine(ofGetWidth()/2.0, 0, ofGetWidth()/2.0, ofGetHeight());
        ofLine(0, ofGetHeight()/2.0, ofGetWidth(), ofGetHeight()/2.0);
        ofSetColor(255);
    }
    
    // - depth data
    ofVec2f debugImageSize(1920/10.0, 1080/10.0);
    if (showPanel){
        // draw below the gui panel
        // ofRect(panel.getPosition().x, panel.getHeight()+4, debugImageSize.x+2, debugImageSize.y+2);
        // texDepth.draw(panel.getPosition().x+1, panel.getHeight()+5, debugImageSize.x, debugImageSize.y);
        
        // draw upper right corner
        ofRect(ofGetWidth()-debugImageSize.x-3, 1, debugImageSize.x+2, debugImageSize.y+2);
        texDepth.draw(ofGetWidth()-debugImageSize.x-2, 2, debugImageSize.x, debugImageSize.y);
    }
    
    // - RGB data
    if (showPanel) {
        // draw below the gui panel
        // ofRect(panel.getPosition().x, panel.getHeight()+5+debugImageSize.y+2, debugImageSize.x+2, debugImageSize.y+2);
        //texRGB.draw(panel.getPosition().x+1, panel.getHeight()+5+debugImageSize.y+3, debugImageSize.x, debugImageSize.y);
        
        // draw upper right corner
        ofRect(ofGetWidth()-debugImageSize.x-3, debugImageSize.y+4, debugImageSize.x+2, debugImageSize.y+2);
        texRGB.draw(ofGetWidth()-debugImageSize.x-2, debugImageSize.y+5, debugImageSize.x, debugImageSize.y);
    }
    
    // - saved reference depth pixels
    if (showPanel) {
        // draw upper right corner
        ofRect(ofGetWidth()-debugImageSize.x-3, debugImageSize.y*2+7, debugImageSize.x+2, debugImageSize.y+2);
        savedReferenceDepthTexture.draw(ofGetWidth()-debugImageSize.x-2, debugImageSize.y*2+8, debugImageSize.x, debugImageSize.y);
    }
    
    // - diff depth
    if (showPanel) {
        // draw upper right corner
        ofRect(ofGetWidth()-debugImageSize.x-3, debugImageSize.y*3+10, debugImageSize.x+2, debugImageSize.y+2);
        diffDepthTexture.draw(ofGetWidth()-debugImageSize.x-2, debugImageSize.y*3+11, debugImageSize.x, debugImageSize.y);
    }
    
    // - info about ofxKinectV2
    //    ofDrawBitmapString("ofxKinectV2: Work in progress addon.\nBased on the excellent work by the OpenKinect libfreenect2 team\n\n-Only supports one Kinect v2 at a time. \n-Requires USB 3.0 port ( superspeed )\n-Requires patched libusb. If you have the libusb from ofxKinect ( v1 ) linked to your project it will prevent superspeed on Kinect V2", 10, 14);
    
    // info about shortcut keys
    if (showPanel) {
        ofDrawBitmapStringHighlight("press f: toggle full screen         ", ofGetWidth()-300, ofGetHeight()-130, ofColor::white, ofColor::black);
        ofDrawBitmapStringHighlight("press h: hide/show GUI              ", ofGetWidth()-300, ofGetHeight()-110, ofColor::white, ofColor::black);
        ofDrawBitmapStringHighlight("press 0: load sakura model          ", ofGetWidth()-300, ofGetHeight()-90, ofColor::white, ofColor::black);
        ofDrawBitmapStringHighlight("press 1: load bitcoin & dgcoin model", ofGetWidth()-300, ofGetHeight()-70, ofColor::white, ofColor::black);
        ofDrawBitmapStringHighlight("press 2: load DNA model             ", ofGetWidth()-300, ofGetHeight()-50, ofColor::white, ofColor::black);
        ofDrawBitmapStringHighlight("press 3: load maple model           ", ofGetWidth()-300, ofGetHeight()-30, ofColor::white, ofColor::black);
        ofDrawBitmapStringHighlight("press w: apply force                ", ofGetWidth()-300, ofGetHeight()-10, ofColor::white, ofColor::black);
    }
}

bool ofApp::valueIsInKinectRange(float value){
    return (value > kinect.minDistance && value < kinect.maxDistance);
}

void ofApp::resetPressed(){
    panel.loadFromFile("settings.xml");
    camera.setPosition(cameraPosition);
    camera.lookAt(ofVec3f(cameraLookAt), ofVec3f(0, -1, 0));
}

void ofApp::enableSmoothLightingChanged(bool &enableSmoothLightingStatus){
    ofSetSmoothLighting(enableSmoothLighting);
}

void ofApp::enableScanPeopleChanged(bool &enableScanPeople){
    if (enableScanPeople) {
        kinectMesh.clear();
        if (kinectBulletShape != NULL) {
            kinectBulletShape->remove();
        }
    }else{
        if(kinect.getDepthPixels().getWidth() != 0){
            updateKinectMesh();
            setupWhenKinectIsReady();
        }
    }
}

void ofApp::saveReferenceDepthPixelsPressed(){
    savedReferenceDepthPixels = kinect.getRawDepthPixels();
    savedReferenceDepthTexture.loadData(kinect.getDepthPixels()); // be careful, this is NOT raw depth pixels
}

void ofApp::timerComplete(string &name){
    ofLogNotice("timerComplete");
    int currentCount = timer.getLoopCurrentCount();
    int totalCount = timer.getLoopTotalCount();
    
    if(name=="play"){
        if(currentCount==totalCount){
            cout << currentCount << "/" << totalCount << endl;
            cout << "*** Complete ***" << endl;
            timer.setName("interval");
            timer.setTime(MODEL_INTERVAL_SECONDS*1000, 1);
            timer.start();
        }else{
            cout << currentCount << "/" << totalCount << endl;
        }
    }else if("interval"){
        if(currentCount==totalCount){
            cout << currentCount << "/" << totalCount << endl;
            cout << "*** Complete ***" << endl;
            
            ++currentModelSetId %= (int)modelSetVector.size();
            switch (currentModelSetId) {
                case 0:
                    // sakura
                    enableDrawAssimpModelWireFrame = false;
                    changeAssimpModel(0);
                    break;
                case 1:
                    // bitcoin & dgcoin
                    enableDrawAssimpModelWireFrame = false;
                    changeAssimpModel(1);
                    break;
                case 2:
                    // dna
                    enableDrawAssimpModelWireFrame = false;
                    changeAssimpModel(2);
                    break;
                case 3:
                    // maple
                    enableDrawAssimpModelWireFrame = false;
                    changeAssimpModel(3);
                    break;
                default:
                    break;
            }
            
            timer.setName("play");
            timer.setTime(MODEL_PLAY_SECONDS*1000, 1);
            timer.start();
        }else{
            cout << currentCount << "/" << totalCount << endl;
        }
    }else{
        // ignore
    }
}

void ofApp::doEase(ofParameter<int> dmxChannel, unsigned duration, unsigned delay){
    if (dmxChannel == 0) {
        // ease in
        tween.setParameters(1, easingSine, ofxTween::easeIn, 0, 255, duration, delay);
    }
    if (dmxChannel == 255) {
        tween.setParameters(1, easingSine, ofxTween::easeIn, 255, 0, duration, delay);
    }
}

void ofApp::changeAssimpModel(int modelSetId){
    // remove and clear all bullet shapees
    for (int i = 0; i < (int)assimpModelBulletShapes.size(); i++) {
        assimpModelBulletShapes[i]->remove();
    }
    assimpModelBulletShapes.clear();
    // clear assimp model
    // select model by modelId
    switch (modelSetId) {
        case 0:
            // sakura
            currentModelSetId = 0;
            break;
        case 1:
            // bitcoin & dgcoin
            currentModelSetId = 1;
            break;
        case 2:
            // dna
            currentModelSetId = 2;
            break;
        case 3:
            // maple
            currentModelSetId = 3;
            break;
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case ' ':{
            // add one sphere
            shared_ptr< ofxBulletSphere > ss( new ofxBulletSphere() );
            ss->create( world.world, camera.getPosition()+100.0f*camera.getLookAtDir(), 0.05, 30.0);
            ss->add();
            ofVec3f frc(camera.getLookAtDir());
            frc.normalize();
            ss->applyCentralForce(frc*1000);
            spheres.push_back( ss );
        }
            break;
        case '0':
            // sakura
            enableDrawAssimpModelWireFrame = false;
            changeAssimpModel(0);
            break;
        case '1':
            // bitcoin & dgcoin
            enableDrawAssimpModelWireFrame = false;
            changeAssimpModel(1);
            break;
        case '2':
            // dna
            enableDrawAssimpModelWireFrame = false;
            changeAssimpModel(2);
            break;
        case '3':
            // maple
            enableDrawAssimpModelWireFrame = false;
            changeAssimpModel(3);
            break;
        case 'f':
            ofToggleFullscreen();
            break;
        case 'h':
            // show/hide gui panel
            showPanel = !showPanel;
            
            // show/hide mouse cursor
            showPanel ? ofShowCursor() : ofHideCursor();
            break;
        case 'm':{
            // add model
            ofxBulletAssimpShape *bulletCustomShape;
            bulletCustomShape = new ofxBulletAssimpShape;
            ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
            bulletCustomShape->init((btCompoundShape*)assimpModelBulletShapes[0]->getCollisionShape(), assimpModelBulletShapes[0]->getCentroid());
            bulletCustomShape->create(world.world, modelStartPosition, startRot, modelMass);
            bulletCustomShape->add();
            ofVec3f frc(camera.getLookAtDir());
            frc.normalize();
            bulletCustomShape->applyCentralForce(frc*0.005);
            assimpModelBulletShapes.push_back(bulletCustomShape);
        }
            break;
        case 'w':{
            ofVec3f wind(0, 0, 0.01);
            for (int i = 0; i < assimpModelBulletShapes.size(); i++) {
                assimpModelBulletShapes[i]->applyCentralForce(wind);
            }
        }
            break;
        default:
            break;
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
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
