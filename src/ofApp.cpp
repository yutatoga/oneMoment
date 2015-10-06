#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    if( !ofFile::doesFileExist("11to16.bin") ){
        ofSystemAlertDialog("Make sure you have 11to16.bin, xTable.bin and zTable.bin in your data folder!");
        ofExit();
    }
    ofBackground(0, 0, 0);
    
    // listener
    reset.addListener(this, &ofApp::resetPressed);
    enableSmoothLighting.addListener(this, &ofApp::enableSmoothLightingChanged);
    
    // gui
    showPanel = true;
    showCursor = true;
    // - kinect
    panel.setup("distance in mm", "settings.xml", 0, 0);
    panel.add(kinect.minDistance);
    panel.add(kinect.maxDistance);
    panel.add(step.set("step", 5, 3, 30));
    panel.add(stopUpdatingKinectBullet.set("stopUpdatingKinectBullet", false));
    // - debug
    panel.add(enableDrawDebug.set("enableDrawDebug", true));
    panel.add(enableDrawWireFrame.set("enableDrawWireFrame", true));
    panel.add(hideKinectMesh.set("hideKinectMesh", false));
    panel.add(enableDrawGuideLine.set("enableDrawGuideLine", false));
    panel.add(enableMouseInput.set("enableMouseInput", true));
    panel.add(enableDrawDebugSpheres.set("enableDrawDebugSpheres", false));
    panel.add(reset.setup("reset"));
    // - world
    panel.add(worldGravity.set("worldGravity", ofVec3f(0, 0, 15.0), ofVec3f(-30, -30, -30), ofVec3f(30, 30, 30)));
    panel.add(modelMass.set("modelMass", 0.000005, 0.000005, 1)); // 1 is 1 kg
    panel.add(enableAddModel.set("enableAddModel", false));
    panel.add(enableAddModelRandom.set("enableAddModelRandom", false));
    // - dmx
    panel.add(dmxChannel1.set("DMX Channel 1", 127, 0, 255));
    // - light
    panel.add(lightSpecularColor.set("lightSpecularColor", ofFloatColor::red, ofFloatColor::black, ofFloatColor::white));
    panel.add(lightDissuseColor.set("lightDiffuseColor", ofFloatColor::green, ofFloatColor::black, ofFloatColor::white));
    panel.add(lightAmbientColor.set("lightAmbientColor", ofFloatColor::blue, ofFloatColor::black, ofFloatColor::white));
    panel.add(lightAttenuation.set("lightAttenuation", ofVec3f(1.0, 0.0, 0.0), ofVec3f(0.0, 0.0, 0.0), ofVec3f(5.0, 0.01, 0.0001)));
    panel.add(enableSmoothLighting.set("enableSmoothLighting", true));
    panel.add(enableSeparateSpecularLight.set("enableSeparateSpecularLight", false));
    // - material
    panel.add(materialSpecularColor.set("materialSpecularColor", ofFloatColor::red, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialDiffuseColor.set("materialDiffuseColor", ofFloatColor::green, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialAmbientColor.set("materialAmbientColor", ofFloatColor::blue, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialEmissiveColor.set("materialEmissiveColor", ofFloatColor::black, ofFloatColor::black, ofFloatColor::white));
    panel.add(materialShininess.set("materialShininess", 64, 0, 128));
    panel.loadFromFile("settings.xml");
    // >> to be continued in setupWhenKinectIsReady
    
    // kinect
    kinect.open();
    
    // dmx
    dmx.connect("tty.usbserial-ENY46L4I"); // use the name
    //dmx.connect(0); // or use a number
    
    // caemera
    camera.setAutoDistance(false);
    if (!enableMouseInput) camera.disableMouseInput();
    
    // bullet
    world.setup();
    world.enableGrabbing();
    world.setCamera(&camera);
    world.setGravity(worldGravity);
    
    // model
    ofVec3f scale(1.0, 1.0, 1.0);
    assimpModelLoader.loadModel("sakura2/sakura2.3ds", true);
    assimpModelLoader.setPosition(ofGetWidth()/2, ofGetHeight()/2, 0);
    assimpModelLoader.setScale(scale.x, scale.y, scale.z);
    ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
    assimpModelBulletShapes.resize(3);
    for (int i = 0; i < assimpModelBulletShapes.size(); i++) {
        assimpModelBulletShapes[i] = new ofxBulletCustomShape;
        if (i == 0) {
            for (int i = 0; i < assimpModelLoader.getNumMeshes(); i++) {
                assimpModelBulletShapes[i]->addMesh(assimpModelLoader.getMesh(i), scale, true);
            }
        } else {
            assimpModelBulletShapes[i]->init((btCompoundShape*)assimpModelBulletShapes[0]->getCollisionShape(), assimpModelBulletShapes[0]->getCentroid());
        }
        ofVec3f startLoc = ofVec3f( ofRandom(-5, 5), ofRandom(0, -10), ofRandom(-5, 5) );
        assimpModelBulletShapes[i]->create(world.world, startLoc, startRot, 3.);
        assimpModelBulletShapes[i]->add();
    }
    
    // light
    light.setSpecularColor(lightSpecularColor);
    light.setDiffuseColor(lightDissuseColor);
    light.setAmbientColor(lightAmbientColor);
    light.setAttenuation(lightAttenuation->x, lightAttenuation->y, lightAttenuation->z);
    ofSetSmoothLighting(enableSmoothLighting);
    
    // debug
    ofSetVerticalSync(false);
    ofSetFrameRate(0);
    // - camera target
    debugSphereCameraTarget.set(10, 3);
}

void ofApp::setupWhenKinectIsReady(){
    int w = rawDepthPixels.getWidth();
    int h = rawDepthPixels.getHeight();
    int d = (int)kinect.maxDistance.getMax();
    // gui
    // light
    panel.add(lightPosition.set("lightPosition", ofVec3f(w/2.0, h/2.0, kinect.minDistance/2.0), ofVec3f(0, 0, -d), ofVec3f(w, h, d)));
    // - camera
    panel.add(cameraFov.set("cameraFov", 60, 1, 180));
    panel.add(cameraNearDist.set("cameraNearDist", 6.65107, 0, 100));
    panel.add(cameraFarDist.set("cameraFarDist", 6651.07, 0, d));
    panel.add(cameraPosition.set("cameraPosition", ofVec3f(w/2.0, h/2.0, 0), ofVec3f(0, 0, -d), ofVec3f(w, h, d)));
    panel.add(cameraLookAt.set("cameraLookAt", ofVec3f(w/2.0, h/2.0, kinect.minDistance), ofVec3f(0, 0, -d), ofVec3f(w, h, d)));
    // - world
    panel.add(modelStartPosition.set("modelStartPosition", ofVec3f(cameraPosition), cameraPosition.getMin(), cameraPosition.getMax()));
    // - gui
    panel.loadFromFile("settings.xml");
    panel.minimizeAll();
    
    // light
    light.setPosition(lightPosition);
    light.setPointLight();
    
    // camera
    camera.setPosition(cameraPosition);
    ofVec3f upVector(0, -1, 0);
    camera.lookAt(ofVec3f(cameraLookAt), upVector);
    
    // debug spheres
    float debugSphereRadius = 5;
    int debugSphereResolution = 3;
    float samplingNumber = 100;
    ofVec3f debugSphereNumber(w/samplingNumber, h/samplingNumber, d/samplingNumber);
    ofVec3f gapBetweenSpheres(w/debugSphereNumber.x, h/debugSphereNumber.y, d/debugSphereNumber.z);
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
    
    // - init kinectBulletShape
    kinectBulletShape = shared_ptr< ofxBulletTriMeshShape >( new ofxBulletTriMeshShape() );
    kinectBulletShape->create( world.world, kinectMesh, ofVec3f(0,0,0), 0.f, ofVec3f(0, 0, 0), ofVec3f(w, h, kinect.maxDistance.getMax()) );
    kinectBulletShape->add();
    kinectBulletShape->enableKinematic();
    kinectBulletShape->setActivationState( DISABLE_DEACTIVATION );
}

//--------------------------------------------------------------
void ofApp::update(){
    // dmx
    dmx.setLevel(1, dmxChannel1);
    
    // kinect
    kinect.update();
    if( kinect.isFrameNew() ){
        texDepth.loadData(kinect.getDepthPixels());
        texRGB.loadData(kinect.getRgbPixels());
        rawDepthPixels = kinect.getRawDepthPixels();
    }
    
    // mesh
    kinectMesh.clear();
    int w = rawDepthPixels.getWidth();
    int h = rawDepthPixels.getHeight();
    float depth;
    
    // add vertex to mesh and save indexes
    vector< vector<int> > indexes;
    int id = 0;
    for (int y = 0; y < h; y += step) {
        vector<int> tempVector;
        for (int x = 0; x < w; x += step) {
            float distance = rawDepthPixels[x+y*w];
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
    for (int y = 0; y < h-step; y += step) {
        for (int x = 0; x < w-step; x += step) {
            if (indexes[y/step][x/step] != -1 &&
                indexes[y/step][x/step+1] !=  -1 &&
                indexes[y/step+1][x/step+1] != -1 &&
                indexes[y/step+1][x/step] != -1) {
                kinectMesh.addTriangle(indexes[y/step][x/step], indexes[y/step][x/step+1], indexes[y/step+1][x/step+1]);
                kinectMesh.addTriangle(indexes[y/step][x/step], indexes[y/step+1][x/step+1], indexes[y/step+1][x/step]);
            }
        }
    }
    
    // bullet
    if (h != 0 && kinectBulletShape == NULL ) {
        setupWhenKinectIsReady();
    }
    
    // kinect
    if (!stopUpdatingKinectBullet) {
        if (h != 0 && kinectBulletShape != NULL) {
            kinectBulletShape->remove();
            kinectBulletShape->create( world.world, kinectMesh, ofVec3f(0,0,0), 0.f, ofVec3f(0, 0, 0), ofVec3f(w, h, kinect.maxDistance.getMax()) );
            kinectBulletShape->add();
            kinectBulletShape->enableKinematic();
            kinectBulletShape->setActivationState( DISABLE_DEACTIVATION );
        }
    }
    
    // model
    if (enableAddModel) {
        // add model in modelStartPosition
        ofxBulletCustomShape *bulletCustomShape;
        bulletCustomShape = new ofxBulletCustomShape;
        ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
        bulletCustomShape->init((btCompoundShape*)assimpModelBulletShapes[0]->getCollisionShape(), assimpModelBulletShapes[0]->getCentroid());
        bulletCustomShape->create(world.world, modelStartPosition, startRot, modelMass);
        bulletCustomShape->add();
        ofVec3f frc(camera.getLookAtDir());
        frc.normalize();
        bulletCustomShape->applyCentralForce(frc*0.005);
        assimpModelBulletShapes.push_back(bulletCustomShape);
    }
    if (enableAddModelRandom) {
        // add model in random x, random y and modelStartPosition.z
        ofxBulletCustomShape *bulletCustomShape;
        bulletCustomShape = new ofxBulletCustomShape;
        ofQuaternion startRot = ofQuaternion(ofRandom(0, 1), ofRandom(0, 1), ofRandom(0, 1), ofRandom(-1*PI, PI));
        bulletCustomShape->init((btCompoundShape*)assimpModelBulletShapes[0]->getCollisionShape(), assimpModelBulletShapes[0]->getCentroid());
        // random square
//        ofVec3f instantModelStartPosition(ofRandom(0, w), ofRandom(0, h), modelStartPosition->z);
        
        // random circle
        float instantRadius = ofRandom(0, 500);
//        float instantRadius = (ofRandom(-500, 500)+ofRandom(-500, 500)+ofRandom(-500, 500)+ofRandom(-500, 500)+ofRandom(-500, 500))/5.0;
        float instantTheta = ofRandom(-PI, PI);
        ofVec3f instantModelStartPosition(w/2.0+instantRadius*cos(instantTheta), h/2.0+instantRadius*sin(instantTheta), modelStartPosition->z);


        bulletCustomShape->create(world.world, instantModelStartPosition, startRot, modelMass);
        bulletCustomShape->add();
        ofVec3f frc(camera.getLookAtDir());
        frc.normalize();
        bulletCustomShape->applyCentralForce(frc*ofRandom(-0.01, 0.01));
        ofVec3f instantTorque(ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005), ofRandom(-0.005, 0.005));
        bulletCustomShape->applyTorque(instantTorque);
        assimpModelBulletShapes.push_back(bulletCustomShape);
    }
    
    // world
    world.setGravity(worldGravity);
    world.update(ofGetLastFrameTime()*2, 20);
    
    // erase model which is in out range(z < 0)
    for( int i = 0; i < assimpModelBulletShapes.size(); i++ ) {
        // FIXME: can not delete assimpModelBulletShapese[0] (maybe better to use shared_ptr. check ofxBullet>SoftBodyMeshExample)
        if (i != 0) {
            if(assimpModelBulletShapes[i]->getPosition().z < 0) {
                assimpModelBulletShapes[i]->remove();
                assimpModelBulletShapes.erase(assimpModelBulletShapes.begin() + i);
                break;
            }else{
                ofRectangle rect(0, 0, w, h);
                if (rect.inside(assimpModelBulletShapes[i]->getPosition().x, assimpModelBulletShapes[i]->getPosition().y)) {
                    float kinectDepth = rawDepthPixels[(int)assimpModelBulletShapes[i]->getPosition().x + (int)assimpModelBulletShapes[i]->getPosition().y*w];
                    if (kinectDepth > kinect.minDistance && kinectDepth < kinect.maxDistance && assimpModelBulletShapes[i]->getPosition().z > kinectDepth) {
                        // add force to the model which is above the kinect mesh
                        if (ofGetMousePressed()) {
                            ofVec3f wind(0, 0, 0.001); // FIXME: hard code
                            assimpModelBulletShapes[i]->applyCentralForce(wind);
                        }
                    }
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
                            enableDrawWireFrame ? kinectMesh.drawWireframe() : kinectMesh.drawFaces();
                        }
                        
                        // spheres
                        for( int i = 0; i < spheres.size(); i++ ) {
                            spheres[i]->draw();
                        }
                        
                        // assimp models
                        ofPoint scale = assimpModelLoader.getScale();
                        ofxAssimpMeshHelper & meshHelper = assimpModelLoader.getMeshHelper(0);
                        // ofMaterial & assimpModelMaterial = meshHelper.material;
                        meshHelper.getTextureRef().bind();{
                            // assimpModelMaterial.begin();{
                                for (int i = 0; i < assimpModelBulletShapes.size(); i++) {
                                    assimpModelBulletShapes[i]->transformGL();{
                                        ofScale(scale.x, scale.y, scale.z);
                                        assimpModelLoader.getCurrentAnimatedMesh(0).drawFaces();
                                    } assimpModelBulletShapes[i]->restoreTransformGL();
                                }
                            // }assimpModelMaterial.end();
                        }meshHelper.getTextureRef().unbind();
                        
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
    ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));
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
        ofRect(panel.getPosition().x, panel.getHeight()+4, debugImageSize.x+2, debugImageSize.y+2);
        texDepth.draw(panel.getPosition().x+1, panel.getHeight()+5, debugImageSize.x, debugImageSize.y);
    }
    
    // - RGB data
    if (showPanel) {
        ofRect(panel.getPosition().x, panel.getHeight()+5+debugImageSize.y+2, debugImageSize.x+2, debugImageSize.y+2);
        texRGB.draw(panel.getPosition().x+1, panel.getHeight()+5+debugImageSize.y+3, debugImageSize.x, debugImageSize.y);
    }
    
    // - info
    //    ofDrawBitmapString("ofxKinectV2: Work in progress addon.\nBased on the excellent work by the OpenKinect libfreenect2 team\n\n-Only supports one Kinect v2 at a time. \n-Requires USB 3.0 port ( superspeed )\n-Requires patched libusb. If you have the libusb from ofxKinect ( v1 ) linked to your project it will prevent superspeed on Kinect V2", 10, 14);
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

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case ' ':{
            // add one sphere
            shared_ptr< ofxBulletSphere > ss( new ofxBulletSphere() );
            ss->create( world.world, camera.getPosition()+100.0f*camera.getLookAtDir(), 0.0000051, 30.0);
            ss->add();
            ofVec3f frc(camera.getLookAtDir());
            frc.normalize();
            // ss->applyCentralForce(frc*0.01);
            spheres.push_back( ss );
        }
            break;
        case 'f':
            ofToggleFullscreen();
            break;
        case 'h':
            showPanel = !showPanel;
            break;
        case 'm':{
            // add model
            ofxBulletCustomShape *bulletCustomShape;
            bulletCustomShape = new ofxBulletCustomShape;
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
            ofVec3f wind(0.01, 0, 0);
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
    showCursor ? ofHideCursor() : ofShowCursor();
    showCursor = !showCursor;
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
