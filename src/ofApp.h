#pragma once

#define DMX_CHANNEL_NUMBER 2
#define DMX_TIMER_PER_SECONDS 10
#define MODEL_PLAY_SECONDS 164
#define MODEL_INTERVAL_SECONDS 16
#define MODEL_NUMBER 5
#define PROBABILITY_FACTOR_MIN_FACTOR 0.1
#define PROBABILITY_FACTOR_MAX_FACTOR 10000

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"
#include "ofxBullet.h"
#include "ofxAssimpModelLoader.h"
#include "ofxDmx.h"
#include "ofxTween.h"
#include "ofxSimpleTimer.h"

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
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    void setupWhenKinectIsReady();
    bool valueIsInKinectRange(float value);
    void doEase(ofParameter<int> dmxChannel, unsigned duration, unsigned delay);
    void changeAssimpModel(int modelId);
    void updateKinectMesh();
    // listener
    void resetPressed();
    void enableSmoothLightingChanged(bool &enableSmoothLightingStatus);
    void enableScanPeopleChanged(bool &enableScanPeople);
    void saveReferenceDepthPixelsPressed();
    void timerComplete(string &name);
    
    // gui
    ofxPanel panel;
    bool showPanel;

    //- kinect
    ofParameter<int> step;
    ofParameter<bool> stopUpdatingKinectBullet;
    ofParameter<bool> enableDrawDebug;
    ofParameter<bool> enableDrawKinectWireFrame;
    ofParameter<bool> enableDrawAssimpModelWireFrame;
    ofParameter<bool> hideKinectMesh;
    ofParameter<bool> enableDrawGuideLine;
    ofParameter<bool> enableMouseInput;
    ofParameter<bool> enableDrawDebugSpheres;
    // - camera
    ofParameter<ofVec3f> cameraPosition;
    ofParameter<ofVec3f> cameraLookAt;
    ofParameter<float> cameraFov;
    ofParameter<float> cameraNearDist;
    ofParameter<float> cameraFarDist;
    // - light
    ofParameter<ofFloatColor> lightSpecularColor;
    ofParameter<ofFloatColor> lightDissuseColor;
    ofParameter<ofFloatColor> lightAmbientColor;
    ofParameter<ofVec3f> lightPosition;
    ofParameter<ofVec3f> lightAttenuation;
    ofParameter<bool> enableSmoothLighting;
    ofParameter<bool> enableSeparateSpecularLight;
    // - material
    ofParameter<ofFloatColor> materialSpecularColor;
    ofParameter<ofFloatColor> materialDiffuseColor;
    ofParameter<ofFloatColor> materialAmbientColor;
    ofParameter<ofFloatColor> materialEmissiveColor;
    ofParameter<float> materialShininess;
    ofParameter<bool> enableAddModel;
    ofParameter<bool> enableAddModelRandom;
    // - world
    ofParameter<ofVec3f> worldGravity;
    ofParameter<ofVec3f> modelStartPosition;
    ofParameter<float> modelMass;
    // - dmx
    ofParameter<bool> enableDmx;
    ofParameter<int> dmxChannels[DMX_CHANNEL_NUMBER];
    // - scanning cloth or scanning people
    ofParameter<bool> enableScanPeople;
    ofxButton saveReferenceDepthPixels;
    ofParameter<float> probabilityFactor;
    // - reset
    ofxButton reset;
    
    // kinect
    ofxKinectV2 kinect;
    ofTexture texDepth;
    ofTexture texRGB;
    ofFloatPixels rawDepthPixels;
    ofTexture savedReferenceDepthTexture;
    ofFloatPixels savedReferenceDepthPixels;
    ofFloatPixels diffDepthPixels;
    ofTexture diffDepthTexture;
    unsigned char *diffDepthTexturePixels;
    ofMesh kinectMesh;
    int kinectWidth;
    int kinectHeight;
    int kinectDepth;
    bool kinectIsReady;
    // camera
    ofEasyCam camera;
    
    // 3d model
    ofxAssimpModelLoader assimpModelLoader;
    int currentModelId;
    
    // bullet
    ofxBulletWorldRigid world;
    shared_ptr<ofxBulletTriMeshShape> kinectBulletShape;
    vector<shared_ptr<ofxBulletSphere> > spheres;
    vector<ofxBulletCustomShape*> assimpModelBulletShapes;
    vector<ofVec3f> modelStartPositions;
    
    // light
    ofLight light;
    
    // material
    ofMaterial material;
    
    // dmx
    ofxDmx dmx;
    
    // tween
    ofxTween tween;
    ofxEasingSine easingSine;
    
    // timer
    ofxSimpleTimer timer;
    
    // debug
    // - debug spheres
    vector<ofSpherePrimitive> debugSpheres;
    // - camera target
    ofSpherePrimitive debugSphereCameraTarget;
};
