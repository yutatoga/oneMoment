#pragma once
#include "ofMain.h"
#include "ofxBulletAssimpShape.h"
#include "ofxBulletCustomShape.h"

class ofxBulletAssimpShape : public ofxBulletCustomShape{
public:
    ofxBulletAssimpShape();
    ofxBulletAssimpShape(vector<int> _assimpModelIds);
    ofxBulletAssimpShape(vector<int> _assimpModelIds, int setId, int index);
    ofxBulletAssimpShape(int setId, int index);
    ~ofxBulletAssimpShape();
    
    vector<int> getAssimpModelIds();
    int getSetId();
    int getIndex();
    
    vector<int> assimpModelIds;
    int modelIndex[2];
};