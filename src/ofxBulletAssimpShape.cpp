#include "ofxBulletAssimpShape.h"

ofxBulletAssimpShape::ofxBulletAssimpShape(){
    modelIndex[0] = 0;
    modelIndex[1] = 0;
}

ofxBulletAssimpShape::ofxBulletAssimpShape(vector<int> _assimpModelIds){
    modelIndex[0] = 0;
    modelIndex[1] = 0;
    assimpModelIds = _assimpModelIds;
}

ofxBulletAssimpShape::ofxBulletAssimpShape(vector<int> _assimpModelIds, int setId, int index){
    assimpModelIds = _assimpModelIds;
    modelIndex[0] = setId;
    modelIndex[1] = index;
}

ofxBulletAssimpShape::ofxBulletAssimpShape(int setId, int index){
    modelIndex[0] = setId;
    modelIndex[1] = index;
}


vector<int> ofxBulletAssimpShape::getAssimpModelIds(){
    return assimpModelIds;
}

int ofxBulletAssimpShape::getSetId(){
    return modelIndex[0];
}

int ofxBulletAssimpShape::getIndex(){
    return modelIndex[1];
}

ofxBulletAssimpShape::~ofxBulletAssimpShape(){

}
