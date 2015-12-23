#ifndef _WZTERRAINMAP_H
#define _WZTERRAINMAP_H

class MapController
{
public:
    void setMapSize(int width, int height){this->width = width; this->height = height;}
    
    int getWidth()const{return this->width;}
    int getHeight()const{return this->height;}
    
    bool isInMap(int x, int y)const;
    
public:
	MapController(int width, int height);
	~MapController();
    
	void render();
    
	static const double MapGridSize;
	static const double restDensity;
    
private:
    int width;
    int height;
};

#endif