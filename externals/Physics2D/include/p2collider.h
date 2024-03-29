/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef SFGE_P2COLLIDER_H
#define SFGE_P2COLLIDER_H

#include <p2shape.h>
#include "engine/entity.h"
#include <p2aabb.h>

/**
* \brief Struct defining a p2Collider when creating one
*/
struct p2ColliderDef
{
	void* userData;
	p2Shape* shape;
	float restitution;
	bool isSensor = false;
};

/**
* \brief Representation of a Collider attached to a p2Body
*/

class p2Collider
{
public:
	ShapeType GetShapeType();
	p2Collider(p2ColliderDef colDef = {nullptr,nullptr,0.0f,false});
	void init(p2ColliderDef * colDef);
	/**
	* \brief Check if the p2Collider is a sensor
	*/
	bool IsSensor() const;
	/**
	* \brief Return the userData
	*/
	void* GetUserData() const;
	p2Shape* GetShape() const;
	void SetUserData(void* colliderData);
	p2AABB BuildAABBCollider(p2Vec2 position);
	
	p2RectShape GetRect();
	p2CircleShape GetCircle();
	
private:
	ShapeType m_ShapeType;
	p2RectShape rectShape;
	p2CircleShape circleShape;
	void* userData = nullptr;
	p2ColliderDef colliderDefinition;
};


#endif