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
#include <p2body.h>

void p2Body::Init(p2BodyDef* bodyDef)
{
	this->position = bodyDef->position;
	this->linearVelocity = bodyDef->linearVelocity;
	this->type = bodyDef->type;
	
	m_Colliders.resize(MAX_COLLIDER_LEN);
}

p2Vec2 p2Body::GetLinearVelocity() const
{
	return linearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	linearVelocity = velocity;
}
float p2Body::GetAngularVelocity()
{
	return angularVelocity;
}

p2Vec2 p2Body::GetPosition()
{
	return position;
}

p2Collider * p2Body::CreateCollider(p2ColliderDef * colliderDef)
{
	p2Collider& collider = m_Colliders[m_ColliderIndex];
	m_ColliderIndex++;
	collider.init(colliderDef);
	aabb = collider.BuildAABBCollider(GetPosition());
	/*
	if (collider.GetShape()->type == CIRCLE)
	{
		p2CircleShape shape = collider.GetShape()->type;
		p2Vec2 radius = p2Vec2(shape.GetRadius(), shape.GetRadius());
		aabb.topRight = GetPosition() + radius;
		aabb.bottomLeft = GetPosition() - radius;
	}
	
	if (collider.GetShape()->type == RECT)
	{
		p2RectShape shape;
		ShapeType myShape;

		shape = collider.GetShape()->type;
	}
	*/
	// colliderDef->shape.
	return &collider;
}

void p2Body::ApplyForceToCenter(const p2Vec2& force)
{
	linearVelocity += force;
}

void p2Body::SetPosition(const p2Vec2 position)
{
	this->position = position;
}

p2BodyType p2Body::GetType() const
{
	return type;
}

float p2Body::GetMass() const
{
	return 0.0f;
}

p2AABB p2Body::GetAabb()
{
	return aabb;
}

void p2Body::RepositionAabb(p2Vec2 pos)
{
	aabb.update(pos);
}

ShapeType p2Body::GetShapeType()
{
	return m_Colliders[0].GetShapeType();
}

int p2Body::GetCollSize()
{
	return m_ColliderIndex;
}

p2Collider p2Body::GetCol()
{
	return m_Colliders[0];
}

p2CircleShape p2Body::GetCircle()
{
	return m_Colliders[0].GetCircle();
}

p2RectShape p2Body::GetRect()
{
	return m_Colliders[0].GetRect();
}

/*
p2AABB * p2Body::CreateRectAabb(p2RectShape shape)
{
	p2AABB& aabb{shape.};
	return &aabb;
}
*/
