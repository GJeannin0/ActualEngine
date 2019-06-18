#include "..\include\p2collider.h"
#include <iostream>



p2Collider::p2Collider(p2ColliderDef colDef)
{
	colliderDefinition = colDef;

	std::printf("size set AHHHHHHHHHHHHHH Xspecial \n");
}

bool p2Collider::IsSensor() const
{
	return colliderDefinition.isSensor;
}

void * p2Collider::GetUserData() const
{
	return userData;
}

p2Shape* p2Collider::GetShape() const
{
	return colliderDefinition.shape;
}

void p2Collider::SetUserData(void* colliderData)
{
	userData = colliderData;
}

p2AABB p2Collider::BuildAABBCollider(p2Vec2 position)
{
	p2AABB aabb;
	auto type = GetShape()->type; // Exception thrown
	p2Vec2 extend;
	switch (type)
	{
	case ShapeType::CIRCLE:
	{
		p2CircleShape* circleShape = static_cast<p2CircleShape*>(GetShape());
		extend = p2Vec2(circleShape->GetRadius(), circleShape->GetRadius());
	}
	break;
	case ShapeType::RECT:
	{
		p2RectShape* rectShape = static_cast<p2RectShape*>(GetShape());
		extend = rectShape->GetSize();
	}
	break;
	}
	aabb.bottomLeft = (position - extend);
	aabb.topRight = (position + extend);

	return aabb;
}

void p2Collider::init(p2ColliderDef * colDef)
{
	colliderDefinition = *colDef;
	m_ShapeType = colDef->shape->type;
	if(m_ShapeType == CIRCLE)
		circleShape = *static_cast<p2CircleShape*>(GetShape());
	if (m_ShapeType == RECT)
	rectShape = *static_cast<p2RectShape*>(GetShape());
}

ShapeType p2Collider::GetShapeType()
{
	return m_ShapeType;
}

p2RectShape p2Collider::GetRect()
{
	return rectShape;
}

p2CircleShape p2Collider::GetCircle()
{
	return circleShape;
}