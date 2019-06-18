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
#include <p2world.h>
#include <algorithm>


p2World::p2World(p2Vec2 gravity): m_Gravity(gravity)
{
	m_Bodies.resize(MAX_BODY_LEN);
}

void p2World::Step(float dt)
{
	int n = 0;
	std::vector<p2Vec2> positionChanges;
	std::vector<p2Vec2> speedChanges;

	for (p2Body& body : m_Bodies)
	{
		// if(body.GetType()

		// printf("AABB is (%.3f , %.3f) (%.3f , %.3f) \n \n", body.GetAabb().GetBottomLeft().x, body.GetAabb().GetBottomLeft().y, body.GetAabb().GetTopRight().x, body.GetAabb().GetTopRight().y);

		// printf("a body starts in %.3f %.3f \n", body.GetPosition().x, body.GetPosition().x);
		// Calculate forces
		// TODO: Apply angular velocity
		// body.SetLinearVelocity(body.GetLinearVelocity());

		// Apply acceleration
		if (body.GetType() != p2BodyType::NONE)
		{
			n += 1;
			positionChanges.push_back(p2Vec2(0, 0));
			speedChanges.push_back(p2Vec2(0, 0));

			if (body.GetType() != p2BodyType::STATIC)
			{

				if (body.GetType() == p2BodyType::DYNAMIC)
				{

					body.SetLinearVelocity(body.GetLinearVelocity() + m_Gravity * dt);
				}
				body.SetPosition(body.GetPosition() + body.GetLinearVelocity() * dt);
				body.RepositionAabb(body.GetPosition());
			}
		}
		// Apply movement

		// printf("a body is in %.3f %.3f \n \n", body.GetPosition().x, body.GetPosition().y);


		/*
		p2Vec2 bodyLinearVelocity = body.GetLinearVelocity();
		body.SetLinearVelocity(bodyLinearVelocity + (this->m_Gravity * dt *body.GetGravityMultiplier()));
		body.SetPosition(body.GetPosition() + body.GetLinearVelocity());
		*/
	}

	if (m_Bodies.size() > 1)
	{
		for (int i = 0; i < n - 1; i++)
		{
			// std::printf("On a un client en %f %f \n", m_Bodies[0].GetAabb().bottomLeft.x, m_Bodies[0].GetAabb().bottomLeft.y);
			for (int j = 1+i; j < n; j++)
			{
				if (m_Bodies[i].GetType() == p2BodyType::STATIC && m_Bodies[j].GetType() == p2BodyType::STATIC)
					continue;
				if (m_Bodies[i].GetType() == p2BodyType::KINEMATIC && m_Bodies[j].GetType() == p2BodyType::STATIC)
					continue;
				if (m_Bodies[i].GetType() == p2BodyType::STATIC && m_Bodies[j].GetType() == p2BodyType::KINEMATIC)
					continue;
				if (m_Bodies[i].GetType() == p2BodyType::KINEMATIC && m_Bodies[j].GetType() == p2BodyType::KINEMATIC)
					continue;
				
				// std::printf("COLLISION A check %f %f \n", m_Bodies[i].GetAabb().GetBottomLeft().y, m_Bodies[j].GetAabb().GetBottomLeft().y);

				if (AabbContact(m_Bodies[i].GetAabb(), m_Bodies[j].GetAabb()))
				{
					if (m_Bodies[i].GetShapeType() == CIRCLE && m_Bodies[j].GetShapeType() == CIRCLE)
					{
						float addedRadius = m_Bodies[i].GetCircle().GetRadius() + m_Bodies[j].GetCircle().GetRadius();
						float distance = (m_Bodies[i].GetPosition() - m_Bodies[j].GetPosition()).GetMagnitude();
						if (addedRadius > distance)
						{
							p2Vec2 normal = m_Bodies[i].GetPosition() - m_Bodies[j].GetPosition();
							normal.NormalizeSelf();

							positionChanges[i] += normal * (addedRadius - distance) / 2;
							positionChanges[j] -= normal * (addedRadius - distance) / 2;

							speedChanges[i] += m_Bodies[j].GetLinearVelocity() - m_Bodies[i].GetLinearVelocity();
							speedChanges[j] += m_Bodies[i].GetLinearVelocity() - m_Bodies[j].GetLinearVelocity();

							std::printf("CIRCULAIRE X CIRCULAIRE COLLISIONNE \n");
						}
					}

					if (m_Bodies[i].GetShapeType() == RECT && m_Bodies[j].GetShapeType() == RECT)
					{
						std::printf("Double Rect COLLISIONNE \n");

						p2Vec2 normal = RectRectCollisionNormal(m_Bodies[i].GetAabb(), m_Bodies[j].GetAabb());
						positionChanges[i] += normal / 2;
						positionChanges[j] -= normal / 2;

						speedChanges[i] += m_Bodies[j].GetLinearVelocity() - m_Bodies[i].GetLinearVelocity();
						speedChanges[j] += m_Bodies[i].GetLinearVelocity() - m_Bodies[j].GetLinearVelocity();
					}

					if (m_Bodies[i].GetShapeType() == CIRCLE && m_Bodies[j].GetShapeType() == RECT)
					{
						float radius = m_Bodies[j].GetCircle().GetRadius();
						p2Vec2 normal = m_Bodies[j].GetPosition() - m_Bodies[i].GetPosition();
						normal.NormalizeSelf();
						p2Vec2 closestPointToRect = normal * radius + m_Bodies[i].GetPosition();
						if (radius <= (m_Bodies[j].GetPosition() - m_Bodies[i].GetPosition()).GetMagnitude() ||
																		((closestPointToRect.x > m_Bodies[j].GetAabb().bottomLeft.x && closestPointToRect.x < m_Bodies[j].GetAabb().topRight.x) && 
																		(closestPointToRect.y > m_Bodies[j].GetAabb().bottomLeft.y && closestPointToRect.x < m_Bodies[j].GetAabb().topRight.y)))
						{
							p2Vec2 normal = m_Bodies[i].GetPosition() - m_Bodies[j].GetPosition();

							positionChanges[i] += normal / 2;
							positionChanges[j] -= normal / 2;

							speedChanges[i] += m_Bodies[j].GetLinearVelocity() - m_Bodies[i].GetLinearVelocity();
							speedChanges[j] += m_Bodies[i].GetLinearVelocity() - m_Bodies[j].GetLinearVelocity();

							std::printf("CIRCULAIRE X Rect touche \n");
						}
					}

					if (m_Bodies[j].GetShapeType() == CIRCLE && m_Bodies[i].GetShapeType() == RECT)
					{
						float radius = m_Bodies[i].GetCircle().GetRadius();
						p2Vec2 normal = m_Bodies[i].GetPosition() - m_Bodies[j].GetPosition();
						normal.NormalizeSelf();
						p2Vec2 closestPointToRect = normal * radius + m_Bodies[j].GetPosition();
						if (radius <= (m_Bodies[i].GetPosition() - m_Bodies[j].GetPosition()).GetMagnitude() ||
							((closestPointToRect.x > m_Bodies[i].GetAabb().bottomLeft.x && closestPointToRect.x < m_Bodies[i].GetAabb().topRight.x) &&
							(closestPointToRect.y > m_Bodies[i].GetAabb().bottomLeft.y && closestPointToRect.x < m_Bodies[i].GetAabb().topRight.y)))
						{
							p2Vec2 normal = m_Bodies[j].GetPosition() - m_Bodies[i].GetPosition();

							positionChanges[i] += normal / 2;
							positionChanges[j] -= normal / 2;

							speedChanges[i] += m_Bodies[j].GetLinearVelocity() - m_Bodies[i].GetLinearVelocity();
							speedChanges[j] += m_Bodies[i].GetLinearVelocity() - m_Bodies[j].GetLinearVelocity();

							std::printf("CIRCULAIRE X Rect touche \n");
						}
					}
				}
			}

			// Pourquoi mes AABB se checkent pas ?

		}
	}

	n = 0;

	for (p2Body& body : m_Bodies)
	{
		if (body.GetType() == p2BodyType::DYNAMIC)
		{
			body.SetPosition(body.GetPosition() + positionChanges[n]);

			std::printf("speed given %f \n", speedChanges[n].GetMagnitude());

			body.SetLinearVelocity(body.GetLinearVelocity() + speedChanges[n] * 0.5);
		}
		if (body.GetType() != p2BodyType::NONE)
		{
			n += 1;
		}
	}
	
	// Quadtree

	// Check for collision
}

p2Body * p2World::CreateBody(p2BodyDef* bodyDef)
{
	p2Body& body = m_Bodies[m_BodyIndex];
	body.Init(bodyDef);
	m_BodyIndex++;
	return &body;
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
}

bool p2World::AabbContact(p2AABB aabb1, p2AABB aabb2)
{
	// std::printf("check la collision entre %f %f , %f %f et %f %f , %f %f \n", aabb1.GetBottomLeft().x, aabb1.GetBottomLeft().y, aabb1.GetTopRight().x,aabb1.GetTopRight().y,
	//																					aabb2.GetBottomLeft().x, aabb2.GetBottomLeft().y, aabb2.GetTopRight().x, aabb2.GetTopRight().y);

	// If one rectangle is on left side of other 
	if (aabb1.GetBottomLeft().x > aabb2.GetTopRight().x || aabb2.GetBottomLeft().x > aabb1.GetTopRight().x)
	{
		return false;
	}

	// If one rectangle is above other 
	if (aabb1.GetBottomLeft().y > aabb2.GetTopRight().y || aabb2.GetBottomLeft().y > aabb1.GetTopRight().y)
	{
		return false;
	}

	return true;
}

p2Vec2 p2World::RectRectCollisionNormal(p2AABB rect1, p2AABB rect2)
{
	p2Vec2 bottomLeft;
	p2Vec2 topRight;

	p2Vec2 rect1Rect2;

	bottomLeft.x = std::max(rect1.GetBottomLeft().x ,rect2.GetBottomLeft().x);
	bottomLeft.y = std::max(rect1.GetBottomLeft().y , rect2.GetBottomLeft().y);
	topRight.x = std::min(rect1.GetTopRight().x , rect2.GetTopRight().x);
	topRight.y = std::min(rect1.GetTopRight().y , rect2.GetTopRight().y);

	if (bottomLeft.x == rect1.GetBottomLeft().x)
	{
		if (bottomLeft.y == rect1.GetBottomLeft().y)
		{
			rect1Rect2 = topRight - bottomLeft;
		}
		else
		{
			if (topRight.y == rect1.GetTopRight().y)
			{
				rect1Rect2.x = topRight.x - bottomLeft.x;
				rect1Rect2.y = bottomLeft.y - topRight.y;
			}
		}
	}

	if (topRight.x == rect1.GetTopRight().x)
	{
		if (bottomLeft.y == rect1.GetBottomLeft().y)
		{
			rect1Rect2.x = topRight.x - bottomLeft.x;
			rect1Rect2.y = bottomLeft.y - topRight.y;
		}
		else
		{
			if (topRight.y == rect1.GetTopRight().y)
			{
				rect1Rect2 = topRight - bottomLeft;
			}
		}
	}

	return rect1Rect2;
}
