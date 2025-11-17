#include "PhysicsEngine.h"
#include "Vector2Utils.h"

using namespace PhysicsUtils;

PhysicsEngine::PhysicsEngine(float width, float height) :
	worldWidth(width), worldHeight(height), gravity(0.0f, 500.0f), spatialGrid(width, height, 100.0f)
    {

    }


void PhysicsEngine::addBody(std::unique_ptr<RigidBody> body)
    {
        bodies.push_back(std::move(body));
    }

    void PhysicsEngine::clearDynamicBodies()
    {
        bodies.erase(
            std::remove_if(
                bodies.begin(),
                bodies.end(),
                [](const std::unique_ptr<RigidBody>& body) { return !body->isStatic; }),
            bodies.end());
    }

    size_t PhysicsEngine::getDynamicBodyCount() const
    {
        return std::count_if(bodies.begin(), bodies.end(),
            [](const std::unique_ptr<RigidBody>& body) 
            { return !body->getIsStatic(); });
    }

    void PhysicsEngine::update(float deltaTime)
    {
        //first we applied gravity to aall bodies
        for (auto& body : bodies)
        {
            if (!body->isStatic && !body->isResting)
            {
                body->applyForce(gravity * body->mass);
            }
            body->updateCollisionInfo(deltaTime);
        }


        //update  each particles position
        for (auto& body : bodies)
        {
            body->update(deltaTime);
            body->checkBoundaryCollision(worldWidth, worldHeight);
        }

		updateSpatialGrid();
		auto potentialCollisions = spatialGrid.getPotentialCollisions();

        //check for collisions between bodies O(n^2) 
        //example: we have 100 bodies = 4950 checks, 200 bodies = 19,900 
        for (const auto& pair : potentialCollisions)
        {
            if ((pair.first->getIsStatic() || pair.first->getIsResting())
				&& (pair.second->getIsStatic() || pair.second->getIsResting()))
            {
                continue;
            }

			checkCollision(*pair.first, *pair.second);

        }

        //passing a call on to our particle system. 
       
		particleSystem.update(deltaTime);
    }



    //circle to circle collision detection, this is a geometric test
    //position correction (seperate overlapping objects)
    //impulse based collision response (velocity change)
    //friction calculation 

    //instead of applying forces over time (f = ma) instead we apply instant velocity changes
    // impulse j = change in momentum = Δ(mv)
    //for an instantaneous collision this is more stable than a force based
    //style is used in modern game engines such as bullet, physx or box2d
    //
    //newtons third law 
    //for every action, there is an equal and opposite reaction
    //body 1 - receives impulse of j 
    //body 2 - receives the impulse of -j (same magnitude, but in an opposite direction)
    //the total momentum is conserved 

    void PhysicsEngine::checkCollision(RigidBody& body1, RigidBody& body2)
    {
        //step 1 - collision detection ( NARROW PHASE )
        sf::Vector2f difference = body2.position - body1.position;

        float distance = length(difference); //distance between centers 

        float minDistance = body1.radius + body2.radius;//sum of radii

        //if the circles are overlapping
        if (distance < minDistance)
        {
            //edge case: bodies exactly on top of each other? 
            // prevent a division by zero when we call to normalise

            if (distance < 0.001f) //prevent division by zero for any overlapping bodies
            {
                distance = 0.001f;
                difference = sf::Vector2f(1.0f, 0.0f) * minDistance; //pushing them apart horizontally
            }
            //wake up resting bodies
            body1.wake();
            body2.wake();



            //step 2 - calculate the collision geometry
            // we need the colllision normal: the direction to push the bodies apart
            // points between body1 and body2
            // 
            // Body 1 -> normal -> body2
            //   0  ----------------> 0
            // 
            //collision detected
            sf::Vector2f normal = normalise(difference);


            //need to look out for overlap/penetration: how much the circles are intersecting 
            // example radii 20 and 30, and the distance is 45
            // - mindistance = 50 (they should be at least 50 apart) 
            // - overlap = 50 - 45 = 5 pixels of penetration 
            // 
            //seperate the bodies
            float overlap = minDistance - distance;

            //seperation factor slightly over-seperated to prevent jitter
            float seperationFactor = 1.01f;

            //CONTACT POINT - where the collision occured, located on body1s surface, along the collision normal
            sf::Vector2f contactPoint = body1.getPosition() + normal * body1.getRadius();
            body1.addCollisionInfo(contactPoint, -normal, overlap);
            body2.addCollisionInfo(contactPoint, normal, overlap);


            //step 3: Position correction
            //seperate the overlapping bodies, this aims to prevent objects from getting
            // "stuck" inside each other

            //1, both dyanmic: split the seperation (Each move by half) 
            //2, one is static so only move the dynamic body 
            //3, both are static so nothing moves

            if (!body1.isStatic && !body2.isStatic)
            {
                //both dynamic
                body1.position -= normal * (overlap * 0.5f * seperationFactor);
                body2.position += normal * (overlap * 0.5f * seperationFactor);
            }
            else if (!body1.isStatic)
            {
                //body 1 is dynamic
                body1.position -= normal * (overlap * seperationFactor);
            }
            else if (!body2.isStatic)
            {
                //body 2 is dynamic
                body2.position += normal * (overlap * seperationFactor);
            }
            //case 3, both are static 


            //step 4 calculate velocity at contact point 
            //for rotating object, velocity is at contact point !== velocity at center

            //formaula v_contact = v_Center + w * r
            // w (omega) aka angular velocity
            // r = vector from the center to the contact point
            // x = cross product (in 2d: produces perpendicular velocity) 
            //2d cross product 
            // w x r = (-w * r.y, w * r.x)
            // 
            // spinning basketball hits the floor, center might be moving horizontally,
            // but the contact point also has velovityy from the spin 
            // total velocity = linear + rotational components
            // 
            // 
            sf::Vector2f r1 = contactPoint - body1.getPosition(); //radius vector to contact
            sf::Vector2f r2 = contactPoint - body2.getPosition();
                
            //calculate the velocity at the contact point ( linear + rotational)
            sf::Vector2f v1 = body1.getVelocity() + sf::Vector2f(-body1.getAngularVelocity() * r1.y,
                body1.getAngularVelocity() * r1.x);
            //calculate the velocity at the contact point ( linear + rotational)
            sf::Vector2f v2 = body2.getVelocity() + sf::Vector2f(-body2.getAngularVelocity() * r2.y,
                body2.getAngularVelocity() * r2.x);



            //calculate relative velocity : how fast objects are approaching/seperating
            //positive along normal = seperating
            //negative along the normal = approaching
            sf::Vector2f relativeVelocity = v2 - v1;
            float velocityAlongNormal = dot(relativeVelocity, normal);

            //if the objects are already seperating, dont apply impulse 
            if (velocityAlongNormal < 0)
            {
                return;
            }
            

            //step 5 - calculate impulse magnitude 
            
            //calculate the restitution
            float e = std::min(body1.restitution, body2.restitution);

            //e == the coefficient of restitution
            //aka how bouncy the collision is
            //if e = 0 perfectly inelastic, on collision objects stick together
            //if e = 1: perfectly elastic (100% energy conversion)

            //inverse mass sum 
            //for cillision resolution we tend to work with inverse mass (1/m)
            //static objects have infinite mass, so 1/infinite = 0 (easy to handle)
            //these sums will determine how much the impulse affects linear motion

            float invMassSum = 0.0f;
            if (!body1.getIsStatic()) invMassSum += 1.f / body1.getMass();
            if (!body2.getIsStatic()) invMassSum += 1.f / body2.getMass();


			//cross product - scalar for rotation effects
			float r1CrossN = cross(r1, normal);
			float r2CrossN = cross(r2, normal);

            //rotation will be added later
            float invInertiaSum = 0.0f;
			if (!body1.getIsStatic()) invInertiaSum += (r1CrossN * r1CrossN) / body1.getInertia();
            if (!body2.getIsStatic()) invInertiaSum += (r2CrossN * r2CrossN) / body2.getInertia();
            //impulse formula (derived from conservation of momentum and energy)
            // j = -(1 + e) * v_rel_n / (1/m1 + 1/m2 + rotational_terms)

            //where:
            // j = impulse magnitude (scalar)
            //e = coefficient of restitution 
            //v_rel_n = relative velocity along the normal
            //rotational terms angular effects from the cross products



            //calculate the impulse scalar
            float j = -(1 + e) * velocityAlongNormal;
            j /= (invMassSum * invInertiaSum);


            //impulse vector : direction and magnitude - impulse points along the collision normal 
            //magnitude j was calculated ^^^ 
            sf::Vector2f impulse = j * normal;

            //visual sugar
            //create a particle burst at the point of impact
            float impactIntensity = std::abs(j) / 100.f;
            sf::Color averageColour(  (body1.getColour().r + body2.getColour().r) / 2,
                (body1.getColour().g + body2.getColour().g) / 2,(body1.getColour().b + body2.getColour().b) / 2);

            //call to particle system to create a burst of colours at the given point, with the given colours
            particleSystem.createImpactBurst(contactPoint, normal, averageColour, std::min(1.0f, impactIntensity));

            //step 6: APPLY the IMPULSE - newtons third law!

            // "For every action, there is an equal and opposite reaction"
            // velocity change = impulse / mass
            // Δv = j / m 
            // calculate angular velocity change from impulse
            //Δw = (r x j) / I (change in angular velocity = torque / inertia) 

            //key points 
            //body 1 gets negative impulse (negative direction)
            //body 2 gets positive impulse (positive direction)

            //equal magnitude, opposite directions == newtons 3rd law
            //heavier objects change velocity less ( larger m in denominator)
            //off-center hits create more spin


            if (!body1.isStatic)
            {
                body1.velocity -= impulse / body1.mass;
				body1.setAngularVelocity(body1.getAngularVelocity() - cross(r1, impulse) / body1.getInertia());
            }
            if (!body2.isStatic)
            {
                body2.velocity += impulse / body2.mass;
				body2.setAngularVelocity(body2.getAngularVelocity() + cross(r2, impulse) / body2.getInertia());
            }


            //apply friction (tangential component)
            sf::Vector2f tangent = relativeVelocity - normal * velocityAlongNormal;
            if (length(tangent) > 0.001f)
            {
                tangent = normalise(tangent);

				//average friction coefficient of both materials
                float frictionCoEff = (body1.friction + body2.friction) * 0.5f;
                
				// friction impulse magnitude
				// similar formula to normal impulse but using tangent direction
                float jt = -dot(relativeVelocity, tangent);

                // coulomb friction law
				// friction force limited by normal force
				// mu = coefficient of friction
				// f_friction < mu * f_normal

				float frictionLimit = std::abs(j * frictionCoEff);
				jt = std::clamp(jt, -frictionLimit, frictionLimit);
                


                sf::Vector2f frictionImpulse = jt * tangent;
                if (!body1.isStatic)
                {
                    body1.velocity -= frictionImpulse / body1.mass;
					body1.setAngularVelocity(body1.getAngularVelocity() - cross(r1, frictionImpulse) / body1.getInertia());
                }
                if (!body2.isStatic)
                {
                    body2.velocity += frictionImpulse / body2.mass;
					body2.setAngularVelocity(body2.getAngularVelocity() + cross(r2, frictionImpulse) / body2.getInertia());
                }

            }
        }
    }

    void PhysicsEngine::updateSpatialGrid()
    {
        spatialGrid.clear();
        for (auto& body : bodies)
        {
            spatialGrid.insert(body.get());
		}
    }

    void PhysicsEngine::drawBatchedGlows(sf::RenderWindow& window)
    {
		glowVertices.clear();
        glowVertices.setPrimitiveType(sf::PrimitiveType::Triangles);

        const int glowLayers = 3;
        const int segments = 16;

        for (const auto& body : bodies)
        {
			if (body->getIsStatic()) continue;

            sf::Vector2f position = body->getPosition();
			float radius = body->getRadius();
            sf::Color colour = body->getColour();
            float impactIntensity = body->impactIntensity;
			bool isResting = body->getIsResting();
            
            // calculate display colour
            sf::Color displayColour = isResting ? 
                sf::Color(colour.r / 2, colour.g / 2, colour.b / 2) : colour;

            float flashIntensity = impactIntensity * 100.0f;

			displayColour.r = std::min(255, static_cast<int>(displayColour.r + flashIntensity));
			displayColour.g = std::min(255, static_cast<int>(displayColour.g + flashIntensity));
            displayColour.b = std::min(255, static_cast<int>(displayColour.b + flashIntensity));

            // draw glow layers as triangle fans

            for(int layer = glowLayers; layer > 0; --layer)
            {
                float glowRadius = radius + (layer * 4.0f) + (impactIntensity * 5.0f);

				float alpha = isResting ? 10.0f : 20.0f;
                alpha = alpha / (layer + 1) + (impactIntensity * 30.0f);

                sf::Color glowColour(displayColour.r, displayColour.g, displayColour.b, static_cast<uint8_t>(alpha));
			
                for (int i = 0; i < segments; ++i)
                {
                    // create triangle fan for circle
                    float angle1 = (i * 2.0f * 3.14159265f) / segments;
					float angle2 = ((i + 1) * 2.0f * 3.14159265f) / segments;

					sf::Vector2f p1 = position + sf::Vector2f(std::cos(angle1) * glowRadius, std::sin(angle1) * glowRadius);
                    sf::Vector2f p2 = position + sf::Vector2f(std::cos(angle2) * glowRadius, std::sin(angle2) * glowRadius);

					glowVertices.append(sf::Vertex(position, glowColour));
					glowVertices.append(sf::Vertex(p1, glowColour));
                    glowVertices.append(sf::Vertex(p2, glowColour));
                }
			}

        }

        window.draw(glowVertices);
    }

    void PhysicsEngine::drawBatchedTrails(sf::RenderWindow& window)
    {
        trailVertices.clear();
        trailVertices.setPrimitiveType(sf::PrimitiveType::Triangles);

        for (const auto& body : bodies)
        {
			const auto& trail = body->getMotionTrail();
            sf::Color colour = body->getColour();
			if (trail.size() < 2) continue;

            for(size_t i = 1; i < trail.size(); ++i)
            {
				uint8_t alpha = static_cast<uint8_t>(trail[i].alpha * 250);

				sf::Color trailColour(colour.r, colour.g, colour.b, alpha);

				trailVertices.append(sf::Vertex(trail[i - 1].position, trailColour));
                trailVertices.append(sf::Vertex(trail[i].position, trailColour));
			}
        }

        window.draw(trailVertices);

    }

    void PhysicsEngine::draw(sf::RenderWindow& window, bool showVelocity, bool showTrails, bool showDebug)
    {
        // draw particles
        drawBatchedGlows(window);
        // draw motion trails
        if (showTrails)
        {
            for (auto& body : bodies)
            {
                body->drawMotionTrail(window);
            }
        drawBatchedTrails(window);
        }

        // draw bodies (glows, main shapes, cores, rotation indicators)
        for (auto& body : bodies)
        {
            body->draw(window, showVelocity);
        }

        // debug visualisations
        if (showDebug)
        {
            for (auto& body : bodies)
            {
                body->drawDebug(window);
            }
        }
    }

    void PhysicsEngine::setGravity(const sf::Vector2f& g)
    {
        gravity = g;
        for (auto& body : bodies)
        {
            body->wake();
        }
    }


    //next session we will figure out the call to finding bodies at specific locations

    RigidBody* PhysicsEngine::getBodyAt(const sf::Vector2f& point)
    {
        for (auto& body : bodies)
        {
            sf::Vector2f diff = body->position - point;
            if (length(diff) <= body->radius)
            {
                return body.get();
            }
        }
        return nullptr;
    }

