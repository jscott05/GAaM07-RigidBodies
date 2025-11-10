#include "PhysicsEngine.h"
#include "Vector2Utils.h"

using namespace PhysicsUtils;

    PhysicsEngine::PhysicsEngine(float width, float height) : 
        worldWidth(width), worldHeight(height), gravity(0.0f, 500.0f)
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

        //check for collisions between bodies O(n^2) 
        //example: we have 100 bodies = 4950 checks, 200 bodies = 19,900 
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            for (size_t j = i + 1; j < bodies.size(); ++j)
            {
                if (bodies[i]->getIsStatic() && bodies[j]->getIsStatic())
                {
                    continue;
                }
                //at this point here we will do the collision checks where we compare every particle to every other particle
                checkCollision(*bodies[i], *bodies[j]);
            }
        }

        //passing a call on to our particle system. 
        

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

    void checkCollision(RigidBody& body1, RigidBody& body2)
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
                body1.position -= normal * (overlap);
            }
            else if (!body2.isStatic)
            {
                //body 2 is dynamic
                body2.position += normal * (overlap);
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

            //rotation will be added later
            float invInertiaSum = 0.0f;

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
            float impactInensity = std::abs(j) / 100.f;
            sf::Color averageColour(  (body1.getColour().r + body2.getColour().r) / 2,
                (body1.getColour().g + body2.getColour().g) / 2,(body1.getColour().b + body2.getColour().b) / 2);

            //call to particle system to create a burst of colours at the given point, with the given colours


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
            }
            if (!body2.isStatic)
            {
                body2.velocity += impulse / body2.mass;
            }


            //apply friction (tangential component)
            sf::Vector2f tangent = relativeVelocity - normal * velocityAlongNormal;
            if (length(tangent) > 0.001f)
            {
                tangent = normalise(tangent);

                float frictionCoEff = (body1.friction + body2.friction) * 0.5f;
                float jt = -dot(relativeVelocity, tangent);

                if (!body1.isStatic && !body2.isStatic)
                {
                    jt /= (1 / body1.mass + 1 / body2.mass);
                }
                else if (!body1.isStatic)
                {
                    jt /= (1 / body2.mass);
                }
                else if (!body2.isStatic)
                {
                    jt /= (1 / body1.mass);
                }

                //coulombs law: limit friction
                float frictionLimit = std::abs(j * frictionCoEff);
                jt = std::clamp(jt, -frictionLimit, frictionLimit);

                sf::Vector2f frictionImpulse = jt * tangent;
                if (!body1.isStatic)
                {
                    body1.velocity -= frictionImpulse / body1.mass;
                }
                if (!body2.isStatic)
                {
                    body2.velocity += frictionImpulse / body2.mass;
                }

            }
        }

    }

    void draw(sf::RenderWindow& window)
    {
        for (auto& body : bodies)
        {
            body->draw(window);
        }
    }

    void setGravity(const sf::Vector2f& g)
    {
        gravity = g;
        for (auto& body : bodies)
        {
            body->wake();
        }
    }


    //next session we will figure out the call to finding bodies at specific locations

    RigidBody* getBodyAt(const sf::Vector2f& point)
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
};
