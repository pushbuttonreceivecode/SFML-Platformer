#include <CollisionSystem.hpp>

const int X_AXIS = 1;
const int Y_AXIS = 2;

// Arbitrarily small number used to account for floating point rounding error
const float COLLISION_EPSILON = 0.00001f;
// Arbitrarily small number used to account for floating point rounding error
const float TIMESLICE_EPSILON = 0.00001f;
// Arbitrarily high number used prevent infinite loops in degenerate cases where collisions are unresolvable
const int MAX_TIMESLICE_SUBDIVISIONS = 32;

phys::collisionSystem::collisionSystem()
{

}

phys::collisionSystem::~collisionSystem()
{

}

void phys::collisionSystem::setCollisionInfo(bool t, bool b, bool l, bool r){
    m_collisionInfo.m_collisionTop = t;
	m_collisionInfo.m_collisionBottom = b;
	m_collisionInfo.m_collisionLeft = l;
	m_collisionInfo.m_collisionRight = r;
}

void phys::collisionSystem::setBodyInfo(platformBody& platform){
    m_bodyInfo.m_type = platform.m_type;
	m_bodyInfo.m_id = platform.m_id;
	m_bodyInfo.m_height = platform.m_height;
	m_bodyInfo.m_width = platform.m_width;
	m_bodyInfo.m_collisionBottom = platform.m_collisionBottom;
	m_bodyInfo.m_collisionTop = platform.m_collisionTop;
	m_bodyInfo.m_collisionLeft = platform.m_collisionLeft;
	m_bodyInfo.m_collisionRight = platform.m_collisionRight;
	m_bodyInfo.m_falling = platform.m_falling;
	m_bodyInfo.m_position = platform.m_position;
	m_bodyInfo.m_surfaceVelocity = platform.m_surfaceVelocity;
}

bool phys::collisionSystem::movingToStaticIntersectionTest(class dynamicBody * dynamicBody, class platformBody platformBody, struct collision * outCollision){

	float timeX = 0.f, timeY = 0.f;
	bool m_collisionX = false;
	bool m_collisionY = false;
	float newX = 0.f, newY = 0.f;
	float staticLeft, staticRight, staticBottom, staticTop;
	float movingLeft, movingRight, movingBottom, movingTop;
	float lastMovingLeft, lastMovingRight, lastMovingBottom, lastMovingTop;

	staticLeft = platformBody.m_position.x;
	staticRight = staticLeft + platformBody.m_width;
	staticBottom = platformBody.m_position.y;
	staticTop = staticBottom + platformBody.m_height;
	movingLeft = dynamicBody->m_position.x;
	movingRight = movingLeft + dynamicBody->m_width;
	movingBottom = dynamicBody->m_position.y;
	movingTop = movingBottom + dynamicBody->m_height;
	lastMovingLeft = dynamicBody->m_lastPosition.x;
	lastMovingRight = lastMovingLeft + dynamicBody->m_width;
	lastMovingBottom = dynamicBody->m_lastPosition.y;
	lastMovingTop = lastMovingBottom + dynamicBody->m_height;

	// Incorrect sweeping, but better than nothing
	if (std::min(movingBottom, lastMovingBottom) < staticTop - COLLISION_EPSILON && std::max(movingTop, lastMovingTop) > staticBottom + COLLISION_EPSILON) {
		if (lastMovingRight <= staticLeft + COLLISION_EPSILON && movingRight > staticLeft + COLLISION_EPSILON) {
			m_collisionX = true;
			m_collisionRight = true;
			m_collisionLeft = false;
			platformBody.m_collisionRight = true;
			platformBody.m_collisionLeft = false;
			timeX = (staticLeft - lastMovingRight) / (movingLeft - lastMovingLeft);
		}
		else if (lastMovingLeft >= staticRight - COLLISION_EPSILON && movingLeft < staticRight - COLLISION_EPSILON) {
			m_collisionX = true;
			m_collisionLeft = true;
			m_collisionRight = false;
			platformBody.m_collisionLeft = true;
			platformBody.m_collisionRight = false;
			timeX = (staticRight - lastMovingLeft) / (movingLeft - lastMovingLeft);
		}

		// Compensate somewhat for incorrect sweeping by discarding collisions that don't intersect at collision time
		// Without this, collisions with cracks where you're moving perpendicular to the wall faster than parallel to it will be registered
		if (m_collisionX) {
			newY = lastMovingBottom + (movingBottom - lastMovingBottom) * timeX;
			if (newY >= staticTop - COLLISION_EPSILON || newY + platformBody.m_height <= staticBottom + COLLISION_EPSILON) {
				m_collisionX = false;
				m_collisionLeft = false;
				m_collisionRight = false;
				platformBody.m_collisionLeft = false;
				platformBody.m_collisionRight = false;
			}
		}
	}else{
            m_collisionLeft = false;
            m_collisionRight = false;
            platformBody.m_collisionLeft = false;
            platformBody.m_collisionRight = false;
    }

	if (std::min(movingLeft, lastMovingLeft) < staticRight - COLLISION_EPSILON && std::max(movingRight, lastMovingRight) > staticLeft + COLLISION_EPSILON) {
		if (lastMovingTop <= staticBottom + COLLISION_EPSILON && movingTop > staticBottom + COLLISION_EPSILON) {
			m_collisionY = true;
			m_collisionBottom = true;
			m_collisionTop = false;
			platformBody.m_collisionBottom = true;
			platformBody.m_collisionTop = false;
			timeY = (staticBottom - lastMovingTop) / (movingBottom - lastMovingBottom);
		}
		else if (lastMovingBottom >= staticTop - COLLISION_EPSILON && movingBottom < staticTop - COLLISION_EPSILON) {
			m_collisionTop = true;
			platformBody.m_collisionTop = true;
			m_collisionBottom = false;
			platformBody.m_collisionBottom = false;

			if(platformBody.m_type == phys::bodyType::jumpthrough)
                m_collisionY = false;
            else
                m_collisionY = true;

			timeY = (staticTop - lastMovingBottom) / (movingBottom - lastMovingBottom);
		}

		if (m_collisionX) {
			newX = lastMovingLeft + (movingLeft - lastMovingLeft) * timeY;
			if (newX >= staticRight - COLLISION_EPSILON || newY + platformBody.m_width <= staticLeft + COLLISION_EPSILON) {
                m_collisionY = false;
                m_collisionTop = false;
				m_collisionBottom = false;
				platformBody.m_collisionTop = false;
				platformBody.m_collisionBottom = false;
			}
		}
	}else{
            m_collisionTop = false;
            m_collisionBottom = false;
            platformBody.m_collisionTop = false;
            platformBody.m_collisionBottom = false;
    }

	if (m_collisionX || m_collisionY) {

		if (!m_collisionX || (m_collisionY && timeY < timeX)) {
			outCollision->time = timeY;
			outCollision->axis = Y_AXIS;
			outCollision->surfaceArea = std::min(staticRight, newX + dynamicBody->m_width) - std::max(staticLeft, newX);
			return true;
		}

		outCollision->time = timeX;
		outCollision->axis = X_AXIS;
		outCollision->surfaceArea = std::min(staticTop, newY + dynamicBody->m_height) - std::max(staticBottom, newY);
        return true;
	}

	return false;
}

void phys::collisionSystem::resolveStaticCollision(class dynamicBody * dynamicBody, struct collision collision, float timesliceRemaining) {

	dynamicBody->m_position.x = dynamicBody->m_lastPosition.x + (dynamicBody->m_position.x - dynamicBody->m_lastPosition.x) * collision.time;
	dynamicBody->m_position.y = dynamicBody->m_lastPosition.y + (dynamicBody->m_position.y - dynamicBody->m_lastPosition.y) * collision.time;

	if (collision.axis == X_AXIS) {
		dynamicBody->m_velocity.x = 0.0f;
	}
	else{
        dynamicBody->m_velocity.y = 0.0f;
	}

	dynamicBody->m_lastPosition = dynamicBody->m_position;

	dynamicBody->m_position.x += dynamicBody->m_velocity.x * (1.0f - collision.time) * timesliceRemaining;
	dynamicBody->m_position.y += dynamicBody->m_velocity.y * (1.0f - collision.time) * timesliceRemaining;
}

void phys::collisionSystem::resolveCollisions(class dynamicBody * dynamicBody, class platformBody * staticBodies, int numberOfPlatformBodies) {
	int platformBodyIndex;
	float timesliceRemaining;
	int numberOfTimesSubdivided;
	struct collision collision, bestCollision;
	bool collided;

	timesliceRemaining = 1.0f;
	numberOfTimesSubdivided = 0;

	while (timesliceRemaining > TIMESLICE_EPSILON) {
		bestCollision.time = 1.0f + TIMESLICE_EPSILON;
		collided = false;

		for (platformBodyIndex = 0; platformBodyIndex < numberOfPlatformBodies; platformBodyIndex++) {
			if (movingToStaticIntersectionTest(dynamicBody, staticBodies[platformBodyIndex], &collision)) {
                    setBodyInfo(staticBodies[platformBodyIndex]);//set platform body info
				if (collision.time < bestCollision.time ||
					(collision.time < bestCollision.time + TIMESLICE_EPSILON &&
					collision.surfaceArea > bestCollision.surfaceArea)) {
					collided = true;
					bestCollision = collision;
					setCollisionInfo(m_collisionTop, m_collisionBottom, m_collisionLeft, m_collisionRight);//set collision info
				}
			}
		}

		if (!collided) {
			break;
		}

		resolveStaticCollision(dynamicBody, bestCollision, timesliceRemaining);

		timesliceRemaining = (1.0f - bestCollision.time) * timesliceRemaining;
		numberOfTimesSubdivided++;

		if (numberOfTimesSubdivided > MAX_TIMESLICE_SUBDIVISIONS) {
			fprintf(stderr, "Warning: std::maximum timeslice subdivisons reached; collision resolution may be incomplete\n");
			break;
		}
	}
}

//initialize static data members outside of the class definition
phys::bodyInfo phys::collisionSystem::m_bodyInfo;
phys::collisionInfo phys::collisionSystem::m_collisionInfo;
bool phys::collisionSystem::m_collisionTop = false;
bool phys::collisionSystem::m_collisionBottom = false;
bool phys::collisionSystem::m_collisionLeft = false;
bool phys::collisionSystem::m_collisionRight = false;
