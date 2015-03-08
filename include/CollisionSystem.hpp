/**
*	\brief Copyright (c) 2011 Alex Diener
*
*	This software is provided 'as-is', without any express or implied
*	warranty. In no event will the authors be held liable for any damages
*	arising from the use of this software.
*
*	Permission is granted to anyone to use this software for any purpose,
*	including commercial applications, and to alter it and redistribute it
*	freely, subject to the following restrictions:
*
*	1. The origin of this software must not be misrepresented; you must not
*	claim that you wrote the original software. If you use this software
*	in a product, an acknowledgment in the product documentation would be
*	appreciated but is not required.
*	2. Altered source versions must be plainly marked as such, and must not be
*	misrepresented as being the original software.
*	3. This notice may not be removed or altered from any source distribution.
*
*	Alex Diener adiener@sacredsoftware.net
*/

#ifndef COLLISION_SYSTEM_HPP
#define COLLISION_SYSTEM_HPP

////////////////////////////////////////////////////////////
/// \brief This class is based on the code
/// initially created by [Alex Diener] in order to
/// increase functionality and provide an object-oriented interface.
/// This includes the addition of different types of bodies
/// in order to create a more robust platformer collision system.
/// It also allows for the implementation of different types of platforms
/// such as conveyor belts, breakable, falling, and moving.
/// This is accomplished by sharing more information about
/// collisions and platforms with the main program.
/// [Alex Diener] : (http://ludobloom.com/tutorials/collision.html/ "Collision Detection")
////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <algorithm>
#include <stdbool.h>
#include <stdio.h>
#include <dynamicBody.hpp>
#include <platformBody.hpp>
#include <iostream>

namespace phys
{
	struct collision{
		float time = 0.f;
		float surfaceArea = 0.f;
		int axis = 0;
	};

    struct collisionInfo{
        bool m_collisionTop = false;
	    bool m_collisionBottom = false;
	    bool m_collisionLeft = false;
	    bool m_collisionRight = false;
    };

    enum bodyType{
        none = 0,
        platform = 1,
        conveyorBelt = 2,
        moving = 3,
        jumpthrough = 4,
        falling = 5,
        vanishing = 6
    };

    struct bodyInfo{
        unsigned int m_type = phys::bodyType::none;
	    unsigned int m_id = 0;
	    float m_width = 0.f;
	    float m_height = 0.f;
	    bool m_collisionTop = false;
	    bool m_collisionBottom = false;
	    bool m_collisionLeft = false;
	    bool m_collisionRight = false;
	    bool m_falling = false;
	    sf::Vector2f m_position = sf::Vector2f(0,0);
        float m_surfaceVelocity = 0.f;
    };

	class collisionSystem
	{
	public:
		collisionSystem();
		~collisionSystem();

		void resolveCollisions(class dynamicBody * dynamicBody, class platformBody * platformBodies, int numberOfPlatformObjects);
        void setBodyInfo(platformBody& platform);
        bodyInfo getBodyInfo(){return m_bodyInfo;};

        void setCollisionInfo(bool t, bool b, bool l, bool r);
        collisionInfo getCollisionInfo(){return m_collisionInfo;};

	private:
		static bool movingToStaticIntersectionTest(class dynamicBody * dynamicBody, class platformBody platformBody, struct collision * outCollision);
		static void resolveStaticCollision(class dynamicBody * dynamicBody, struct collision collision, float timesliceRemaining);

	public:
        static bodyInfo m_bodyInfo;
        static collisionInfo m_collisionInfo;
        static bool m_collisionTop;
        static bool m_collisionBottom;
        static bool m_collisionLeft;
        static bool m_collisionRight;
	};
}

#endif //COLLISION_SYSTEM_HPP
