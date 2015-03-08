#ifndef DYNAMICBODY_HPP
#define DYNAMICBODY_HPP

////////////////////////////////////////////////////////////
/// \brief This class was created to complement the code
/// initially created by [Alex Diener] in order to
/// increase functionality.
/// [Alex Diener] : (http://ludobloom.com/tutorials/collision.html/ "Collision Detection")
////////////////////////////////////////////////////////////

#include <SFML/System/Vector2.hpp>

namespace phys
{
	class dynamicBody
	{
	public:
		dynamicBody();
		~dynamicBody();

	public:
		int m_moveX;
		int m_moveY;
		sf::Vector2f m_lastPosition;
		sf::Vector2f m_position;
		sf::Vector2f m_velocity;
		float m_width;
		float m_height;
	};
}

#endif //DYNAMICBODY_HPP
