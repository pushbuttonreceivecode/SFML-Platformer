#include <dynamicBody.hpp>

phys::dynamicBody::dynamicBody()
{
	m_moveX = 0;
	m_moveY = 0;
	m_lastPosition = sf::Vector2f(0,0);
	m_position = sf::Vector2f(0, 0);
	m_velocity = sf::Vector2f(0, 0);
	m_width = 32.f;
	m_height = 32.f;
}

phys::dynamicBody::~dynamicBody()
{

}
