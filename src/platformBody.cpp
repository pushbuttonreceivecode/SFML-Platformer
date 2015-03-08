#include <platformBody.hpp>

phys::platformBody::platformBody()
{
	m_position = sf::Vector2f(0,0);
	m_width = 32;
	m_height = 32;
	m_type = 1;//initialize to a static body
	m_collisionTop = false;
	m_collisionBottom = false;
	m_collisionLeft = false;
    m_collisionRight = false;
	m_falling = false;//this is only used for falling platforms
	m_surfaceVelocity = 0;//this is only used for conveyor belts and friction on walls etc.
}

phys::platformBody::~platformBody()
{

}
