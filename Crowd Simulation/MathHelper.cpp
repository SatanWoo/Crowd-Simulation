#include "MathHelper.h"
#include <math.h>

MathHelper::MathHelper(double radius)
{
	m_dRadius = radius;
	m_dRadius2 = radius * radius;
	m_dRadius6 = m_dRadius2 * m_dRadius2 * m_dRadius2;
	m_dRadius9 = m_dRadius6 * m_dRadius2 * m_dRadius;

	m_dPoly6 = 315.0 / (64.0 * PI * m_dRadius9);
	m_dSpikyGrad = -45.0 / (PI * m_dRadius6);
	m_dVisLap = -m_dSpikyGrad;
}

MathHelper::~MathHelper()
{}

double MathHelper::poly6(const Vector2D& vec)
{
	double t = m_dRadius2 - vec.squaredLength();
	return t <= 0.0 ? 0.0 : m_dPoly6 * t * t * t;
}

Vector2D MathHelper::spikyGrad(const Vector2D& vec)
{
	double l = vec.length() > 1e-8 ? vec.length() : 1e-8;
	double t = m_dRadius - l;

	if (t <= 0.0) return Vector2D::vec2Zero;
    return vec * t * t * m_dSpikyGrad / l;
}

double MathHelper::vislap(const Vector2D& vec)
{
	double t = m_dRadius - vec.length();
	return t <= 0.0 ? 0.0 : m_dVisLap * t;
}