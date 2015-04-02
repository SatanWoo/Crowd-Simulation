#ifndef _WZKERNELFUNCTION_H
#define _WZKERNELFUNCTION_H

#include "Vector.h"

#define PI 3.14159265359

class MathHelper {
public:
	MathHelper(double radius);
	~MathHelper();

	double poly6(const Vector2D& vec);
	Vector2D spikyGrad(const Vector2D& vec);
	double vislap(const Vector2D& vec);

private:
	double m_dRadius;
	double m_dRadius2;
	double m_dRadius6;
	double m_dRadius9;

	double m_dPoly6;
	double m_dSpikyGrad;
	double m_dVisLap;
};


#endif