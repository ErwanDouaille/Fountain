#ifndef __vecteur3d_h__
#define __vecteur3d_h__

#include <iostream>
#include <math.h>

#ifdef _DEBUG

#include <stdlib.h>
#define TEST_ERROR(cond,error_msg) {if(!(cond)){ std::cerr << (error_msg) << std::endl << " in " << __FILE__ << std::endl << " line " << __LINE__ << std::endl; exit(-1); }}
#define TEST_WARNING(cond,warning_msg) {if(!(cond)) std::cerr << (warning_msg) << std::endl << " in " << __FILE__ << std::endl << " line " << __LINE__ << std::endl; }

#else

#define TEST_ERROR(cond,error_msg)
#define TEST_WARNING(cond,warning_msg)

#endif



class Vecteur3D
{
	friend std::ostream &operator <<(std::ostream &flux,Vecteur3D &v);
public:
  float x,y,z;

  Vecteur3D(){x=y=z=0; };
  Vecteur3D(float _x,float _y,float _z):x(_x),y(_y),z(_z){};
	Vecteur3D(float _x,float _y):x(_x),y(_y),z(0){};
  
  inline void nul(void){x=y=z=0.0;};

  inline float &operator [](int i)
  {
    TEST_ERROR(i>=0 && i<3,"Vecteur3D[] : 3 components vector !!");
    return *(&x+i);
  };

  // Addition
  inline Vecteur3D operator +(const Vecteur3D &v) const
  { return Vecteur3D(x+v.x,y+v.y,z+v.z); };
  inline Vecteur3D &operator +=(const Vecteur3D &v)
  { x+=v.x; y+=v.y; z+=v.z; return *this; };

  // Soustraction
  inline Vecteur3D operator -(const Vecteur3D &v) const
  { return Vecteur3D(x-v.x,y-v.y,z-v.z); };
  inline Vecteur3D &operator -=(const Vecteur3D &v)
  { x-=v.x; y-=v.y; z-=v.z; return *this; };
  inline Vecteur3D operator -(void) const
  { return Vecteur3D(-x,-y,-z); };

  // Multiplication par un scalaire
  inline Vecteur3D &operator *=(float f)
  { x*=f; y*=f; z*=f; return *this; };
  inline Vecteur3D operator *(float f) const
  { return Vecteur3D(x*f,y*f,z*f); };

  //ÊýÁ¿³ý·¨
  inline Vecteur3D operator /(float f) const
  { return Vecteur3D(x/f,y/f,z/f);}

  // Norme et Carre de la norme
  inline float norme(void) const
  { return (float)sqrt(x*x+y*y+z*z); };
  inline float carreNorme(void) const
  { return (x*x+y*y+z*z); };
	inline Vecteur3D normalise(void)
	{ float t=(float)1.0/(float)sqrt((double)(x*x+y*y+z*z));
		x*=t; y*=t; z*=t;
		return Vecteur3D(x,y,z);
	};

  // Produit Scalaire
  inline float operator *(const Vecteur3D &a) const
  { return (x*a.x+y*a.y+z*a.z); };
  
  // Produit Vectorielle
  // Attention au priorite des operateurs !!
  inline Vecteur3D operator ^(const Vecteur3D &a) const
  { return Vecteur3D( y*a.z-z*a.y , z*a.x-x*a.z , x*a.y-y*a.x ); };
  
	inline bool operator ==(const Vecteur3D &a) const
	{ return ((x == a.x) && (y == a.y) && (z == a.z));};

	inline Vecteur3D operator =(const float a) const
	{ return Vecteur3D(a, a, a);};

	inline bool operator !=(const Vecteur3D &a) const
	{ return ((x != a.x) || (y != a.y) || (z != a.z));};

  inline Vecteur3D &operator ^=(const Vecteur3D &a)
  {
    float tempx=x,tempy=y;
    x=y*a.z-z*a.y; y=z*a.x-tempx*a.z; z=tempx*a.y-tempy*a.x;
    return *this;
  };
};

std::ostream &operator <<(std::ostream &flux,Vecteur3D &v);

#endif
