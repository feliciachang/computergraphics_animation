/*
skeleton.h

Definition of the skeleton. 

Written by Jehee Lee

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/

#ifndef _BONEVECTOR_H
#define _BONEVECTOR_H


class bonevector
{
  // negation
  friend bonevector    operator-( bonevector const& );

  // addtion
  friend bonevector    operator+( bonevector const&, bonevector const& );

  // subtraction
  friend bonevector    operator-( bonevector const&, bonevector const& );

  // dot product
  friend double    operator%( bonevector const&, bonevector const& );

  // cross product
  friend bonevector    operator*( bonevector const&, bonevector const& );

  // scalar Multiplication
  friend bonevector    operator*( bonevector const&, double );

  // scalar Division
  friend bonevector    operator/( bonevector const&, double );


  friend double    len( bonevector const& );
  friend bonevector	normalize( bonevector const& );

  friend double       angle( bonevector const&, bonevector const& );

  // member functions
public:
  // constructors
  bonevector() {}
  bonevector( double x, double y, double z ) { p[0]=x; p[1]=y; p[2]=z; }
  bonevector( double a[3] ) { p[0]=a[0]; p[1]=a[1]; p[2]=a[2]; }
  ~bonevector() {};

  // inquiry functions
  double& operator[](int i) { return p[i];}

  double x() const { return p[0]; };
  double y() const { return p[1]; };
  double z() const { return p[2]; };
  void   getValue( double d[3] ) { d[0]=p[0]; d[1]=p[1]; d[2]=p[2]; }
  void   setValue( double d[3] ) { p[0]=d[0]; p[1]=d[1]; p[2]=d[2]; }

  double getValue( int n ) const { return p[n]; }
  bonevector setValue( double x, double y, double z )
  { p[0]=x, p[1]=y, p[2]=z; return *this; }
  double setValue( int n, double x )
  { return p[n]=x; }

  double length() const;

  // change functions
  void set_x( double x ) { p[0]=x; };
  void set_y( double x ) { p[1]=x; };
  void set_z( double x ) { p[2]=x; };

  //data members
  double p[3]; //X, Y, Z components of the bonevector
};

#endif

