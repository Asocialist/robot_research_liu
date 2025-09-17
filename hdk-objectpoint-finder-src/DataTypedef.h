#ifndef DEF_DATATYPEDEF_H
#define DEF_DATATYPEDEF_H

#define DEF_PI  3.14159265358979
#define DEF_2PI 6.28318530717958


/*---Radian angles---*/
typedef struct Angle_t {
	Angle_t() { angle = 0; };
	Angle_t(double a) { this->angle = normalize(a); };

	Angle_t operator+ (const Angle_t angle) { return Angle_t(this->angle + angle.angle); }
	Angle_t operator- (const Angle_t angle) { return Angle_t(this->angle - angle.angle); }
	Angle_t operator* (const Angle_t angle) { return Angle_t(this->angle * angle.angle); }
	Angle_t operator/ (const Angle_t angle) { return Angle_t(this->angle / angle.angle); }
	Angle_t operator= (const Angle_t angle) { return Angle_t(this->angle = angle.angle); }

	Angle_t operator+=(const Angle_t angle) { return Angle_t(this->angle = normalize(this->angle + angle.angle)); }
	Angle_t operator-=(const Angle_t angle) { return Angle_t(this->angle = normalize(this->angle - angle.angle)); }
	Angle_t operator*=(const Angle_t angle) { return Angle_t(this->angle = normalize(this->angle * angle.angle)); }
	Angle_t operator/=(const Angle_t angle) { return Angle_t(this->angle = normalize(this->angle / angle.angle)); }

	Angle_t operator+ (const double angle) { return Angle_t(this->angle + angle); }
	Angle_t operator- (const double angle) { return Angle_t(this->angle - angle); }
	Angle_t operator* (const double angle) { return Angle_t(this->angle * angle); }
	Angle_t operator/ (const double angle) { return Angle_t(this->angle / angle); }
	Angle_t operator= (const double angle) { return Angle_t(this->angle = angle); }

	Angle_t operator+=(const double angle) { return Angle_t(this->angle = normalize(this->angle + angle)); }
	Angle_t operator-=(const double angle) { return Angle_t(this->angle = normalize(this->angle - angle)); }
	Angle_t operator*=(const double angle) { return Angle_t(this->angle = normalize(this->angle * angle)); }
	Angle_t operator/=(const double angle) { return Angle_t(this->angle = normalize(this->angle / angle)); }


	bool operator==(Angle_t &angle) { return (this->angle == angle.angle); }
	operator double() { return (angle); }

	double normalize(double a) {
		if (a <= -DEF_PI || a >DEF_PI) {
			a = a - (DEF_2PI) * static_cast<int>(a / (DEF_2PI));

			if (a <= -DEF_PI)
				return a + DEF_2PI;
			if (a > DEF_PI)
				return a - DEF_2PI;

		}
		return a;
	}

	double angle;
}Angle_t;

/*---Three Vector---*/
template <typename _T>
class Vec3_t {
public:
	Vec3_t() { x = 0; y = 0; z = 0; };
	Vec3_t(_T x, _T y, _T z) { this->x = x; this->y = y; this->z = z; };

	Vec3_t operator+ (const Vec3_t vec_3) { return Vec3_t(this->x + vec_3.x, this->y + vec_3.y, this->z + vec_3.z); }
	Vec3_t operator+=(const Vec3_t vec_3) { return Vec3_t(this->x += vec_3.x, this->y += vec_3.y, this->z += vec_3.z); }
	Vec3_t operator- (const Vec3_t vec_3) { return Vec3_t(this->x - vec_3.x, this->y - vec_3.y, this->z - vec_3.z); }
	Vec3_t operator-=(const Vec3_t vec_3) { return Vec3_t(this->x -= vec_3.x, this->y -= vec_3.y, this->z -= vec_3.z); }

	Vec3_t operator* (const Vec3_t vec_3) { return Vec3_t(this->x * vec_3.x, this->y * vec_3.y, this->z * vec_3.z); }
	Vec3_t operator*=(const Vec3_t vec_3) { return Vec3_t(this->x *= vec_3.x, this->y *= vec_3.y, this->z *= vec_3.z); }

	Vec3_t operator* (const double value) { return Vec3_t(this->x *  value, this->y *  value, this->z *  value); }
	Vec3_t operator*=(const double value) { return Vec3_t(this->x *= value, this->y *= value, this->z *= value); }
	Vec3_t operator/ (const double value) { return Vec3_t(this->x / value, this->y / value, this->z / value); }
	Vec3_t operator/=(const double value) { return Vec3_t(this->x /= value, this->y /= value, this->z /= value); }
	bool   operator==(Vec3_t &vec_3) { return (this->x == vec_3.x && this->y == vec_3.y && this->z == vec_3.z); }
	//operator _T() { return length(); }

	_T length()       { return sqrt(this->x * this->x + this->y * this->y + this->z * this->z); }
	_T lengthSquare() { return     (this->x * this->x + this->y * this->y + this->z * this->z); }

	_T x;
	_T y;
	_T z;

};

/*---Two Vector---*/
template <typename _T>
class Vec2_t {
public:
	Vec2_t() { x = 0; y = 0; };
	Vec2_t(_T x, _T y) { this->x = x; this->y = y; };

	Vec2_t operator+ (const Vec2_t vec_2) { return Vec2_t(this->x + vec_2.x, this->y + vec_2.y); }
	Vec2_t operator+=(const Vec2_t vec_2) { return Vec2_t(this->x += vec_2.x, this->y += vec_2.y); }
	Vec2_t operator- (const Vec2_t vec_2) { return Vec2_t(this->x - vec_2.x, this->y - vec_2.y); }
	Vec2_t operator-=(const Vec2_t vec_2) { return Vec2_t(this->x -= vec_2.x, this->y -= vec_2.y); }

	Vec2_t operator* (const Vec2_t vec_2) { return Vec2_t(this->x * vec_2.x, this->y * vec_2.y); }
	Vec2_t operator*=(const Vec2_t vec_2) { return Vec2_t(this->x *= vec_2.x, this->y *= vec_2.y); }
	Vec2_t operator/ (const Vec2_t vec_2) { return Vec2_t(this->x / vec_2.x, this->y / vec_2.y); }
	Vec2_t operator/=(const Vec2_t vec_2) { return Vec2_t(this->x /= vec_2.x, this->y /= vec_2.y); }

	Vec2_t operator* (const double value) { return Vec2_t(this->x *  value, this->y *  value); }
	Vec2_t operator*=(const double value) { return Vec2_t(this->x *= value, this->y *= value); }
	Vec2_t operator/ (const double value) { return Vec2_t(this->x / value, this->y / value); }
	Vec2_t operator/=(const double value) { return Vec2_t(this->x /= value, this->y /= value); }

	friend Vec2_t operator* (const double value, Vec2_t vec_2) { return Vec2_t(vec_2.x *  value, vec_2.y *  value); }

	bool operator==(Vec2_t &vec_2) { return (this->x == vec_2.x && this->y == vec_2.y); }
	operator double() { return (this->x * this->x + this->y * this->y); }

	_T length() { return (this->x * this->x + this->y * this->y); }

	_T x;
	_T y;

};



typedef Vec3_t<double>  Vec3d_t;
typedef Vec3_t<Angle_t> Vec3A_t;
typedef Vec2_t<double> Vec2d_t;
typedef Vec2_t<int> Vec2i_t;

typedef struct {

	Vec3d_t Pos;
	Vec3d_t Rotate;
	double Scale = 1.0; // �g���

}Transform_t;




#endif