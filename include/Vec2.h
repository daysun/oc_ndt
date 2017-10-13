#ifndef Vec2_h_
#define Vec2_h_

#include <cmath>

struct Vec2;
Vec2 operator*(float r, const Vec2& v);

struct Vec2 {
	union {
		struct {
            float x,y;
		};
        float D[2];
	};

    Vec2() { }
    Vec2(float _x, float _y)
        :x(_x), y(_y)
	{ }

	float& operator[](unsigned int i) {
		return D[i];
	}

	const float& operator[](unsigned int i) const {
		return D[i];
	}

	float maxComponent() const {
		float r = x;
        if(y>r) r = y;
		return r;
	}

	float minComponent() const {
		float r = x;
        if(y<r) r = y;
		return r;
	}

    Vec2 operator+(const Vec2& r) const {
        return Vec2(x+r.x, y+r.y);
	}

    Vec2 operator-(const Vec2& r) const {
        return Vec2(x-r.x, y-r.y);
	}

    Vec2 cmul(const Vec2& r) const {
        return Vec2(x*r.x, y*r.y);
	}

    Vec2 cdiv(const Vec2& r) const {
        return Vec2(x/r.x, y/r.y);
	}

    Vec2 operator*(float r) const {
        return Vec2(x*r,y*r);
	}


    Vec2 operator/(float r) const {
        return Vec2(x/r, y/r);
	}

    Vec2& operator+=(const Vec2& r) {
		x+=r.x;
        y+=r.y;
		return *this;
	}

    Vec2& operator-=(const Vec2& r) {
		x-=r.x;
        y-=r.y;
		return *this;
	}

    Vec2& operator*=(float r) {
        x*=r; y*=r;
		return *this;
	}

	// Inner/dot product
    float operator*(const Vec2& r) const {
        return x*r.x + y*r.y ;
	}

	float norm() const {
        return sqrtf(x*x+y*y);
	}

	float normSquared() const {
        return x*x + y*y ;
	}

	// Cross product
//    Vec2 operator^(const Vec2& r) const {
//        return Vec2(
//				y * r.z - z * r.y,
//				z * r.x - x * r.z,
//				x * r.y - y * r.x
//				);
//	}

    Vec2 normalized() const {
		return *this / norm();
	}
};

inline Vec2 operator*(float r, const Vec2& v) {
    return Vec2(v.x*r, v.y*r);
}

#endif
