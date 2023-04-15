/*
 * zlib License
 * 
 * (C) 2032 XXIV
 * 
 * This software is provided *as-is*, without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
module cutec2;

extern (C):

// this can be adjusted as necessary, but is highly recommended to be kept at 8.
// higher numbers will incur quite a bit of memory overhead, and convex shapes
// over 8 verts start to just look like spheres, which can be implicitly rep-
// resented as a point + radius. usually tools that generate polygons should be
// constructed so they do not output polygons with too many verts.
// Note: polygons in cute_c2 are all *convex*.
enum C2_MAX_POLYGON_VERTS = 8;

// 2d vector
struct c2v
{
    float x;
    float y;
}

// 2d rotation composed of cos/sin pair for a single angle
// We use two floats as a small optimization to avoid computing sin/cos unnecessarily
struct c2r
{
    float c;
    float s;
}

// 2d rotation matrix
struct c2m
{
    c2v x;
    c2v y;
}

// 2d transformation "x"
// These are used especially for c2Poly when a c2Poly is passed to a function.
// Since polygons are prime for "instancing" a c2x transform can be used to
// transform a polygon from local space to world space. In functions that take
// a c2x pointer (like c2PolytoPoly), these pointers can be NULL, which represents
// an identity transformation and assumes the verts inside of c2Poly are already
// in world space.
struct c2x
{
    c2v p;
    c2r r;
}

// 2d halfspace (aka plane, aka line)
struct c2h
{
    c2v n; // normal, normalized
    float d; // distance to origin from plane, or ax + by = d
}

struct c2Circle
{
    c2v p;
    float r;
}

struct c2AABB
{
    c2v min;
    c2v max;
}

// a capsule is defined as a line segment (from a to b) and radius r
struct c2Capsule
{
    c2v a;
    c2v b;
    float r;
}

struct c2Poly
{
    int count;
    c2v[C2_MAX_POLYGON_VERTS] verts;
    c2v[C2_MAX_POLYGON_VERTS] norms;
}

// IMPORTANT:
// Many algorithms in this file are sensitive to the magnitude of the
// ray direction (c2Ray::d). It is highly recommended to normalize the
// ray direction and use t to specify a distance. Please see this link
// for an in-depth explanation: https://github.com/RandyGaul/cute_headers/issues/30
struct c2Ray
{
    c2v p; // position
    c2v d; // direction (normalized)
    float t; // distance along d from position p to find endpoint of ray
}

struct c2Raycast
{
    float t; // time of impact
    c2v n; // normal of surface at impact (unit length)
}

// position of impact p = ray.p + ray.d * raycast.t
extern (D) auto c2Impact(T0, T1)(auto ref T0 ray, auto ref T1 t)
{
    return c2Add(ray.p, c2Mulvs(ray.d, t));
}

// contains all information necessary to resolve a collision, or in other words
// this is the information needed to separate shapes that are colliding. Doing
// the resolution step is *not* included in cute_c2.
struct c2Manifold
{
    int count;
    float[2] depths;
    c2v[2] contact_points;

    // always points from shape A to shape B (first and second shapes passed into
    // any of the c2***to***Manifold functions)
    c2v n;
}

// This define allows exporting/importing of the header to a dynamic library.
// Here's an example.
// #define CUTE_C2_API extern "C" __declspec(dllexport)

// boolean collision detection
// these versions are faster than the manifold versions, but only give a YES/NO result
int c2CircletoCircle (c2Circle A, c2Circle B);
int c2CircletoAABB (c2Circle A, c2AABB B);
int c2CircletoCapsule (c2Circle A, c2Capsule B);
int c2AABBtoAABB (c2AABB A, c2AABB B);
int c2AABBtoCapsule (c2AABB A, c2Capsule B);
int c2CapsuletoCapsule (c2Capsule A, c2Capsule B);
int c2CircletoPoly (c2Circle A, const(c2Poly)* B, const(c2x)* bx);
int c2AABBtoPoly (c2AABB A, const(c2Poly)* B, const(c2x)* bx);
int c2CapsuletoPoly (c2Capsule A, const(c2Poly)* B, const(c2x)* bx);
int c2PolytoPoly (const(c2Poly)* A, const(c2x)* ax, const(c2Poly)* B, const(c2x)* bx);

// ray operations
// output is placed into the c2Raycast struct, which represents the hit location
// of the ray. the out param contains no meaningful information if these funcs
// return 0
int c2RaytoCircle (c2Ray A, c2Circle B, c2Raycast* out_);
int c2RaytoAABB (c2Ray A, c2AABB B, c2Raycast* out_);
int c2RaytoCapsule (c2Ray A, c2Capsule B, c2Raycast* out_);
int c2RaytoPoly (c2Ray A, const(c2Poly)* B, const(c2x)* bx_ptr, c2Raycast* out_);

// manifold generation
// These functions are (generally) slower than the boolean versions, but will compute one
// or two points that represent the plane of contact. This information is usually needed
// to resolve and prevent shapes from colliding. If no collision occured the count member
// of the manifold struct is set to 0.
void c2CircletoCircleManifold (c2Circle A, c2Circle B, c2Manifold* m);
void c2CircletoAABBManifold (c2Circle A, c2AABB B, c2Manifold* m);
void c2CircletoCapsuleManifold (c2Circle A, c2Capsule B, c2Manifold* m);
void c2AABBtoAABBManifold (c2AABB A, c2AABB B, c2Manifold* m);
void c2AABBtoCapsuleManifold (c2AABB A, c2Capsule B, c2Manifold* m);
void c2CapsuletoCapsuleManifold (c2Capsule A, c2Capsule B, c2Manifold* m);
void c2CircletoPolyManifold (c2Circle A, const(c2Poly)* B, const(c2x)* bx, c2Manifold* m);
void c2AABBtoPolyManifold (c2AABB A, const(c2Poly)* B, const(c2x)* bx, c2Manifold* m);
void c2CapsuletoPolyManifold (c2Capsule A, const(c2Poly)* B, const(c2x)* bx, c2Manifold* m);
void c2PolytoPolyManifold (const(c2Poly)* A, const(c2x)* ax, const(c2Poly)* B, const(c2x)* bx, c2Manifold* m);

enum C2_TYPE
{
    CIRCLE = 0,
    AABB = 1,
    CAPSULE = 2,
    POLY = 3
}

// This struct is only for advanced usage of the c2GJK function. See comments inside of the
// c2GJK function for more details.
struct c2GJKCache
{
    float metric;
    int count;
    int[3] iA;
    int[3] iB;
    float div;
}

// This is an advanced function, intended to be used by people who know what they're doing.
//
// Runs the GJK algorithm to find closest points, returns distance between closest points.
// outA and outB can be NULL, in this case only distance is returned. ax_ptr and bx_ptr
// can be NULL, and represent local to world transformations for shapes A and B respectively.
// use_radius will apply radii for capsules and circles (if set to false, spheres are
// treated as points and capsules are treated as line segments i.e. rays). The cache parameter
// should be NULL, as it is only for advanced usage (unless you know what you're doing, then
// go ahead and use it). iterations is an optional parameter.
//
// IMPORTANT NOTE:
// The GJK function is sensitive to large shapes, since it internally will compute signed area
// values. `c2GJK` is called throughout cute c2 in many ways, so try to make sure all of your
// collision shapes are not gigantic. For example, try to keep the volume of all your shapes
// less than 100.0f. If you need large shapes, you should use tiny collision geometry for all
// cute c2 function, and simply render the geometry larger on-screen by scaling it up.
float c2GJK (const(void)* A, C2_TYPE typeA, const(c2x)* ax_ptr, const(void)* B, C2_TYPE typeB, const(c2x)* bx_ptr, c2v* outA, c2v* outB, int use_radius, int* iterations, c2GJKCache* cache);

// Stores results of a time of impact calculation done by `c2TOI`.
struct c2TOIResult
{
    int hit; // 1 if shapes were touching at the TOI, 0 if they never hit.
    float toi; // The time of impact between two shapes.
    c2v n; // Surface normal from shape A to B at the time of impact.
    c2v p; // Point of contact between shapes A and B at time of impact.
    int iterations; // Number of iterations the solver underwent.
}

// This is an advanced function, intended to be used by people who know what they're doing.
//
// Computes the time of impact from shape A and shape B. The velocity of each shape is provided
// by vA and vB respectively. The shapes are *not* allowed to rotate over time. The velocity is
// assumed to represent the change in motion from time 0 to time 1, and so the return value will
// be a number from 0 to 1. To move each shape to the colliding configuration, multiply vA and vB
// each by the return value. ax_ptr and bx_ptr are optional parameters to transforms for each shape,
// and are typically used for polygon shapes to transform from model to world space. Set these to
// NULL to represent identity transforms. iterations is an optional parameter. use_radius
// will apply radii for capsules and circles (if set to false, spheres are treated as points and
// capsules are treated as line segments i.e. rays).
//
// IMPORTANT NOTE:
// The c2TOI function can be used to implement a "swept character controller", but it can be
// difficult to do so. Say we compute a time of impact with `c2TOI` and move the shapes to the
// time of impact, and adjust the velocity by zeroing out the velocity along the surface normal.
// If we then call `c2TOI` again, it will fail since the shapes will be considered to start in
// a colliding configuration. There are many styles of tricks to get around this problem, and
// all of them involve giving the next call to `c2TOI` some breathing room. It is recommended
// to use some variation of the following algorithm:
//
// 1. Call c2TOI.
// 2. Move the shapes to the TOI.
// 3. Slightly inflate the size of one, or both, of the shapes so they will be intersecting.
//    The purpose is to make the shapes numerically intersecting, but not visually intersecting.
//    Another option is to call c2TOI with slightly deflated shapes.
//    See the function `c2Inflate` for some more details.
// 4. Compute the collision manifold between the inflated shapes (for example, use c2PolytoPolyManifold).
// 5. Gently push the shapes apart. This will give the next call to c2TOI some breathing room.
c2TOIResult c2TOI (const(void)* A, C2_TYPE typeA, const(c2x)* ax_ptr, c2v vA, const(void)* B, C2_TYPE typeB, const(c2x)* bx_ptr, c2v vB, int use_radius);

// Inflating a shape.
//
// This is useful to numerically grow or shrink a polytope. For example, when calling
// a time of impact function it can be good to use a slightly smaller shape. Then, once
// both shapes are moved to the time of impact a collision manifold can be made from the
// slightly larger (and now overlapping) shapes.
//
// IMPORTANT NOTE
// Inflating a shape with sharp corners can cause those corners to move dramatically.
// Deflating a shape can avoid this problem, but deflating a very small shape can invert
// the planes and result in something that is no longer convex. Make sure to pick an
// appropriately small skin factor, for example 1.0e-6f.
void c2Inflate (void* shape, C2_TYPE type, float skin_factor);

// Computes 2D convex hull. Will not do anything if less than two verts supplied. If
// more than C2_MAX_POLYGON_VERTS are supplied extras are ignored.
int c2Hull (c2v* verts, int count);
void c2Norms (c2v* verts, c2v* norms, int count);

// runs c2Hull and c2Norms, assumes p->verts and p->count are both set to valid values
void c2MakePoly (c2Poly* p);

// Generic collision detection routines, useful for games that want to use some poly-
// morphism to write more generic-styled code. Internally calls various above functions.
// For AABBs/Circles/Capsules ax and bx are ignored. For polys ax and bx can define
// model to world transformations (for polys only), or be NULL for identity transforms.
int c2Collided (const(void)* A, const(c2x)* ax, C2_TYPE typeA, const(void)* B, const(c2x)* bx, C2_TYPE typeB);
void c2Collide (const(void)* A, const(c2x)* ax, C2_TYPE typeA, const(void)* B, const(c2x)* bx, C2_TYPE typeB, c2Manifold* m);
int c2CastRay (c2Ray A, const(void)* B, const(c2x)* bx, C2_TYPE typeB, c2Raycast* out_);

extern (D) auto c2Min(T0, T1)(auto ref T0 a, auto ref T1 b)
{
    return a < b ? a : b;
}

extern (D) auto c2Max(T0, T1)(auto ref T0 a, auto ref T1 b)
{
    return a > b ? a : b;
}

extern (D) auto c2Abs(T)(auto ref T a)
{
    return a < 0 ? -a : a;
}

extern (D) auto c2Clamp(T0, T1, T2)(auto ref T0 a, auto ref T1 lo, auto ref T2 hi)
{
    return c2Max(lo, c2Min(a, hi));
}

void c2SinCos (float radians, float* s, float* c);

extern (D) int c2Sign(T)(auto ref T a)
{
    return a < 0 ? -1.0f : 1.0f;
}

// The rest of the functions in the header-only portion are all for internal use
// and use the author's personal naming conventions. It is recommended to use one's
// own math library instead of the one embedded here in cute_c2, but for those
// curious or interested in trying it out here's the details:

// The Mul functions are used to perform multiplication. x stands for transform,
// v stands for vector, s stands for scalar, r stands for rotation, h stands for
// halfspace and T stands for transpose.For example c2MulxvT stands for "multiply
// a transform with a vector, and transpose the transform".

// vector ops
c2v c2V (float x, float y);
c2v c2Add (c2v a, c2v b);
c2v c2Sub (c2v a, c2v b);
float c2Dot (c2v a, c2v b);
c2v c2Mulvs (c2v a, float b);
c2v c2Mulvv (c2v a, c2v b);
c2v c2Div (c2v a, float b);
c2v c2Skew (c2v a);
c2v c2CCW90 (c2v a);
float c2Det2 (c2v a, c2v b);
c2v c2Minv (c2v a, c2v b);
c2v c2Maxv (c2v a, c2v b);
c2v c2Clampv (c2v a, c2v lo, c2v hi);
c2v c2Absv (c2v a);
float c2Hmin (c2v a);
float c2Hmax (c2v a);
float c2Len (c2v a);
c2v c2Norm (c2v a);
c2v c2SafeNorm (c2v a);
c2v c2Neg (c2v a);
c2v c2Lerp (c2v a, c2v b, float t);
int c2Parallel (c2v a, c2v b, float kTol);

// rotation ops
c2r c2Rot (float radians);
c2r c2RotIdentity ();
c2v c2RotX (c2r r);
c2v c2RotY (c2r r);
c2v c2Mulrv (c2r a, c2v b);
c2v c2MulrvT (c2r a, c2v b);
c2r c2Mulrr (c2r a, c2r b);
c2r c2MulrrT (c2r a, c2r b);

c2v c2Mulmv (c2m a, c2v b);
c2v c2MulmvT (c2m a, c2v b);
c2m c2Mulmm (c2m a, c2m b);
c2m c2MulmmT (c2m a, c2m b);

// transform ops
c2x c2xIdentity ();
c2v c2Mulxv (c2x a, c2v b);
c2v c2MulxvT (c2x a, c2v b);
c2x c2Mulxx (c2x a, c2x b);
c2x c2MulxxT (c2x a, c2x b);
c2x c2Transform (c2v p, float radians);

// halfspace ops
c2v c2Origin (c2h h);
float c2Dist (c2h h, c2v p);
c2v c2Project (c2h h, c2v p);
c2h c2Mulxh (c2x a, c2h b);
c2h c2MulxhT (c2x a, c2h b);
c2v c2Intersect (c2v a, c2v b, float da, float db);

void c2BBVerts (c2v* out_, c2AABB* bb);
