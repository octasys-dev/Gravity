#pragma GCC diagnostic ignored "-Wcast-function-type"
// ganalyse.cpp
// Pure Data external for analyzing positional data from [g]
// Mode-based output: distance, angle, etc.

#include "m_pd.h"
#include <vector>
#include <string>
#include <cmath>
#include <cstring>

static t_class *gravf_class;
struct t_gravf;

// Function to execute
typedef void (*Calculations)(t_gravf *x, int argc, t_atom *argv);

struct t_gravf
{
    t_object x_obj;
    Calculations func;
    t_outlet *x_out;
    t_inlet *x_in_data; // Inlet for optional data

    float data1; // data componenents from right inlet

    bool tmpb1;  // temporary state value
};

// Helper: calculate distance
float distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

// Helper: calculate angle (in radians)
float angle(float x1, float y1, float x2, float y2)
{
    return std::atan2(y2 - y1, x2 - x1) * (180.0f / M_PI);
}

void calcdefault(t_gravf *x, [[maybe_unused]]int argc, [[maybe_unused]]t_atom *argv)
{
    outlet_float(x->x_out, 0.0f);
}

// Calulates the distance between two bodies or x/y locations
void calcDistance(t_gravf *x, int argc, t_atom *argv)
{
    if (argc < 4)
    {
        pd_error(x, "ganalyse: invalid parameters for distance calculation: expecting [body1.x, body1.y, body2.x, body2.y(");
        return;
    }

    const float x1 = atom_getfloat(argv);
    const float y1 = atom_getfloat(argv + 1);
    const float x2 = atom_getfloat(argv + 2);
    const float y2 = atom_getfloat(argv + 3);

    float d = distance(x1, y1, x2, y2);
    outlet_float(x->x_out, d);
}

// Calulates the angle between two bodies
void calcAngle(t_gravf *x, int argc, t_atom *argv)
{
    if (argc < 4)
    {
        pd_error(x, "ganalyse: invalid parameters for angle calculation: expecting [body1.x, body1.y, body2.x, body2.y(");
        return;
    }

    const float x1 = atom_getfloat(argv);
    const float y1 = atom_getfloat(argv + 1);
    const float x2 = atom_getfloat(argv + 2);
    const float y2 = atom_getfloat(argv + 3);

    float a = angle(x1, y1, x2, y2);
    outlet_float(x->x_out, a);
}

// Calulates the relative velocity between two bodies
void calcRelativeV(t_gravf *x, int argc, t_atom *argv)
{
    if (argc < 4)
    {
        pd_error(x, "ganalyse: invalid parameters for angle calculation: expecting [body1.x, body1.y, body2.x, body2.y(");
        return;
    }

    const float x1 = atom_getfloat(argv);
    const float y1 = atom_getfloat(argv + 1);
    const float x2 = atom_getfloat(argv + 2);
    const float y2 = atom_getfloat(argv + 3);

    float dvx = x2 - x1;
    float dvy = y2 - y1;
    float speed = std::sqrt(dvx * dvx + dvy * dvy);
    outlet_float(x->x_out, speed);
}

// Calulates the approach rate between two bodies
void calcApproach(t_gravf *x, int argc, t_atom *argv)
{
    if (argc < 8)
    {
        pd_error(x, "ganalyse: invalid parameters for approach rate calculation: expecting [body1.x, body1.y, body1.vx, body1.vy, body2.x, body2.y, body2.vx, body2.vy(");
        return;
    }

    const float x1 = atom_getfloat(argv);
    const float y1 = atom_getfloat(argv + 1);
    const float x2 = atom_getfloat(argv + 2);
    const float y2 = atom_getfloat(argv + 3);
    const float vx1 = atom_getfloat(argv + 4);
    const float vy1 = atom_getfloat(argv + 5);
    const float vx2 = atom_getfloat(argv + 6);
    const float vy2 = atom_getfloat(argv + 7);

    float dx = x2 - x1;
    float dy = y2 - y1;
    float dvx = vx2 - vx1;
    float dvy = vy2 - vy1;
    float dist = sqrt(dx * dx + dy * dy);
    float rate = (dist != 0) ? (dvx * dx + dvy * dy) / dist : 0.0f;
    outlet_float(x->x_out, rate);
}

// Calculates the center between two bodies
void calcCenter(t_gravf *x, int argc, t_atom *argv)
{
    if (argc < 4)
    {
        pd_error(x, "ganalyse: invalid parameters for center calculation: expecting [body1.x, body1.y, body2.x, body2.y(");
        return;
    }

    const float x1 = atom_getfloat(argv);
    const float y1 = atom_getfloat(argv + 1);
    const float x2 = atom_getfloat(argv + 2);
    const float y2 = atom_getfloat(argv + 3);

    t_atom out[2];
    SETFLOAT(&out[0], (x1 + x2) / 2.0f);
    SETFLOAT(&out[1], (y1 + y2) / 2.0f);
    outlet_list(x->x_out, &s_list, 2, out);
}

// Sends a bang when if a distance value has been exceeded
void calcInZone(t_gravf *x, int argc, t_atom *argv)
{
    if (argc < 4)
    {
        pd_error(x, "ganalyse: invalid parameters for zone calculation: expecting [body1.x, body1.y, body2.x, body2.y(");
        return;
    }

    const float x1 = atom_getfloat(argv);
    const float y1 = atom_getfloat(argv + 1);
    const float x2 = atom_getfloat(argv + 2);
    const float y2 = atom_getfloat(argv + 3);

    float d = distance(x1, y1, x2, y2);

    // data1 = defined distance
    // tmp1b = current in/out status
    if (d <= x->data1 && !x->tmpb1)
    {
        x->tmpb1 = true;
        outlet_float(x->x_out, 1.0f);
    }
    else if (d > x->data1 && x->tmpb1)
    {
        x->tmpb1 = false;
        outlet_float(x->x_out, 0.0f);
    }
}

// Core list processing
void gravf_list(t_gravf *x, t_symbol *, int argc, t_atom *argv)
{
    x->func(x, argc, argv);
}

// Object creation
void *gravf_new([[maybe_unused]] t_symbol *s, int argc, t_atom *argv)
{
    t_gravf *x = (t_gravf *)pd_new(gravf_class);

    t_symbol *mode;

    if (argc >= 1 && argv[0].a_type == A_SYMBOL)
        mode = argv[0].a_w.w_symbol;
    else
    {
        pd_error(x, "ganalyse: invalid or missing method name: expecting distance, angle, relativev, approach, center, inzone");
        return (void *)x;
    }

    if (mode == gensym("distance"))
    {
        x->func = calcDistance;
    }
    else if (mode == gensym("angle"))
    {
        x->func = calcAngle;
    }
    else if (mode == gensym("relativev"))
    {
        x->func = calcRelativeV;
    }
    else if (mode == gensym("approach"))
    {
        x->func = calcApproach;
    }
    else if (mode == gensym("center"))
    {
        x->func = calcCenter;
    }
    else if (mode == gensym("inzone"))
    {
        x->func = calcInZone;
        x->tmpb1 = false;
    }
    else
    {
        x->func = calcdefault;
        pd_error(x, "ganalyse: unknown analysis function '%s': expecting distance, angle, relativev, approach, center, inzone", mode->s_name);
    }

    x->x_out = outlet_new(&x->x_obj, &s_list);
    x->x_in_data = floatinlet_new(&x->x_obj, &x->data1); //receives values from inlet 2

    return (void *)x;
}

extern "C"
{
    void gravf_setup(void)
    {
        gravf_class = class_new(
            gensym("gravf"),
            (t_newmethod)gravf_new,
            0,
            sizeof(t_gravf),
            CLASS_DEFAULT,
            A_GIMME, 0);

        class_addlist(gravf_class, (t_method)gravf_list);
    }
}
