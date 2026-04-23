#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    /////////////////////////////////
    /// TODO: You can add any [private] member variable or [private] member function you need.
    /////////////////////////////////

public:

    Vec get_v_next() {
        Vec v_pref = (pos_tar - pos_cur);
        if (v_pref.norm() > v_max) {
            v_pref = v_pref.normalize() * v_max;
        }

        int n_robots = monitor->get_robot_number();
        auto is_safe = [&](Vec v_test) {
            if (v_test.norm() > v_max + 1e-7) return false;
            for (int i = 0; i < n_robots; ++i) {
                if (i == id) continue;
                Vec other_pos = monitor->get_pos_cur(i);
                Vec other_v = monitor->get_v_cur(i);
                double other_r = monitor->get_r(i);

                Vec delta_pos = pos_cur - other_pos;
                Vec delta_v = v_test - other_v;
                double project = delta_pos.dot(delta_v);
                if (project >= 0) continue;
                
                project /= -delta_v.norm();
                double min_dis_sqr;
                double combined_r = r + other_r;
                if (project < delta_v.norm() * TIME_INTERVAL) {
                    min_dis_sqr = delta_pos.norm_sqr() - project * project;
                } else {
                    min_dis_sqr = (delta_pos + delta_v * TIME_INTERVAL).norm_sqr();
                }
                if (min_dis_sqr <= combined_r * combined_r - 1e-7) {
                    return false;
                }
            }
            return true;
        };

        if (is_safe(v_pref)) return v_pref;

        Vec best_v(0, 0);
        double best_score = -1e18;

        // Sample velocities
        for (int i = 0; i < 16; ++i) {
            double angle = 2.0 * PI * i / 16.0;
            for (double speed = v_max; speed > 0; speed -= v_max / 4.0) {
                Vec v_test(speed * std::cos(angle), speed * std::sin(angle));
                if (is_safe(v_test)) {
                    double score = v_test.dot(v_pref.normalize()) * v_test.norm();
                    if (score > best_score) {
                        best_score = score;
                        best_v = v_test;
                    }
                }
            }
        }

        if (is_safe(Vec(0, 0))) {
            if (0.0 > best_score) {
                best_v = Vec(0, 0);
            }
        }

        return best_v;
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP