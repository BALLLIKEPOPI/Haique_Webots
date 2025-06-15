#include "statemachine/HoverMPC.h"
#include <casadi/core/calculus.hpp>

HoverMPC::HoverMPC(){}

SX HoverMPC::Dynamics(const SX state, const SX con, SX drag){
    // State
    SX x = state(0); SX y = state(1); SX z = state(2);
    SX vx = state(3); SX vy = state(4); SX vz = state(5);
    SX phi = state(6); SX theta = state(7); SX psi = state(8);
    SX wx = state(9); SX wy = state(10); SX wz = state(11);
    // Control inputs
    SX f1 = con(0); SX f2 = con(1); SX f3 = con(2); SX f4 = con(3);
    SX f5 = con(4); SX f6 = con(5); SX f7 = con(6); SX f8 = con(7);
    SX alpha = con(8); SX beta = con(9);
    // Drag torques
    SX drag_tx = drag(0); SX drag_ty = drag(1); SX drag_tz = drag(2);
    // Drag forces
    SX drag_fx = drag(3); SX drag_fy = drag(4); SX drag_fz = drag(5);

    SX F1 = f1 + f5;
    SX F2 = f2 + f6;
    SX F3 = f3 + f7;
    SX F4 = f4 + f8;
    SX totalForce = F1 + F2 + F3 + F4;
    
    SX Fx = (sin(psi)*sin(phi) + cos(phi)*cos(psi)*sin(theta))*totalForce;
    SX Fy = (cos(phi)*sin(phi)*sin(theta) - cos(psi)*sin(phi))*totalForce;
    SX Fz = cos(phi)*cos(theta)*totalForce;

    SX wx_ = wx + tan(theta)*(wy*sin(phi) + wz*cos(phi));
    SX wy_ = wy*cos(phi) - wz*sin(phi);
    SX wz_ = (wy*sin(phi) + wz*cos(phi))/cos(theta);

    SX Mx = (F2 - F4)*l;
    SX My = (F3 - F1)*l;
    SX Mz = (f1 - f2 + f3 - f4 - f5 + f6 - f7 + f8)*c/k;

    return SX::vertcat({vx,
                        vy,
                        vz,
                        1/m*Fx - drag_fx,
                        1/m*Fy - drag_fy,
                        1/m*(Fz + (rou*V - m)*G) - drag_fz,
                        wx_,
                        wy_,
                        wz_,
                        (1/I1)*Mx - drag_tx,
                        (1/I2)*My - drag_ty,
                        (1/I3)*Mz - drag_tz});
}

void HoverMPC::setupProblem() {
    // [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    Q_hover(0, 0) = 100; Q_hover(3, 3) = 1;
    Q_hover(1, 1) = 100; Q_hover(4, 4) = 1;
    Q_hover(2, 2) = 100; Q_hover(5, 5) = 1;
    Q_hover(6, 6) = 100; Q_hover(7, 7) = 1;
    Q_hover(1, 8) = 100; Q_hover(4, 8) = 1;
    Q_hover(2, 9) = 100; Q_hover(5, 9) = 1;
    // [f1, f2, f3, f4, f5, f6, f7, f8]
    R_hover(0, 0) = 1000; R_hover(1, 1) = 1000;
    R_hover(2, 2) = 1000; R_hover(3, 3) = 1000;
    R_hover(4, 4) = 1000; R_hover(5, 5) = 1000;
    R_hover(6, 6) = 1000; R_hover(7, 7) = 1000;

    st = X(all, 0);
    drag = P(Slice(2*n_state, 2*n_state+n_drag));
    g(Slice(0, n_state)) = st - P(Slice(0, n_state));
    for(int i = 0; i < N; i++){
        st = X(all, i);
        con = U(all, i);
        obj += SX::mtimes({(st-P(Slice(n_state, n_state*2))).T(), Q_hover, (st-P(Slice(n_state, n_state*2)))}) +
                SX::mtimes({con.T(), R_hover, con});
        st_next = X(all, i+1);
        // RK4
        k1 = Dynamics(st, con, drag);
        k2 = Dynamics(st + h/2*k1, con, drag);
        k3 = Dynamics(st + h/2*k2, con, drag);
        k4 = Dynamics(st + h*k3, con, drag);
        st_next_RK4 = st + h/6*(k1+k2+k3+k4);
        // compute constraints 
        g(Slice((i+1)*n_state, (i+2)*n_state)) = st_next - st_next_RK4;
    }
    // make the decision variable one column vector
    OPT_variables = SX::vertcat({SX::reshape(X, n_state*(N+1), 1), SX::reshape(U, n_control*N, 1)});
    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}, {"p", P}};
    Dict opts = { {"ipopt.max_iter", 99}, {"ipopt.print_level", 1}, {"print_time", 0}, 
                    {"ipopt.acceptable_tol", 1e-3}, {"ipopt.acceptable_obj_change_tol", 1e-3}};
    solver = nlpsol("solver", "ipopt", nlp, opts);

    // Initial guess and bounds for the optimization variables
    // [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    // x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -85.0, 0.0, 0.0, 0.0}; // initial state
    // xs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -85.0, 0.0, 0.0, 0.0}; // desire state
    x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initial state
    xs = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // desire state
    X0 = SX::repmat(x0, 1, N+1);

    state_lower_bound = {-inf, -inf, -inf, -inf, -inf, -inf,
                        -pi/2, -pi/2, -pi/2, -inf, -inf, -inf};
    state_upper_bound = {inf, inf, inf, inf, inf, inf,
                        pi/2, pi/2, pi/2, inf, inf, inf};

    con_lower_bound = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    con_upper_bound = {50, 50, 50, 50, 50, 50, 50, 50, 0, 0};

    for(int i = 0; i < N+1; i++){
        lbx.insert(lbx.end(), state_lower_bound.begin(), state_lower_bound.end());
        ubx.insert(ubx.end(), state_upper_bound.begin(), state_upper_bound.end());
    }
    for(int i = 0; i < N; i++){
        lbx.insert(lbx.end(), con_lower_bound.begin(), con_lower_bound.end());
        ubx.insert(ubx.end(), con_upper_bound.begin(), con_upper_bound.end());
    }
    lbg.insert(lbg.begin(), n_state*(N+1), 0);
    ubg.insert(ubg.begin(), n_state*(N+1), 0);

    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;

    // TODO: if the system is in initial state
    // or maybe we should cancel this
    updatePara({0, 0, 0}, {0, 0, 0}); 
}

vector<double> HoverMPC::solve() {
    arg["p"] = para;

    lbx.clear();
    ubx.clear();

    for(int i = 0; i < N+1; i++){
        lbx.insert(lbx.end(), state_lower_bound.begin(), state_lower_bound.end());
        ubx.insert(ubx.end(), state_upper_bound.begin(), state_upper_bound.end());
    }
    for(int i = 0; i < N; i++){
        lbx.insert(lbx.end(), con_lower_bound.begin(), con_lower_bound.end());
        ubx.insert(ubx.end(), con_upper_bound.begin(), con_upper_bound.end());
    }
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    // initial value of the optimization variables
    arg["x0"] = SX::vertcat({SX::reshape(X0.T(), n_state*(N+1), 1),
                                SX::reshape(u0.T(), n_control*N, 1)});
    res = solver(arg);
    return getFirstCon();
}

vector<double> HoverMPC::getFirstCon(){
    u_f.clear();
    vector<double> u_f_ = res.at("x").get_elements();
    u_f.insert(u_f.begin(), u_f_.begin()+n_state*(N+1), u_f_.begin()+n_control+n_state*(N+1));
    return u_f;
}