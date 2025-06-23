#include "../include/attitudectl/attctl.h"
#include "ros/publisher.h"
#include <asm-generic/errno.h>
#include <casadi/core/calculus.hpp>
#include <casadi/core/function.hpp>
#include <casadi/core/generic_matrix.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <cmath>
#include <vector>

MPC_CTL::MPC_CTL(){
    // Q(0, 0) = 10000; Q(3, 3) = 1000;
    // Q(1, 1) = 10000; Q(4, 4) = 1;
    // Q(2, 2) = 1; Q(5, 5) = 1;

    // R(0, 0) = 100000; R(1, 1) = 10000;
    // R(2, 2) = 100000; R(3, 3) = 10000;
    // R(4, 4) = 100000; R(5, 5) = 10000;
    // R(6, 6) = 100000; R(7, 7) = 10000;
    // R(8, 8) = 100; R(9, 9) = 100;

    Q(0, 0) = 100; Q(3, 3) = 1;
    Q(1, 1) = 100; Q(4, 4) = 1;
    Q(2, 2) = 100; Q(5, 5) = 1;

    R(0, 0) = 1000; R(1, 1) = 1000;
    R(2, 2) = 1000; R(3, 3) = 1000;
    R(4, 4) = 1000; R(5, 5) = 1000;
    R(6, 6) = 1000; R(7, 7) = 1000;
    R(8, 8) = 1; R(9, 9) = 1;

    st = X(all, 0);
    drag = P(Slice(6, 9));
    g(Slice(0, n_state)) = st - P(Slice(0, n_state));
    for(int i = 0; i < N; i++){
        st = X(all, i);
        con = U(all, i);
        obj += SX::mtimes({(st-P(Slice(n_state, n_state*2))).T(), Q, (st-P(Slice(n_state, n_state*2)))}) +
                SX::mtimes({con.T(), R, con});
        st_next = X(all, i+1);
        // RK4
        k1 = Dyna_Func(st, con, drag);
        k2 = Dyna_Func(st + h/2*k1, con, drag);
        k3 = Dyna_Func(st + h/2*k2, con, drag);
        k4 = Dyna_Func(st + h*k3, con, drag);
        st_next_RK4 = st + h/6*(k1+k2+k3+k4);
        // compute constraints 
        g(Slice((i+1)*n_state+i*3, (i+2)*n_state+i*3)) = st_next - st_next_RK4;
        g(Slice((i+2)*n_state+i*3, (i+2)*n_state+i*3+1)) = U(1,i)-U(5,i);
        g(Slice((i+2)*n_state+i*3+1, (i+2)*n_state+i*3+2)) = U(3,i)-U(7,i);
        g(Slice((i+2)*n_state+i*3+2, (i+2)*n_state+i*3+3)) = U(8,i)*U(9,i);
    }
    // make the decision variable one column vector
    OPT_variables = SX::vertcat({SX::reshape(X, n_state*(N+1), 1), SX::reshape(U, 10*N, 1)});
    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}, {"p", P}};
    Dict opts = { {"ipopt.max_iter", 99}, {"ipopt.print_level", 1}, {"print_time", 0}, 
                    {"ipopt.acceptable_tol", 1e-3}, {"ipopt.acceptable_obj_change_tol", 1e-3}};
    solver = nlpsol("solver", "ipopt", nlp, opts);

    // Initial guess and bounds for the optimization variables
    // x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -85.0, 0.0, 0.0, 0.0}; // initial state
    // xs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -85.0, 0.0, 0.0, 0.0}; // desire state
    x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initial state
    xs = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0}; // desire state
    X0 = SX::repmat(x0, 1, N+1);

    state_lower_bound = {-pi/2, -pi/2, -pi/2, -inf, -inf, -inf};
    state_upper_bound = {pi/2, pi/2, pi/2, inf, inf, inf};

    con_lower_bound = {0, -50, 0, -50, 0, -50, 0, -50, 0, 0};
    con_upper_bound = {50, 50, 50, 50, 50, 50, 50, 50,0, 0};
    con_lower_bound_n = {0, 0, 0, -50, 0, 0, 0, -50, -pi/2, 0};
    con_upper_bound_n = {50, 50, 50, 50, 50, 50, 50, 50, 0, pi/2};
    con_lower_bound_p = {0, -50, 0, 0, 0, -50, 0, 0, 0 , -pi/2};
    con_upper_bound_p = {50, 50, 50, 50, 50, 50, 50, 50, pi/2, 0};
    for(int i = 0; i < N+1; i++){
        lbx.insert(lbx.end(), state_lower_bound.begin(), state_lower_bound.end());
        ubx.insert(ubx.end(), state_upper_bound.begin(), state_upper_bound.end());
    }
    for(int i = 0; i < N; i++){
        lbx.insert(lbx.end(), con_lower_bound.begin(), con_lower_bound.end());
        ubx.insert(ubx.end(), con_upper_bound.begin(), con_upper_bound.end());
    }
    lbg.insert(lbg.begin(), n_state*(N+1)+3*N, 0);
    ubg.insert(ubg.begin(), n_state*(N+1)+3*N, 0);

    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    updatePara(0, 0, 0); // set the values of the parameters vector
}

MPC_CTL::~MPC_CTL(){
}

void MPC_CTL::solve(){
    arg["p"] = para;
    // cout << " x0[0] - xs[0] : " << x0[0] - xs[0] << endl;
    // cout << " x0[1] - xs[1] : " << x0[1] - xs[1] << endl;
    // cout << " x0[2] - xs[2] : " << x0[2] - xs[2] << endl;
    if((x0[0]-xs[0]<0.05) && (x0[0]-xs[0]>-0.05)){
        // cout << "-0.1 < x0[0]-xs[0] <0.1" << endl;
        for(int i = 0; i < N; i++){
            std::copy(con_lower_bound.begin(), con_lower_bound.end(), 
                lbx.begin() + (N+1) * state_lower_bound.size() + i * con_lower_bound.size());
            std::copy(con_upper_bound.begin(), con_upper_bound.end(), 
                ubx.begin() + (N+1) * state_upper_bound.size() + i * con_upper_bound.size());
        }
    }
    else if(x0[0]-xs[0]>=0.05){
        // cout << "x0[0]-xs[0] >= 0.1" << endl;
        for(int i = 0; i < N; i++){
            std::copy(con_lower_bound_p.begin(), con_lower_bound_p.end(), 
                lbx.begin() + (N+1) * state_lower_bound.size() + i * con_lower_bound_p.size());
            std::copy(con_upper_bound_p.begin(), con_upper_bound_p.end(), 
                ubx.begin() + (N+1) * state_upper_bound.size() + i * con_upper_bound_p.size());
        }
    }
    else {
        // cout << "x0[0]-xs[0] <= -0.1" << endl;
        for(int i = 0; i < N; i++){
            std::copy(con_lower_bound_n.begin(), con_lower_bound_n.end(), 
                lbx.begin() + (N+1) * state_lower_bound.size() + i * con_lower_bound_n.size());
            std::copy(con_upper_bound_n.begin(), con_upper_bound_n.end(), 
                ubx.begin() + (N+1) * state_upper_bound.size() + i * con_upper_bound_n.size());
        }
    }
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    // initial value of the optimization variables
    arg["x0"] = SX::vertcat({SX::reshape(X0.T(), n_state*(N+1), 1),
                                SX::reshape(u0.T(), 10*N, 1)});
    res = solver(arg);
    getFirstCon();
}

void MPC_CTL::updatePara(double touque1, double touque2, double touque3){
    para.clear();
    para.insert(para.end(), x0.begin(), x0.end());
    para.insert(para.end(), xs.begin(), xs.end());
    para.push_back(touque1);
    para.push_back(touque2);
    para.push_back(touque3);
}

void MPC_CTL::getFirstCon(){
    u_f.clear();
    vector<double> u_f_ = res.at("x").get_elements();
    u_f.insert(u_f.begin(), u_f_.begin()+n_state*(N+1), u_f_.begin()+10+n_state*(N+1));
    // cout << u_f << endl;
    static haique_msgs::controlpub_msg uf_msg;
    uf_msg.thrust1 = u_f[0];
    uf_msg.thrust2 = u_f[1];
    uf_msg.thrust3 = u_f[2];
    uf_msg.thrust4 = u_f[3];
    uf_msg.thrust5 = u_f[4];
    uf_msg.thrust6 = u_f[5];
    uf_msg.thrust7 = u_f[6];
    uf_msg.thrust8 = u_f[7];
    uf_msg.alpha = u_f[8];
    uf_msg.beta = u_f[9];
    conPub.publish(uf_msg);
}

void MPC_CTL::updatex0(double phi_, double theta_, double psi_, double p_, double q_, double r_){
    x0.clear();
    x0.push_back(phi_);
    x0.push_back(theta_);
    x0.push_back(psi_);
    x0.push_back(p_);
    x0.push_back(q_);
    x0.push_back(r_);
}

void MPC_CTL::updatexs(){

}