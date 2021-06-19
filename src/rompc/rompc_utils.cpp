/** 
	@file rompc_utils.cpp
	Utility functions to support the ROMPC controller.
*/

#include <cmath>
#include <Eigen/Geometry>

#include <rompc/rompc_utils.hpp>
#include <utils/rotation.hpp>
#include <utils/data.hpp>

/** 
    @brief Constructor, initialize target class 
*/ 
ROMPC_UTILS::Target::Target() {  
    _p_r_i_I.setZero(); 
    _v_r_I_R.setZero(); 
    _q_I_to_R.setZero(); 
    _om_R_I_R.setZero(); 
    _euler.setZero(); 
    _aa_I_to_R.setZero();
}

/**
    @brief Returns the position of the target w.r.t the 
    inertial frame, in inertial frame coordinates
*/
Vec3 ROMPC_UTILS::Target::get_pos(double t) {
    return _p_r_i_I;
}

/**
    @brief Returns the target inertial velocity in 
    target frame coordinates
*/
Vec3 ROMPC_UTILS::Target::get_vel(double t) {
    return _v_r_I_R;
}

/**
    @brief Returns attitude quaternion of FRD target 
    w.r.t inertial NED frame
*/
Vec4 ROMPC_UTILS::Target::get_att_quat(double t) {
    return _q_I_to_R;
}

/**
    @brief Returns attitude euler angles of FRD target
    w.r.t inertial NED frame
*/
Vec3 ROMPC_UTILS::Target::get_att_euler(double t) {
    return _euler;
}

/**
    @brief Returns attitude axis/angle of FRD target
    w.r.t inertial NED frame
*/
Vec3 ROMPC_UTILS::Target::get_att_aa(double t) {
    return _aa_I_to_R;
}

/**
    @brief Returns target body rates w.r.t inertial
    frame, written in target FRD frame coordinates
*/
Vec3 ROMPC_UTILS::Target::get_om(double t) {
    return _om_R_I_R;
}

/**
    @brief Constructor, initialize glideslope target class

    @param[in] S      target speed [m/s]
    @param[in] gamma  target glideslope angle [rad] (negative = decrease altitude)
    @param[in] th     target pitch angle [rad]
*/
ROMPC_UTILS::SGF::SGF(double S, double gamma, double th): Target(),  _th(th) { 
    // Velocity (u,v,w) in target frame coordinates, constant
    _v_r_I_R(0) = S * cos(th - gamma);
    _v_r_I_R(1) = 0.0;
    _v_r_I_R(2) = S * sin(th - gamma);

    // Velocity in inertial coordinates
    _v_r_I_I.setZero();
    _v_r_I_I(2) = S * sin(-gamma); // sink rate, constant
    _S_xy = S * cos(gamma);
}

/**
    @brief Initialize the target position and yaw angle.

    @param[in] p    inertial position [m]
    @param[in] psi  yaw angle [rad]
*/
void ROMPC_UTILS::SGF::initialize(Vec3 p, double psi) {
    // Set initial position
    _p_r_i_I = p;
    
    // Determine velocities in x and y direction in inertial coord.
    _v_r_I_I(0) = _S_xy * cos(psi);
    _v_r_I_I(1) = _S_xy * sin(psi);

    // Compute orientations
    _euler << 0.0, _th, psi;
    Rot::euler_to_quat(_euler, _q_I_to_R);
    Rot::quat_to_axis(_q_I_to_R, _aa_I_to_R);
}

/**
    @brief Position computed by taking starting position
    and integrating inertial velocity

    @param[in] t  time since initialization [s]
*/
Vec3 ROMPC_UTILS::SGF::get_pos(double t) {
    return _p_r_i_I + t * _v_r_I_I;    
}

/**
    @brief Constructor, initialize steady turning flight target class

    @param[in] v      target constant body frame velocity [m/s]
    @param[in] om     target angular velocity in body frame [rad/s] 
    @param[in] phi    target roll angle [rad]
    @param[in] th     target pitch angle [rad]
    @param[in] R      target turning radius [m]
*/
ROMPC_UTILS::STF::STF(Vec3 v, Vec3 om, double phi,  double th, double R): 
                      Target(), _phi(phi), _th(th), _R(R) {
    _v_r_I_R = v;
    _om_R_I_R = om;
    _psi_dot = v.norm()/R; // compute constant yaw rate
}

/**
    @brief Initialize the target position and yaw angle.

    @param[in] p    inertial position [m]
    @param[in] yaw  initial yaw angle [rad]
*/
void ROMPC_UTILS::STF::initialize(Vec3 p, double psi) {
    _p_r_i_I = p; // initial position
    _psi = psi; // initial yaw angle

    // Compute center of circle, a constant
    _p_c_i_I << -_R*sin(_psi), _R*cos(_psi), 0.0;
    _p_c_i_I += _p_r_i_I;
}

/**
    @brief Position computed by integrating yaw rate

    @param[in] t  time since initialization [s]
*/
Vec3 ROMPC_UTILS::STF::get_pos(double t) {
    double psi = _psi + t * _psi_dot;
    _p_r_i_I << _R*sin(psi), -_R*cos(psi), 0.0;
    _p_r_i_I += _p_c_i_I;
    return _p_r_i_I;
}

/**
    @brief Quaternion attitude of target FRD frame
    w.r.t inertial NED frame

    @param[in] t  time since initialization [s]
*/
void ROMPC_UTILS::STF::compute_euler(double t) {
    double psi = Rot::wrap_to_2pi(_psi + t * _psi_dot);
    _euler << _phi, _th, psi;
}

/**
    @brief Quaternion attitude of target FRD frame
    w.r.t inertial NED frame

    @param[in] t  time since initialization [s]
*/
Vec4 ROMPC_UTILS::STF::get_att_quat(double t) {
    compute_euler(t);
    Rot::euler_to_quat(_euler, _q_I_to_R);
    return _q_I_to_R;
}

/**
    @brief Euler attitude of target FRD frame
    w.r.t inertial NED frame

    @param[in] t  time since initialization [s]
*/
Vec3 ROMPC_UTILS::STF::get_att_euler(double t) {
    compute_euler(t);
    return _euler;
}

/**
    @brief Axis/angle attitude of target FRD frame
    w.r.t inertial NED frame

    @param[in] t  time since initialization [s]
*/
Vec3 ROMPC_UTILS::STF::get_att_aa(double t) {
    compute_euler(t);
    Rot::euler_to_quat(_euler, _q_I_to_R);
    Rot::quat_to_axis(_q_I_to_R, _aa_I_to_R);
    return _aa_I_to_R;
}

/** 
    @brief Constructor, initialize OCP object

    @param[in] filepath  path to where the QP matrices are
    @param[in] m         dimension of control vector
*/ 
ROMPC_UTILS::OCP::OCP(const std::string filepath, const double tmax)
                      : _success(false), _solve_time(0.0), _tmax(tmax) {  
    MatX F = Data::load_matrix(filepath + "/F.csv");
    _G = Data::load_matrix(filepath + "/G.csv");
    MatX E = Data::load_matrix(filepath + "/E.csv");
    VecX ubE = Data::load_matrix(filepath + "/ub.csv");
    VecX params = Data::load_matrix(filepath + "/params.csv");
    
    _nV = E.cols();
    _nC = E.rows();
    _n = _G.rows();
    _dt = params(1); // discretization time in seconds
    _N = params(0); // horizon steps

    // Initialize QP problem
    _ocp = qpOASES::QProblem(_nV, _nC);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    _ocp.setOptions(options);

    // Define constant problem data structures
    _F = ArrPtr(new qpOASES::real_t[_nV*_nV]);
    _E = ArrPtr(new qpOASES::real_t[_nC*_nV]);
    _ubE = ArrPtr(new qpOASES::real_t[_nC]);
    _g = ArrPtr(new qpOASES::real_t[_nV]);
    _U = ArrPtr(new qpOASES::real_t[_nV]);

    eigen_to_qpoases(F, _F);
    eigen_to_qpoases(E, _E);
    eigen_to_qpoases(ubE, _ubE);

    Eigen::VectorXd x0;
    x0.resize(_n);
    x0.setZero();
    set_x0(x0);

    qpOASES::int_t nWSR = 1000;
    qpOASES::real_t cputime = _tmax;
    qpOASES::returnValue st = _ocp.init(_F.get(), _g.get(), _E.get(), nullptr, nullptr, 
                              nullptr, _ubE.get(), nWSR, &cputime);

    if (st == qpOASES::SUCCESSFUL_RETURN) _success = true;
    else _success = false;
    
    _solve_time = cputime;
}

/**
    @brief Converts a Eigen::MatrixXd matrix into an array
    assuming row major ordering used by qpOASES
*/
void ROMPC_UTILS::OCP::eigen_to_qpoases(const MatX& M, ArrPtr& m) {
    RowMajMat::Map(m.get(), M.rows(), M.cols()) = M;
}

/**
    @brief Converts a Eigen::MatrixXd matrix into an array
    assuming row major ordering used by qpOASES
*/
void ROMPC_UTILS::OCP::set_x0(const VecX x0) {
    eigen_to_qpoases(x0.transpose()*_G, _g);
}

/**
    @brief Converts a Eigen::MatrixXd matrix into an array
    assuming row major ordering used by qpOASES

    @param[in] x0        Initial condition
    @param[in] uopt      Writes u_0* to this vector

    @param[out] success  boolean if solved successfully
*/
void ROMPC_UTILS::OCP::solve(const VecX x0, Vec4& uopt) {
    set_x0(x0); // Set the new initial condition

    qpOASES::int_t nWSR = 1000;
    qpOASES::real_t comptime = _tmax;
    qpOASES::returnValue st = _ocp.hotstart(_g.get(), nullptr, nullptr, 
                                        nullptr, _ubE.get(), nWSR, &comptime);

    if (st == qpOASES::SUCCESSFUL_RETURN) {
        _ocp.getPrimalSolution(_U.get());
        uopt << _U[0], _U[1], _U[2], _U[3];
        _success = true;
    }
    else {
        uopt.setZero();
        _success = false;
    }

    _solve_time = comptime;
}

/**
    @brief Get most recent status of OCP solver
*/
bool ROMPC_UTILS::OCP::success() {
    return _success;    
}

/**
    @brief Get most recent QP solve time
*/
double ROMPC_UTILS::OCP::solve_time() {
    return _solve_time;
}

double ROMPC_UTILS::OCP::get_dt() {
    return _dt;
}

int ROMPC_UTILS::OCP::get_N() {
    return _N;
}

/**
    @brief Computes operator T such that om = T(aa) * aa_dot which
    transforms the rate of change of axis/angle representation of
    rotation from A to B into the angular velocity of B w.r.t A written
    in B coordinates.

    @param[in] aa  axis angle representation aa = (th1, th2, th3)
    @param[in] T   transformation operator matrix
*/
void ROMPC_UTILS::tangential_transf(const Vec3& aa, Mat3& T) {
    double th2 = aa.squaredNorm(); // theta squared

    double c1, c2, c3;
    if (th2 < 5e-6) {
        double th4 = th2 * th2;
        c1 = 1.0  - th2/6   + th4/120;  // + O(th^6)
        c2 = 1/2. - th2/24  + th4/720;  // + O(th^6)
        c3 = 1/6. - th2/120 + th4/5040; // + O(th^6)
    }
    else {
        double th = sqrt(th2);
        c1 = sin(th)/th;
        c2 = (1 - cos(th))/th2;
        c3 = (th - sin(th))/(th * th2);
    }
    Mat3 skew_th = (Mat3() << 0.0, -aa(2), aa(1),
                              aa(2), 0.0, -aa(0),
                             -aa(1), aa(0), 0.0).finished();

    T = c1*Mat3::Identity() - c2*skew_th + c3*(aa*aa.transpose());
}

/**
    @brief Converts the angular velocity vector om for a frame B rotating w.r.t A
    into the rate of change of the axis/angle parameters that represent the
    rotation from A to B.

    @param[in] aa     axis/angle repr. of rotation from A to B frames [rad]
    @param[in] om     angular velocity of B w.r.t A [rad/s]
    @param[in] aadot  time derivative of aa [rad/s]
*/
void ROMPC_UTILS::om_to_aadot(const Vec3& aa, const Vec3& om, Vec3& aadot) {
    Mat3 T;
    ROMPC_UTILS::tangential_transf(aa, T);
    aadot = T.inverse() * om;
}

/**
    @brief Given current axis/angle parameters for rotation from A to B and their
    time derivatives, computes the angular velocity vector of B w.r.t A.

    @param[in] aa     axis/angle repr. of rotation from A to B frames [rad]
    @param[in] aadot  time derivative of aa [rad/s]
    @param[in] om     angular velocity of B w.r.t A [rad/s]
*/
void ROMPC_UTILS::aadot_to_om(const Vec3& aa, const Vec3& aadot, Vec3& om) {
    Mat3 T;
    ROMPC_UTILS::tangential_transf(aa, T);
    om = T * aadot;
}

